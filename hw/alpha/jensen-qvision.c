/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Jensen EISA Compaq QVision 1024/E, experimental single-controller subset.
 *
 * Bt484 register/cursor behavior follows 86Box
 * src/video/ramdac/vid_ramdac_bt48x.c and the Jensen S3 integration:
 * Copyright 2016-2018 Miran Grca.
 * Copyright 2018 TheCollector1995.
 * VGA core integration follows QEMU's ISA VGA (Fabrice Bellard, 2003).
 * References: DEC EK-A0653-UG.001, original CPQ3011 ECU resource definitions,
 * and the QVision register interfaces documented by RBIL.
 * No vendor ROM or operating-system driver code is included.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/isa/isa.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "../display/vga_int.h"
#include "trace.h"

#define TYPE_JENSEN_QVISION "jensen-qvision-1024"
OBJECT_DECLARE_SIMPLE_TYPE(JensenQVision, JENSEN_QVISION)

typedef struct QVisionPort {
    MemoryRegion mr;
    JensenQVision *state;
    uint16_t base;
} QVisionPort;

struct JensenQVision {
    ISADevice parent_obj;
    VGACommonState vga;
    GraphicHwOps hw_ops;
    MemoryRegion aperture, eisa, rom, highmap;
    QVisionPort standard;
    QVisionPort extra[11];
    uint8_t registers[65536];
    uint8_t board_control, setup, select;
    uint8_t dac_command[3], cursor_ram[256], cursor_palette[12];
    uint8_t dac_address, dac_component;
    uint16_t cursor_x, cursor_y;
    int cursor_old_y;
    bool cursor_old_enabled;
    uint32_t slot;
    char *romfile;
    int (*base_bpp)(VGACommonState *);
    void (*base_params)(VGACommonState *, VGADisplayParams *);
    void (*base_resolution)(VGACommonState *, int *, int *);
};

static const uint16_t qvision_extra_ports[] = {
    0x0102, 0x13c0, 0x23c0, 0x33c0, 0x43c0, 0x46e8,
    0x53c0, 0x63c0, 0x73c0, 0x83c0, 0x93c0,
};

static bool qvision_selected(JensenQVision *s)
{
    return (s->board_control & 1) && (s->select & 15) == (s->setup & 15);
}

static bool qvision_packed(JensenQVision *s)
{
    return s->vga.gr[0x40] & 1;
}

static void qvision_update_map(JensenQVision *s)
{
    uint64_t base = (uint64_t)s->vga.gr[0x48] << 20 |
                    (uint64_t)(s->vga.gr[0x49] & 15) << 28;
    bool enable = qvision_selected(s) && base >= MiB;

    memory_region_transaction_begin();
    memory_region_set_enabled(&s->highmap, false);
    memory_region_set_address(&s->highmap, base);
    memory_region_set_enabled(&s->highmap, enable);
    memory_region_transaction_commit();
    trace_jensen_qvision_map(base, enable);
}

static int qvision_bpp(VGACommonState *vga)
{
    JensenQVision *s = container_of(vga, JensenQVision, vga);

    if (!qvision_packed(s)) {
        return s->base_bpp(vga);
    }
    if (!(s->dac_command[2] & 0x20)) {
        return 8;
    }
    switch (s->dac_command[1] & 0x60) {
    case 0x00:
        return 32;
    case 0x20:
        return s->dac_command[1] & 8 ? 16 : 15;
    case 0x40:
        return 8;
    default:
        return 0;
    }
}

static void qvision_params(VGACommonState *vga, VGADisplayParams *params)
{
    JensenQVision *s = container_of(vga, JensenQVision, vga);

    s->base_params(vga, params);
    params->start_addr |= (vga->gr[0x42] & 0x0c) << 14;
    if (qvision_packed(s)) {
        params->line_compare = 0x7fff;
    }
}

static void qvision_resolution(VGACommonState *vga, int *width, int *height)
{
    JensenQVision *s = container_of(vga, JensenQVision, vga);

    s->base_resolution(vga, width, height);
}

static bool qvision_update_display(void *opaque)
{
    VGACommonState *vga = opaque;
    JensenQVision *s = container_of(vga, JensenQVision, vga);
    uint8_t mode = vga->gr[5];

    if (qvision_packed(s)) {
        vga->gr[5] = (mode & ~0x60) | 0x40;
    }
    vga->hw_ops->gfx_update(vga);
    vga->gr[5] = mode;
    return true;
}

static void qvision_cursor_invalidate(VGACommonState *vga)
{
    JensenQVision *s = container_of(vga, JensenQVision, vga);
    bool enabled = s->dac_command[2] & 3;
    int y = (int)s->cursor_y - 32;

    if (s->cursor_old_enabled) {
        vga_invalidate_scanlines(vga, MAX(0, s->cursor_old_y),
                                 MAX(0, s->cursor_old_y + 32));
    }
    if (enabled) {
        vga_invalidate_scanlines(vga, MAX(0, y), MAX(0, y + 32));
    }
    s->cursor_old_y = y;
    s->cursor_old_enabled = enabled;
}

static void qvision_cursor_line(VGACommonState *vga, uint8_t *dest, int y)
{
    JensenQVision *s = container_of(vga, JensenQVision, vga);
    int x = (int)s->cursor_x - 32;
    int row = y - ((int)s->cursor_y - 32);
    unsigned mode = s->dac_command[2] & 3;
    uint32_t colors[4], *pixels = (uint32_t *)dest;

    if (!mode || row < 0 || row >= 32 ||
        surface_bits_per_pixel(qemu_console_surface(vga->con)) != 32) {
        return;
    }
    for (unsigned i = 0; i < 4; i++) {
        uint8_t *rgb = s->cursor_palette + i * 3;
        colors[i] = vga->dac_8bit ? rgb_to_pixel32(rgb[0], rgb[1], rgb[2]) :
            rgb_to_pixel32(c6_to_8(rgb[0]), c6_to_8(rgb[1]), c6_to_8(rgb[2]));
    }
    for (int i = MAX(0, -x); i < MIN(32, (int)vga->last_scr_width - x); i++) {
        unsigned offset = row * 4 + i / 8, bit = 7 - (i & 7);
        unsigned value = ((s->cursor_ram[offset] >> bit) & 1) |
                         (((s->cursor_ram[offset + 128] >> bit) & 1) << 1);

        if (mode == 1 && value) {
            pixels[x + i] = colors[value];
        } else if (mode == 2 && value < 2) {
            pixels[x + i] = colors[value + 1];
        } else if (mode == 2 && value == 3) {
            pixels[x + i] ^= 0x00ffffff;
        } else if (mode == 3 && (value & 2)) {
            pixels[x + i] = colors[(value & 1) + 1];
        }
    }
}

static int qvision_dac_register(unsigned port)
{
    switch (port) {
    case 0x83c8: return 4;
    case 0x83c9: return 5;
    case 0x83c6: return 6;
    case 0x83c7: return 7;
    case 0x13c8: return 8;
    case 0x13c9: return 9;
    case 0x13c6: return 10;
    case 0x13c7: return 11;
    case 0x93c8: return 12;
    case 0x93c9: return 13;
    case 0x93c6: return 14;
    case 0x93c7: return 15;
    default: return -1;
    }
}

static uint8_t qvision_read_byte(JensenQVision *s, unsigned port)
{
    VGACommonState *vga = &s->vga;
    int dac = qvision_dac_register(port);

    if (port == 0x83c4) {
        return s->select;
    }
    if (!qvision_selected(s)) {
        return 0xff;
    }
    switch (dac) {
    case 4: case 7:
        return s->dac_address;
    case 5: {
        uint8_t value = s->cursor_palette[(s->dac_address & 3) * 3 +
                                         s->dac_component++];
        if (s->dac_component == 3) {
            s->dac_component = 0;
            s->dac_address++;
        }
        return value;
    }
    case 6: return s->dac_command[0];
    case 8: return s->dac_command[1];
    case 9: return s->dac_command[2];
    case 10: return 0x40; /* Bt484 identification/status. */
    case 11: return s->cursor_ram[s->dac_address++];
    case 12: return s->cursor_x;
    case 13: return s->cursor_x >> 8;
    case 14: return s->cursor_y;
    case 15: return s->cursor_y >> 8;
    }
    if (port >= 0x3b0 && port <= 0x3df) {
        return vga_ioport_read(vga, port);
    }
    return s->registers[port];
}

static void qvision_write_byte(JensenQVision *s, unsigned port, uint8_t value)
{
    VGACommonState *vga = &s->vga;
    unsigned index = vga->gr_index;
    int dac = qvision_dac_register(port);

    if (port == 0x83c4) {
        s->select = value & 15;
        qvision_update_map(s);
        return;
    }
    if (!qvision_selected(s)) {
        return;
    }
    switch (dac) {
    case 4: case 7:
        s->dac_address = value;
        s->dac_component = 0;
        return;
    case 5:
        s->cursor_palette[(s->dac_address & 3) * 3 + s->dac_component++] = value;
        if (s->dac_component == 3) {
            s->dac_component = 0;
            s->dac_address++;
        }
        return;
    case 6:
        s->dac_command[0] = value;
        vga->dac_8bit = !!(value & 2);
        vga->full_update_gfx = true;
        return;
    case 8: s->dac_command[1] = value; return;
    case 9: s->dac_command[2] = value; return;
    case 10: return; /* Status is read-only on Bt484. */
    case 11: s->cursor_ram[s->dac_address++] = value; return;
    case 12: s->cursor_x = (s->cursor_x & 0xf00) | value; return;
    case 13: s->cursor_x = (s->cursor_x & 0xff) | ((value & 15) << 8); return;
    case 14: s->cursor_y = (s->cursor_y & 0xf00) | value; return;
    case 15: s->cursor_y = (s->cursor_y & 0xff) | ((value & 15) << 8); return;
    }
    if (port == 0x3ce) {
        vga->gr_index = value;
        return;
    }
    if (port == 0x3cf && index >= 9) {
        if (index == 0x0c) {
            return; /* Controller identification is read-only. */
        }
        if (index != 0x0f && (vga->gr[0x0f] & 15) != 5) {
            return;
        }
        if (index >= 0x40 && !(vga->gr[0x10] & 8)) {
            return;
        }
        vga->gr[index] = value;
        if (index == 0x48 || index == 0x49) {
            qvision_update_map(s);
        }
        vga->full_update_gfx = vga->full_update_text = true;
        return;
    }
    /* Stopping the VGA sequencer also drops AVGA and the high-address map.
     * The original QVision console driver relies on this when restoring text.
     */
    if (port == 0x3c5 && vga->sr_index == 0 && (value & 3) != 3) {
        vga->gr[0x40] &= ~1;
        vga->gr[0x48] = vga->gr[0x49] = 0;
        qvision_update_map(s);
    }
    if (port == 0x3c7 || port == 0x3c8) {
        s->dac_address = value;
        s->dac_component = 0;
    }
    if (port >= 0x3b0 && port <= 0x3df) {
        vga_ioport_write(vga, port, value);
        return;
    }
    s->registers[port] = value;
    if ((port & 0xfff0) != 0x63c0 && port != 0x0102 && port != 0x46e8) {
        qemu_log_mask(LOG_UNIMP, "QVision: unimplemented engine write %04x=%02x\n",
                      port, value);
    }
}

static uint64_t qvision_port_read(void *opaque, hwaddr addr, unsigned size)
{
    QVisionPort *p = opaque;
    uint64_t value = 0;

    for (unsigned i = 0; i < size; i++) {
        value |= (uint64_t)qvision_read_byte(p->state, p->base + addr + i) <<
                 (i * 8);
    }
    trace_jensen_qvision_read(p->base + addr, value, size);
    return value;
}

static void qvision_port_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    QVisionPort *p = opaque;

    trace_jensen_qvision_write(p->base + addr, value, size);
    for (unsigned i = 0; i < size; i++) {
        qvision_write_byte(p->state, p->base + addr + i, value >> (i * 8));
    }
}

static const MemoryRegionOps qvision_port_ops = {
    .read = qvision_port_read,
    .write = qvision_port_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
    .impl = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
};

static uint64_t qvision_memory_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenQVision *s = opaque;

    return qvision_selected(s) ? vga_mem_readb(&s->vga, addr) : 0xff;
}

static void qvision_memory_write(void *opaque, hwaddr addr,
                                 uint64_t value, unsigned size)
{
    JensenQVision *s = opaque;

    if (qvision_selected(s)) {
        vga_mem_writeb(&s->vga, addr, value);
    }
}

static const MemoryRegionOps qvision_memory_ops = {
    .read = qvision_memory_read,
    .write = qvision_memory_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4, .unaligned = true },
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static uint64_t qvision_eisa_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenQVision *s = opaque;
    uint8_t value;
    static const uint8_t id[] = { 0x0e, 0x11, 0x30, 0x11 };

    value = addr < 4 ? id[addr] : addr == 4 ? s->board_control :
            addr == 5 ? s->setup : 0;
    trace_jensen_qvision_read((s->slot << 12) + 0xc80 + addr, value, size);
    return value;
}

static void qvision_eisa_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned size)
{
    JensenQVision *s = opaque;

    trace_jensen_qvision_write((s->slot << 12) + 0xc80 + addr, value, size);
    if (addr == 4) {
        s->board_control = value & 1;
    } else if (addr == 5) {
        s->setup = value & 0x1f;
    }
    qvision_update_map(s);
}

static const MemoryRegionOps qvision_eisa_ops = {
    .read = qvision_eisa_read,
    .write = qvision_eisa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static void qvision_reset(DeviceState *dev)
{
    JensenQVision *s = JENSEN_QVISION(dev);

    vga_common_reset(&s->vga);
    memset(s->registers, 0, sizeof(s->registers));
    memset(s->dac_command, 0, sizeof(s->dac_command));
    memset(s->cursor_ram, 0, sizeof(s->cursor_ram));
    memset(s->cursor_palette, 0, sizeof(s->cursor_palette));
    s->board_control = 1;
    s->setup = 0x10;
    s->select = s->dac_address = s->dac_component = 0;
    s->cursor_x = s->cursor_y = 0;
    s->cursor_old_enabled = false;
    s->cursor_old_y = 0;
    s->vga.gr[0x0c] = 0x37; /* Early QVision 1024 (RBIL P0707). */
    qvision_update_map(s);
}

static void qvision_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    JensenQVision *s = JENSEN_QVISION(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    VGACommonState *vga = &s->vga;
    QVisionPort *p;

    if (s->slot < 1 || s->slot > 6 || !s->romfile) {
        error_setg(errp, "QVision requires EISA slot 1..6 and an external ROM");
        return;
    }
    vga->vram_size_mb = 1;
    if (!vga_common_init(vga, OBJECT(dev), errp)) {
        return;
    }
    s->base_bpp = vga->get_bpp;
    s->base_params = vga->get_params;
    s->base_resolution = vga->get_resolution;
    vga->get_bpp = qvision_bpp;
    vga->get_params = qvision_params;
    vga->get_resolution = qvision_resolution;
    vga->force_shadow = true;
    vga->cursor_invalidate = qvision_cursor_invalidate;
    vga->cursor_draw_line = qvision_cursor_line;
    /* Do not install Bochs VBE ports or a generic chain-4 alias. */
    memory_region_init_io(&s->aperture, OBJECT(dev), &qvision_memory_ops, s,
                          "qvision-memory", 0x20000);
    memory_region_add_subregion_overlap(isa_address_space(isa), 0xa0000,
                                        &s->aperture, 1);
    p = &s->standard;
    p->state = s;
    p->base = 0x3b0;
    memory_region_init_io(&p->mr, OBJECT(dev), &qvision_port_ops, p,
                          "qvision-vga", 0x30);
    memory_region_add_subregion(isa_address_space_io(isa), 0x3b0, &p->mr);
    for (unsigned i = 0; i < ARRAY_SIZE(s->extra); i++) {
        p = &s->extra[i];
        p->state = s;
        p->base = qvision_extra_ports[i];
        memory_region_init_io(&p->mr, OBJECT(dev), &qvision_port_ops, p,
                              "qvision-extended", p->base == 0x0102 ||
                              p->base == 0x46e8 ? 1 : 16);
        memory_region_add_subregion(isa_address_space_io(isa), p->base, &p->mr);
    }
    memory_region_init_io(&s->eisa, OBJECT(dev), &qvision_eisa_ops, s,
                          "qvision-eisa", 16);
    memory_region_add_subregion(isa_address_space_io(isa),
                                (s->slot << 12) + 0xc80, &s->eisa);
    memory_region_init_alias(&s->highmap, OBJECT(dev), "qvision-highmap",
                             &vga->vram, 0, MiB);
    memory_region_set_enabled(&s->highmap, false);
    memory_region_add_subregion(isa_address_space(isa), 0, &s->highmap);
    memory_region_init_rom(&s->rom, OBJECT(dev), "qvision-rom", 32 * KiB, errp);
    if (*errp) {
        return;
    }
    memory_region_add_subregion(isa_address_space(isa), 0xc0000, &s->rom);
    if (rom_add_file_mr(s->romfile, &s->rom, -1) < 0) {
        error_setg(errp, "Could not load QVision ROM '%s'", s->romfile);
        return;
    }
    s->hw_ops = *vga->hw_ops;
    s->hw_ops.gfx_update = qvision_update_display;
    vga->con = qemu_graphic_console_create(dev, 0, &s->hw_ops, vga);
    qvision_reset(dev);
}

static const Property qvision_properties[] = {
    DEFINE_PROP_UINT32("slot", JensenQVision, slot, 1),
    DEFINE_PROP_STRING("romfile", JensenQVision, romfile),
};

static const VMStateDescription qvision_vmstate = {
    .name = TYPE_JENSEN_QVISION,
    .unmigratable = true,
};

static void qvision_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = qvision_realize;
    dc->vmsd = &qvision_vmstate;
    dc->user_creatable = false;
    device_class_set_legacy_reset(dc, qvision_reset);
    device_class_set_props(dc, qvision_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo qvision_info = {
    .name = TYPE_JENSEN_QVISION,
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(JensenQVision),
    .class_init = qvision_class_init,
};

static void qvision_register_types(void)
{
    type_register_static(&qvision_info);
}

type_init(qvision_register_types)
