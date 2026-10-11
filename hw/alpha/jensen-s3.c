/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Jensen ISA S3 86C928, experimental graphics subset.
 *
 * Register decoding, locking and raster operations adapted from 86Box
 * src/video/vid_s3.c and src/video/ramdac/vid_ramdac_bt48x.c:
 * Copyright 2008-2019 Sarah Walker.
 * Copyright 2016-2019 Miran Grca.
 * Copyright 2018 TheCollector1995.
 * Adapted to QEMU's VGA core and synchronous, bounded command execution.
 * Reference: S3 DB02-C, 86C928 GUI Accelerator, September 1992.
 * No vendor option ROM is included; the board retains its VGA console ROM.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "hw/isa/isa.h"
#include "../display/vga_int.h"
#include "trace.h"

#define TYPE_JENSEN_S3 "jensen-s3-928"
OBJECT_DECLARE_SIMPLE_TYPE(JensenS3State, JENSEN_S3)

typedef struct JensenS3Port {
    MemoryRegion mr;
    JensenS3State *state;
    unsigned port;
} JensenS3Port;

struct JensenS3State {
    ISADevice parent_obj;
    VGACommonState vga;
    GraphicHwOps hw_ops;
    MemoryRegion ports, aperture;
    JensenS3Port accel[18];
    int (*vga_bpp)(VGACommonState *);
    void (*vga_params)(VGACommonState *, VGADisplayParams *);
    void (*vga_resolution)(VGACommonState *, int *, int *);
    uint16_t reg[16], multi[16], multi_latch;
    uint8_t dac_command[4], cursor_ram[1024], cursor_palette[12];
    uint16_t dac_addr, cursor_x, cursor_y;
    uint8_t dac_component;
    uint32_t progress;
    bool pending, unsupported;
};

static const uint16_t s3_ports[] = {
    0x42e8, 0x4ae8, 0x82e8, 0x86e8, 0x8ae8, 0x8ee8,
    0x92e8, 0x96e8, 0x9ae8, 0x9ee8, 0xa2e8, 0xa6e8,
    0xaae8, 0xaee8, 0xb6e8, 0xbae8,
};

enum {
    SUBSYS, ADVFUNC, CUR_Y, CUR_X, DEST_Y, DEST_X, ERROR_TERM,
    MAJOR, COMMAND, SHORT_STROKE, BACKGROUND, FOREGROUND,
    WRITE_MASK, READ_MASK, BACK_MIX, FORE_MIX,
};

static bool s3_packed(JensenS3State *s)
{
    return s->vga.cr[0x31] & 8;
}

static unsigned s3_pitch(JensenS3State *s)
{
    unsigned offset = s->vga.cr[0x13] | ((s->vga.cr[0x51] & 0x30) << 4);
    return offset * 8;
}

/* Enhanced scanout bypasses the VGA Graphics Controller shift mode. */
static void s3_update_display(void *opaque)
{
    VGACommonState *vga = opaque;
    JensenS3State *s = container_of(vga, JensenS3State, vga);
    uint8_t mode = vga->gr[5];

    if (s3_packed(s) &&
        ((s->reg[ADVFUNC] & 1) || (vga->cr[0x40] & 8))) {
        vga->gr[5] = (mode & ~0x60) | 0x40;
    }
    /* The VGA renderer is synchronous under the BQL. Keep guest readback. */
    vga->hw_ops->gfx_update(vga);
    vga->gr[5] = mode;
}

static int s3_bpp(VGACommonState *vga)
{
    JensenS3State *s = container_of(vga, JensenS3State, vga);
    if (!s3_packed(s)) {
        return s->vga_bpp(vga);
    }
    /* The accelerated model currently implements only indexed 8-bit pixels. */
    return 8;
}

static void s3_params(VGACommonState *vga, VGADisplayParams *params)
{
    JensenS3State *s = container_of(vga, JensenS3State, vga);
    s->vga_params(vga, params);
    if (s3_packed(s)) {
        params->line_offset = s3_pitch(s);
        params->start_addr |= ((vga->cr[0x31] & 0x30) << 12) |
                              ((vga->cr[0x51] & 3) << 18);
    }
}

static void s3_resolution(VGACommonState *vga, int *width, int *height)
{
    JensenS3State *s = container_of(vga, JensenS3State, vga);
    s->vga_resolution(vga, width, height);
    if (s3_packed(s)) {
        *width += (vga->cr[0x5d] & 2) << 10;
        *height += (vga->cr[0x5e] & 2) << 9;
    }
}

/* The sixteen S3 mix functions, independent of the selected pixel source. */
static uint16_t s3_mix(unsigned mix, uint16_t src, uint16_t dst)
{
    switch (mix & 15) {
    case 0:
        return ~dst;
    case 1:
        return 0;
    case 2:
        return 0xffff;
    case 3:
        return dst;
    case 4:
        return ~src;
    case 5:
        return src ^ dst;
    case 6:
        return ~(src ^ dst);
    case 7:
        return src;
    case 8:
        return ~(src & dst);
    case 9:
        return ~src | dst;
    case 10:
        return src | ~dst;
    case 11:
        return src | dst;
    case 12:
        return src & dst;
    case 13:
        return src & ~dst;
    case 14:
        return ~src & dst;
    case 15:
        return ~(src | dst);
    }
    g_assert_not_reached();
}

static unsigned s3_engine_pitch(JensenS3State *s)
{
    /* CR50 sets the engine stride independently of the display offset. */
    switch (s->vga.cr[0x50] & 0xc1) {
    case 0x00:
        return s->vga.cr[0x31] & 2 ? 2048 : 1024;
    case 0x01:
        return 1152;
    case 0x40:
        return 640;
    case 0x80:
        return 800;
    case 0x81:
        return 1600;
    case 0xc0:
        return 1280;
    default:
        return 1024;
    }
}

static unsigned s3_pixel_bytes(JensenS3State *s)
{
    return s->vga.cr[0x50] & 0x10 ? 2 : 1;
}

static uint32_t s3_address(JensenS3State *s, int x, int y, bool source)
{
    unsigned base = (s->multi[14] >> (source ? 2 : 0)) & 3;
    return (base * MiB + (y * s3_engine_pitch(s) + x) * s3_pixel_bytes(s)) &
           (s->vga.vram_size - 1);
}

static void s3_pixel(JensenS3State *s, int x, int y, uint16_t source,
                     bool foreground, uint16_t cpu)
{
    unsigned mix = s->reg[foreground ? FORE_MIX : BACK_MIX];
    uint32_t addr;
    unsigned bytes = s3_pixel_bytes(s);
    uint16_t old, value, mask = s->reg[WRITE_MASK];
    bool inside = x >= s->multi[2] && x <= s->multi[4] &&
                  y >= s->multi[1] && y <= s->multi[3];
    if (!(s->reg[COMMAND] & 0x10) || inside == !!(s->multi[14] & 0x20)) {
        return;
    }
    switch ((mix >> 5) & 3) {
    case 0:
        source = s->reg[BACKGROUND];
        break;
    case 1:
        source = s->reg[FOREGROUND];
        break;
    case 2:
        source = cpu;
        break;
    case 3:
        break;
    }
    addr = s3_address(s, x, y, false);
    old = bytes == 2 ? lduw_le_p(s->vga.vram_ptr + addr) :
                       s->vga.vram_ptr[addr];
    value = s3_mix(mix, source, old);
    value = (old & ~mask) | (value & mask);
    if (bytes == 2) {
        stw_le_p(s->vga.vram_ptr + addr, value);
    } else {
        s->vga.vram_ptr[addr] = value;
    }
    memory_region_set_dirty(&s->vga.vram, addr, bytes);
}

static void s3_step(JensenS3State *s, uint16_t cpu, bool foreground)
{
    unsigned width = (s->reg[MAJOR] & 0xfff) + 1;
    int dx = s->reg[COMMAND] & 0x20 ? 1 : -1;
    int dy = s->reg[COMMAND] & 0x80 ? 1 : -1;
    int x = (s->reg[CUR_X] & 0xfff) + (s->progress % width) * dx;
    int y = (s->reg[CUR_Y] & 0xfff) + (s->progress / width) * dy;
    uint16_t source = 0;
    unsigned op = s->reg[COMMAND] >> 13;
    if (op == 6 || op == 7) {
        uint32_t addr;
        int dest_x = (s->reg[DEST_X] & 0xfff) + (s->progress % width) * dx;
        int dest_y = (s->reg[DEST_Y] & 0xfff) + (s->progress / width) * dy;
        if (op == 7) {
            x = (s->reg[CUR_X] & 0xfff) + (dest_x & 7);
            y = (s->reg[CUR_Y] & 0xfff) + (dest_y & 7);
        }
        addr = s3_address(s, x, y, true);
        source = s3_pixel_bytes(s) == 2 ?
                 lduw_le_p(s->vga.vram_ptr + addr) : s->vga.vram_ptr[addr];
        x = dest_x;
        y = dest_y;
        if ((s->multi[10] & 0xc0) == 0xc0) {
            foreground = (source & s->reg[READ_MASK]) == s->reg[READ_MASK];
            source = foreground;
        }
    }
    s3_pixel(s, x, y, source, foreground, cpu);
    s->progress++;
    if (s->progress == width * ((s->multi[0] & 0xfff) + 1)) {
        s->pending = false;
    }
}

static void s3_command(JensenS3State *s)
{
    unsigned op = s->reg[COMMAND] >> 13;
    s->progress = 0;
    s->pending = false;
    s->unsupported = false;
    trace_jensen_s3_command(s->reg[COMMAND], s->reg[CUR_X], s->reg[CUR_Y],
                            s->reg[MAJOR] + 1, s->multi[0] + 1);
    trace_jensen_s3_state(s->reg[COMMAND], s->reg[CUR_X], s->reg[CUR_Y],
                         s->reg[DEST_X], s->reg[DEST_Y],
                         (s->reg[MAJOR] & 0xfff) + 1,
                         (s->multi[0] & 0xfff) + 1, s->multi[10],
                         s->reg[FORE_MIX] | (s->reg[BACK_MIX] << 16),
                         s->reg[WRITE_MASK] | (s->vga.cr[0x50] << 16));
    if (op == 0) {
        return;
    }
    if ((op != 2 && op != 6 && op != 7) || (s->vga.cr[0x50] & 0x20) ||
        (s->multi[14] & 0x100) ||
        ((s->reg[COMMAND] & 0x100) &&
         (!(s->reg[COMMAND] & 1) || (s->reg[COMMAND] & 0x600) >= 0x400))) {
        s->unsupported = true;
        qemu_log_mask(LOG_UNIMP,
                      "Jensen S3: unsupported command/depth %04x/%02x\n",
                      s->reg[COMMAND], s->vga.cr[0x50]);
        return;
    }
    s->pending = true;
    if (!(s->reg[COMMAND] & 0x100)) {
        while (s->pending) {
            s3_step(s, 0, true);
        }
    }
}

static void s3_transfer(JensenS3State *s, uint32_t value, unsigned size)
{
    unsigned i, bits = MIN(size * 8,
                           8U << ((s->reg[COMMAND] >> 9) & 3));
    unsigned bytes = s3_pixel_bytes(s);
    bool mono = (s->multi[10] & 0xc0) == 0x80 ||
                (s->reg[COMMAND] & 2);
    unsigned width = (s->reg[MAJOR] & 0xfff) + 1;
    unsigned row = s->progress / width;
    if (!s->pending || !(s->reg[COMMAND] & 0x100)) {
        return;
    }
    if ((s->reg[COMMAND] & 0x1000) && bits >= 16 &&
        (mono || bytes == 1)) {
        value = ((value & 0xff00ff00) >> 8) | ((value & 0x00ff00ff) << 8);
    }
    for (i = 0; i < (mono ? bits : bits / (8 * bytes)) && s->pending; i++) {
        uint16_t pixel = 0;

        if (!mono) {
            /* BYTE_SWAP selects byte order, not the bits of a 16-bpp pixel. */
            unsigned offset = bytes == 1 ? bits - 8 - i * 8 : i * 16;
            pixel = value >> offset;
        }
        s3_step(s, pixel, !mono || !!(value & (1U << (bits - 1 - i))));
        /* Every CPU row is padded to the configured transfer width. */
        if (s->progress / width != row) {
            break;
        }
    }
}

static uint64_t s3_accel_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenS3State *s = opaque;
    unsigned port = addr, i;
    uint32_t value = 0;
    if (!(s->vga.cr[0x40] & 1)) {
        return MAKE_64BIT_MASK(0, size * 8);
    }
    if ((port & ~3) == 0xbee8) {
        value = s->multi[s->multi[15] & 15];
    } else {
        for (i = 0; i < ARRAY_SIZE(s3_ports); i++) {
            if ((port & ~3) == s3_ports[i]) {
                value = i == COMMAND ?
                    ((s->pending || s->unsupported) ? 0x200 : 0) : s->reg[i];
                break;
            }
        }
    }
    trace_jensen_s3_io(port, value, size, false);
    return (value >> ((port & 3) * 8)) & MAKE_64BIT_MASK(0, size * 8);
}

/* Radial short vectors used by the NT driver for focus outlines. */
static void s3_short_stroke(JensenS3State *s, uint16_t word)
{
    static const int8_t dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    static const int8_t dy[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
    unsigned part, i;
    int x = s->reg[CUR_X] & 0xfff;
    int y = s->reg[CUR_Y] & 0xfff;

    if ((s->reg[COMMAND] >> 13) != 0 || !(s->reg[COMMAND] & 8) ||
        (s->reg[COMMAND] & 0x100) || (s->vga.cr[0x50] & 0x20)) {
        s->unsupported = true;
        qemu_log_mask(LOG_UNIMP, "Jensen S3: unsupported short vector %04x\n",
                      s->reg[COMMAND]);
        return;
    }
    for (part = 0; part < 2; part++) {
        unsigned shift = (s->reg[COMMAND] & 0x1000) ? part * 8 : (1 - part) * 8;
        uint8_t vector = word >> shift;
        unsigned dir = vector >> 5;
        for (i = 0; i <= (vector & 15); i++) {
            if (vector & 0x10) {
                s3_pixel(s, x, y, 0, true, 0);
            }
            if (i != (vector & 15)) {
                x = (x + dx[dir]) & 0xfff;
                y = (y + dy[dir]) & 0xfff;
            }
        }
    }
    s->reg[CUR_X] = x;
    s->reg[CUR_Y] = y;
}

static void s3_accel_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    JensenS3State *s = opaque;
    unsigned port = addr, i, shift = (port & 3) * 8;
    uint16_t word;
    trace_jensen_s3_io(port, value, size, true);
    if (!(s->vga.cr[0x40] & 1)) {
        return;
    }
    if ((port & ~3) == 0xe2e8) {
        s3_transfer(s, value, size);
        return;
    }
    if ((port & ~3) == 0xbee8) {
        /* Indexed multifunction control is committed on the high byte. */
        word = (s->multi_latch &
                ~(MAKE_64BIT_MASK(0, size * 8) << shift)) |
               (value << shift);
        s->multi_latch = word;
        if (size >= 2 || shift == 8) {
            s->multi[word >> 12] = word & 0xfff;
        }
        return;
    }
    for (i = 0; i < ARRAY_SIZE(s3_ports); i++) {
        if ((port & ~3) == s3_ports[i]) {
            s->reg[i] = (s->reg[i] &
                         ~(MAKE_64BIT_MASK(0, size * 8) << shift)) |
                        (value << shift);
            if (i == COMMAND && (size >= 2 || shift == 8)) {
                s3_command(s);
            } else if (i == SHORT_STROKE && (size >= 2 || shift == 8)) {
                s3_short_stroke(s, s->reg[i]);
            } else if (i == SUBSYS && s->reg[i] & 0x8000) {
                s->pending = s->unsupported = false;
            }
            return;
        }
    }
    qemu_log_mask(LOG_UNIMP,
                  "Jensen S3: unsupported accelerator port %04x\n", port);
}

/* Each small port region adds its absolute base to the callback address. */
static uint64_t s3_engine_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenS3Port *p = opaque;
    return s3_accel_read(p->state, p->port + addr, size);
}

static void s3_engine_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    JensenS3Port *p = opaque;
    s3_accel_write(p->state, p->port + addr, value, size);
}

static const MemoryRegionOps s3_accel_ops = {
    .read = s3_engine_read,
    .write = s3_engine_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static unsigned s3_dac_index(JensenS3State *s, unsigned port)
{
    return (port & 3) | ((s->vga.cr[0x55] & 3) << 2);
}

static uint8_t s3_dac_read(JensenS3State *s, unsigned port)
{
    unsigned index = s3_dac_index(s, port);
    switch (index) {
    case 6:
        return s->dac_command[0];
    case 8:
        return s->dac_command[1];
    case 9:
        return s->dac_command[2];
    case 10:
        return (s->dac_command[0] & 0x80) && (s->dac_addr & 0x3f) == 1 ?
                   s->dac_command[3] : 0x60; /* Bt485 status */
    case 11:
        return s->cursor_ram[s->dac_addr++ & 1023];
    case 12:
        return s->cursor_x;
    case 13:
        return s->cursor_x >> 8;
    case 14:
        return s->cursor_y;
    case 15:
        return s->cursor_y >> 8;
    default:
        return vga_ioport_read(&s->vga, port);
    }
}

static void s3_dac_write(JensenS3State *s, unsigned port, uint8_t value)
{
    unsigned index = s3_dac_index(s, port);
    switch (index) {
    case 0:
    case 3:
    case 4:
    case 7:
        s->dac_addr = value | ((s->dac_command[3] & 3) << 8);
        s->dac_component = 0;
        if (index < 4) {
            vga_ioport_write(&s->vga, port, value);
        }
        break;
    case 5:
        s->cursor_palette[(s->dac_addr & 3) * 3 + s->dac_component++] = value;
        if (s->dac_component == 3) {
            s->dac_component = 0;
            s->dac_addr++;
        }
        break;
    case 6:
        s->dac_command[0] = value;
        s->vga.dac_8bit = !!(value & 2);
        break;
    case 8:
        s->dac_command[1] = value;
        break;
    case 9:
        s->dac_command[2] = value;
        break;
    case 10:
        if ((s->dac_command[0] & 0x80) && (s->dac_addr & 0x3f) == 1) {
            s->dac_command[3] = value;
        }
        break;
    case 11:
        s->cursor_ram[s->dac_addr++ & 1023] = value;
        break;
    case 12:
        s->cursor_x = (s->cursor_x & 0xf00) | value;
        break;
    case 13:
        s->cursor_x = (s->cursor_x & 0xff) | ((value & 15) << 8);
        break;
    case 14:
        s->cursor_y = (s->cursor_y & 0xf00) | value;
        break;
    case 15:
        s->cursor_y = (s->cursor_y & 0xff) | ((value & 15) << 8);
        break;
    default:
        vga_ioport_write(&s->vga, port, value);
        break;
    }
    s->vga.full_update_gfx = true;
}

static uint64_t s3_vga_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenS3State *s = opaque;
    VGACommonState *vga = &s->vga;
    unsigned port = addr + 0x3b0;
    uint8_t value;
    if (port == 0x3c6 || port == 0x3c7 || port == 0x3c8 || port == 0x3c9) {
        value = s3_dac_read(s, port);
    } else if ((port == 0x3b5 || port == 0x3d5) && vga->cr_index >= 0x30) {
        value = vga->cr[vga->cr_index];
        if (vga->cr_index == 0x30 &&
            (vga->cr[0x38] & 0xcc) != 0x48 &&
            (vga->cr[0x39] & 0xe0) != 0xa0) {
            value = 0xff;
        }
        trace_jensen_s3_cr(vga->cr_index, value, false);
    } else {
        value = vga_ioport_read(vga, port);
    }
    return value;
}

static void s3_vga_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    JensenS3State *s = opaque;
    VGACommonState *vga = &s->vga;
    unsigned port = addr + 0x3b0;
    unsigned index = vga->cr_index;
    if (port >= 0x3c6 && port <= 0x3c9) {
        s3_dac_write(s, port, value);
    } else if (port == 0x3b4 || port == 0x3d4) {
        vga->cr_index = value & 0x7f;
    } else if ((port == 0x3b5 || port == 0x3d5) && index >= 0x20) {
        trace_jensen_s3_cr(index, value, true);
        if (index == 0x30 || index == 0x37 ||
            (index == 0x36 && vga->cr[0x39] != 0xa5) ||
            (index < 0x40 && index != 0x36 && index != 0x38 && index != 0x39 &&
             (vga->cr[0x38] & 0xcc) != 0x48) ||
            (index >= 0x40 && (vga->cr[0x39] & 0xe0) != 0xa0)) {
            return;
        }
        vga->cr[index] = value;
        vga->full_update_gfx = true;
    } else {
        vga_ioport_write(vga, port, value);
    }
}

static const MemoryRegionOps s3_vga_ops = {
    .read = s3_vga_read,
    .write = s3_vga_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static unsigned s3_bank_offset(JensenS3State *s, unsigned bank)
{
    bool chain4 = s->vga.sr[4] & 8;
    unsigned size = s->vga.vram_size >> (chain4 ? 0 : 2);
    return (bank << (chain4 ? 16 : 14)) & (size - 1);
}

static uint64_t s3_memory_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenS3State *s = opaque;
    unsigned bank = (s->vga.cr[0x35] & 15) | ((s->vga.cr[0x51] & 12) << 2);
    if (s3_packed(s) && addr < 0x10000) {
        addr = (bank * 0x10000 + addr) & (s->vga.vram_size - 1);
        return s->vga.vram_ptr[addr];
    }
    s->vga.bank_offset = s3_bank_offset(s, bank);
    return vga_mem_readb(&s->vga, addr);
}

static void s3_memory_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    JensenS3State *s = opaque;
    unsigned bank = (s->vga.cr[0x35] & 15) | ((s->vga.cr[0x51] & 12) << 2);
    if (s3_packed(s) && addr < 0x10000) {
        addr = (bank * 0x10000 + addr) & (s->vga.vram_size - 1);
        s->vga.vram_ptr[addr] = value;
        memory_region_set_dirty(&s->vga.vram, addr, 1);
    } else {
        s->vga.bank_offset = s3_bank_offset(s, bank);
        vga_mem_writeb(&s->vga, addr, value);
    }
}

static const MemoryRegionOps s3_memory_ops = {
    .read = s3_memory_read,
    .write = s3_memory_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static void s3_reset(DeviceState *dev)
{
    JensenS3State *s = JENSEN_S3(dev);
    vga_common_reset(&s->vga);
    memset(s->reg, 0, sizeof(s->reg));
    memset(s->multi, 0, sizeof(s->multi));
    s->multi_latch = 0;
    memset(s->dac_command, 0, sizeof(s->dac_command));
    memset(s->cursor_ram, 0, sizeof(s->cursor_ram));
    memset(s->cursor_palette, 0, sizeof(s->cursor_palette));
    s->dac_addr = s->cursor_x = s->cursor_y = s->dac_component = 0;
    s->progress = 0;
    s->pending = s->unsupported = false;
    s->vga.cr[0x30] = 0x91; /* 86C928D */
    s->vga.cr[0x36] = 0x8f; /* ISA, 2 MiB, 8-bit ROM */
    s->vga.cr[0x37] = 0x02;
    s->reg[WRITE_MASK] = s->reg[READ_MASK] = 0xff;
    s->multi[3] = s->multi[4] = 0xfff;
}

static void s3_realize(DeviceState *dev, Error **errp)
{
    JensenS3State *s = JENSEN_S3(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    VGACommonState *vga = &s->vga;
    unsigned i;
    vga->vram_size_mb = 2;
    if (!vga_common_init(vga, OBJECT(dev), errp)) {
        return;
    }
    /* Keep all banked accesses in s3_memory_ops: the generic VGA chain-4
     * alias has higher priority and does not follow S3 CR35/CR51 banks.
     * Leaving legacy_address_space NULL disables only that optimization.
     */
    s->vga_bpp = vga->get_bpp;
    s->vga_params = vga->get_params;
    s->vga_resolution = vga->get_resolution;
    vga->get_bpp = s3_bpp;
    vga->get_params = s3_params;
    vga->get_resolution = s3_resolution;
    memory_region_init_io(&s->ports, OBJECT(dev), &s3_vga_ops, s,
                          "jensen-s3-vga", 0x30);
    memory_region_add_subregion(isa_address_space_io(isa), 0x3b0, &s->ports);
    memory_region_init_io(&s->aperture, OBJECT(dev), &s3_memory_ops, s,
                          "jensen-s3-memory", 0x20000);
    memory_region_add_subregion_overlap(isa_address_space(isa), 0xa0000,
                                        &s->aperture, 1);
    /* The last two regions decode multifunction control and pixel transfer. */
    for (i = 0; i < ARRAY_SIZE(s->accel); i++) {
        JensenS3Port *p = &s->accel[i];
        p->state = s;
        p->port = i < 16 ? s3_ports[i] : i == 16 ? 0xbee8 : 0xe2e8;
        memory_region_init_io(&p->mr, OBJECT(dev), &s3_accel_ops, p,
                              "jensen-s3-engine", 4);
        memory_region_add_subregion(isa_address_space_io(isa), p->port, &p->mr);
    }
    s->hw_ops = *vga->hw_ops;
    s->hw_ops.gfx_update = s3_update_display;
    vga->con = graphic_console_init(dev, 0, &s->hw_ops, vga);
}

static const VMStateDescription s3_vmstate = {
    .name = TYPE_JENSEN_S3,
    .unmigratable = true,
};

static void s3_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = s3_realize;
    dc->vmsd = &s3_vmstate;
    device_class_set_legacy_reset(dc, s3_reset);
    dc->user_creatable = false;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo s3_info = {
    .name = TYPE_JENSEN_S3,
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(JensenS3State),
    .class_init = s3_class_init,
};

static void s3_register_types(void)
{
    type_register_static(&s3_info);
}

type_init(s3_register_types)
