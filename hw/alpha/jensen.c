/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DECpc AXP 150 / DEC 2000 model 300 (Jensen, PB22H-KB).
 *
 * Address decoding and SROM handoff follow DEC EK-A0638-TD.001,
 * chapters 5, 6, 8 and 10. The serial boot ROM is not emulated;
 * execution starts at the system Flash bootstrap in PAL mode.
 */

#include "qemu/osdep.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/block/flash.h"
#include "hw/char/serial-mm.h"
#include "hw/input/i8042.h"
#include "hw/intc/i8259.h"
#include "hw/isa/isa.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/rtc/mc146818rtc.h"
#include "hw/timer/i8254.h"
#include "system/reset.h"
#include "system/system.h"
#include "cpu.h"
#include "internals.h"

#define TYPE_JENSEN_MACHINE MACHINE_TYPE_NAME("jensen")
OBJECT_DECLARE_SIMPLE_TYPE(JensenMachineState, JENSEN_MACHINE)

#define JENSEN_FLASH_SIZE MiB
#define JENSEN_BOOT_SIZE (512 * KiB)
#define JENSEN_BOOT_BASE 0x4000
#define JENSEN_BOOT_ENTRY (JENSEN_BOOT_BASE + 0x40)
#define JENSEN_INTA_BASE 0x100000000ULL
#define JENSEN_FLASH_BASE 0x180000000ULL
#define JENSEN_FLASH1_BASE 0x1a0000000ULL
#define JENSEN_LOCAL_BASE 0x1c0000000ULL
#define JENSEN_HAE_BASE 0x1d0000000ULL
#define JENSEN_SYSCTL_BASE 0x1e0000000ULL
#define JENSEN_SPARE_BASE 0x1f0000000ULL
#define JENSEN_EISA_MEM_BASE 0x200000000ULL
#define JENSEN_EISA_IO_BASE 0x300000000ULL

struct JensenMachineState {
    MachineState parent_obj;
    AlphaCPU *cpu;
    MemoryRegion isa_io, isa_mem;
    MemoryRegion local, eisa_io, eisa_mem, flash_window[2];
    MemoryRegion inta, hae_reg, sysctl_reg, spare_reg;
    AddressSpace io_as, mem_as, flash_as[2];
    PFlashCFI01 *flash[2];
    uint8_t hae, sysctl, spare, local_irqs;
};

static uint64_t jensen_local_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenMachineState *s = opaque;
    return address_space_ldub(&s->io_as, addr >> 9,
                              MEMTXATTRS_UNSPECIFIED, NULL);
}

static void jensen_local_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    JensenMachineState *s = opaque;
    address_space_stb(&s->io_as, addr >> 9, value,
                      MEMTXATTRS_UNSPECIFIED, NULL);
}

static const MemoryRegionOps jensen_local_ops = {
    .read = jensen_local_read,
    .write = jensen_local_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

static uint64_t jensen_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    AddressSpace *as = opaque;
    return address_space_ldub(as, addr >> 9,
                              MEMTXATTRS_UNSPECIFIED, NULL);
}

static void jensen_flash_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    AddressSpace *as = opaque;
    address_space_stb(as, addr >> 9, value,
                      MEMTXATTRS_UNSPECIFIED, NULL);
}

static const MemoryRegionOps jensen_flash_ops = {
    .read = jensen_flash_read,
    .write = jensen_flash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

/* CA<8:5> encodes byte enables; the value occupies its EISA byte lane. */
static uint64_t jensen_sparse_read(JensenMachineState *s, AddressSpace *as,
                                   hwaddr addr)
{
    hwaddr busaddr = (addr >> 7) | ((hwaddr)s->hae << 25);
    unsigned len = ((addr >> 5) & 3) + 1;
    uint8_t data[4] = { 0xff, 0xff, 0xff, 0xff };
    uint64_t value = 0;
    unsigned i;

    address_space_read(as, busaddr, MEMTXATTRS_UNSPECIFIED, data, len);
    for (i = 0; i < len; i++) {
        value |= (uint64_t)data[i] << (i * 8);
    }
    return value << ((busaddr & 3) * 8);
}

static void jensen_sparse_write(JensenMachineState *s, AddressSpace *as,
                                hwaddr addr, uint64_t value)
{
    hwaddr busaddr = (addr >> 7) | ((hwaddr)s->hae << 25);
    unsigned len = ((addr >> 5) & 3) + 1;
    uint8_t data[4];
    unsigned i;

    value >>= (busaddr & 3) * 8;
    for (i = 0; i < len; i++) {
        data[i] = value >> (i * 8);
    }
    address_space_write(as, busaddr, MEMTXATTRS_UNSPECIFIED, data, len);
}

static uint64_t jensen_eisa_io_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenMachineState *s = opaque;
    return jensen_sparse_read(s, &s->io_as, addr);
}

static void jensen_eisa_io_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned size)
{
    JensenMachineState *s = opaque;
    jensen_sparse_write(s, &s->io_as, addr, value);
}

static const MemoryRegionOps jensen_eisa_io_ops = {
    .read = jensen_eisa_io_read,
    .write = jensen_eisa_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 8,
};

static uint64_t jensen_eisa_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    JensenMachineState *s = opaque;
    return jensen_sparse_read(s, &s->mem_as, addr);
}

static void jensen_eisa_mem_write(void *opaque, hwaddr addr,
                                 uint64_t value, unsigned size)
{
    JensenMachineState *s = opaque;
    jensen_sparse_write(s, &s->mem_as, addr, value);
}

static const MemoryRegionOps jensen_eisa_mem_ops = {
    .read = jensen_eisa_mem_read,
    .write = jensen_eisa_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 8,
};

static uint64_t jensen_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    return *(uint8_t *)opaque;
}

static void jensen_reg_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    *(uint8_t *)opaque = value;
}

static const MemoryRegionOps jensen_reg_ops = {
    .read = jensen_reg_read,
    .write = jensen_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

static uint64_t jensen_inta_read(void *opaque, hwaddr addr, unsigned size)
{
    return pic_read_irq(isa_pic);
}

static void jensen_inta_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    /* INTA only responds to reads; writes have no effect. */
}

static const MemoryRegionOps jensen_inta_ops = {
    .read = jensen_inta_read,
    .write = jensen_inta_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

static void jensen_local_irq(void *opaque, int n, int level)
{
    JensenMachineState *s = opaque;
    s->local_irqs = deposit32(s->local_irqs, n, 1, level);
    qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpu), ALPHA_CPU_INPUT_IRQ5),
                 (s->local_irqs & 3) != 0);
    qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpu), ALPHA_CPU_INPUT_IRQ3),
                 (s->local_irqs & 12) != 0);
}

static void jensen_reset(void *opaque)
{
    JensenMachineState *s = opaque;
    MachineState *ms = MACHINE(s);
    CPUAlphaState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    s->hae = 0;
    s->spare = 0;
    s->local_irqs = 0;
    s->sysctl = ms->ram_size == 128 * MiB ? 0xa0 : 0x20;
    env->ipr.pal_base = JENSEN_BOOT_BASE;
    alpha_cpu_set_reg_value(env, 20, ms->ram_size / MiB);
    /* V2.2 SROM handoff uses R24 = 6600 for the 150 MHz CPU timing. */
    alpha_cpu_set_reg_value(env, 24, 6600);
    cpu_set_pc(CPU(s->cpu), JENSEN_BOOT_ENTRY | R_PC_PAL_MODE_MASK);
}

static void jensen_map(JensenMachineState *s, MemoryRegion *mr,
                       const MemoryRegionOps *ops, void *opaque,
                       const char *name, hwaddr base, uint64_t size)
{
    memory_region_init_io(mr, OBJECT(s), ops, opaque, name, size);
    memory_region_add_subregion(get_system_memory(), base, mr);
}

static void jensen_init(MachineState *machine)
{
    JensenMachineState *s = JENSEN_MACHINE(machine);
    Clock *refclk;
    ISABus *bus;
    ISADevice *rtc, *kbd;
    unsigned i;
    qemu_irq *pic;
    const char *bios = machine->firmware ?: "jensen.bin";
    g_autofree char *path = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios);
    g_autofree char *image = NULL;
    gsize image_len;

    if (machine->ram_size != 64 * MiB && machine->ram_size != 128 * MiB) {
        error_report("Jensen currently supports 64M or 128M of RAM");
        exit(EXIT_FAILURE);
    }
    if (!path || !g_file_get_contents(path, &image, &image_len, NULL) ||
        image_len != JENSEN_FLASH_SIZE) {
        error_report("Jensen requires a raw 1 MiB system ROM (-bios); "
                     "extract it from the update using scripts/jensen-extract.py");
        exit(EXIT_FAILURE);
    }

    refclk = clock_new(OBJECT(machine), "system-refclk");
    clock_set_hz(refclk, 150000000);
    s->cpu = ALPHA_CPU(object_new(machine->cpu_type));
    qdev_connect_clock_in(DEVICE(s->cpu), "cpu-refclk", refclk);
    qdev_realize_and_unref(DEVICE(s->cpu), NULL, &error_fatal);
    qemu_register_reset(jensen_reset, s);
    memory_region_add_subregion(get_system_memory(), 0, machine->ram);

    memory_region_init(&s->isa_io, OBJECT(s), "jensen-isa-io", 0x10000);
    memory_region_init(&s->isa_mem, OBJECT(s), "jensen-isa-memory", 4 * GiB);
    address_space_init(&s->io_as, &s->isa_io, "jensen-io");
    address_space_init(&s->mem_as, &s->isa_mem, "jensen-memory");
    bus = isa_bus_new(NULL, &s->isa_mem, &s->isa_io, &error_fatal);
    pic = i8259_init(bus, qdev_get_gpio_in(DEVICE(s->cpu),
                                         ALPHA_CPU_INPUT_IRQ1));
    isa_bus_register_input_irqs(bus, pic);
    i8254_pit_init(bus, 0x40, 0, NULL);

    rtc = isa_new(TYPE_MC146818_RTC);
    qdev_prop_set_uint16(DEVICE(rtc), "iobase", 0x170);
    qdev_prop_set_int32(DEVICE(rtc), "base_year", 1980);
    isa_realize_and_unref(rtc, bus, &error_fatal);
    qdev_connect_gpio_out(DEVICE(rtc), 0,
                          qdev_get_gpio_in(DEVICE(s->cpu),
                                           ALPHA_CPU_INPUT_IRQ0));
    kbd = isa_create_simple(bus, TYPE_I8042);
    qdev_connect_gpio_out(DEVICE(kbd), 0,
                          qemu_allocate_irq(jensen_local_irq, s, 2));
    qdev_connect_gpio_out(DEVICE(kbd), 1,
                          qemu_allocate_irq(jensen_local_irq, s, 3));
    serial_mm_init(&s->isa_io, 0x3f8, 0,
                   qemu_allocate_irq(jensen_local_irq, s, 0), 1843200 / 16,
                   serial_hd(0), DEVICE_LITTLE_ENDIAN);
    serial_mm_init(&s->isa_io, 0x2f8, 0,
                   qemu_allocate_irq(jensen_local_irq, s, 1), 1843200 / 16,
                   serial_hd(1), DEVICE_LITTLE_ENDIAN);
    if (machine->enable_graphics) {
        isa_vga_init(bus);
    }

    for (i = 0; i < 2; i++) {
        DeviceState *flash = qdev_new(TYPE_PFLASH_CFI01);
        MemoryRegion *mr;

        qdev_prop_set_string(flash, "name",
                             i ? "jensen.nvram" : "jensen.flash");
        qdev_prop_set_uint32(flash, "num-blocks", 16);
        qdev_prop_set_uint64(flash, "sector-length", 64 * KiB);
        qdev_prop_set_uint8(flash, "width", 1);
        qdev_prop_set_uint8(flash, "device-width", 1);
        qdev_prop_set_uint16(flash, "id0", 0x89);
        qdev_prop_set_uint16(flash, "id1", 0xa2);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(flash), &error_fatal);
        s->flash[i] = PFLASH_CFI01(flash);
        mr = pflash_cfi01_get_memory(s->flash[i]);
        if (i == 0) {
            memcpy(memory_region_get_ram_ptr(mr), image, image_len);
        } else {
            memset(memory_region_get_ram_ptr(mr), 0xff, JENSEN_FLASH_SIZE);
        }
        address_space_init(&s->flash_as[i], mr,
                           i ? "jensen-nvram" : "jensen-flash");
    }
    /*
     * V2.2 extends the bootstrap image beyond the original 256 KiB ROM.
     * Load the first 512 KiB bank; copying the whole Flash would
     * overwrite the SRM BSS area above the decompressed image.
     */
    rom_add_blob_fixed("jensen.srom-handoff", image, JENSEN_BOOT_SIZE,
                       JENSEN_BOOT_BASE);

    jensen_map(s, &s->flash_window[0], &jensen_flash_ops,
                &s->flash_as[0], "jensen-feprom0",
                JENSEN_FLASH_BASE, 512 * MiB);
    jensen_map(s, &s->flash_window[1], &jensen_flash_ops,
                &s->flash_as[1], "jensen-feprom1",
                JENSEN_FLASH1_BASE, 512 * MiB);
    jensen_map(s, &s->local, &jensen_local_ops, s, "jensen-local",
                JENSEN_LOCAL_BASE, 256 * MiB);
    jensen_map(s, &s->eisa_io, &jensen_eisa_io_ops, s, "jensen-eisa-io",
                JENSEN_EISA_IO_BASE, 4 * GiB);
    jensen_map(s, &s->eisa_mem, &jensen_eisa_mem_ops, s, "jensen-eisa-memory",
                JENSEN_EISA_MEM_BASE, 4 * GiB);
    jensen_map(s, &s->inta, &jensen_inta_ops, s, "jensen-inta",
                JENSEN_INTA_BASE, 256 * MiB);
    jensen_map(s, &s->hae_reg, &jensen_reg_ops, &s->hae, "jensen-hae",
                JENSEN_HAE_BASE, 256 * MiB);
    jensen_map(s, &s->sysctl_reg, &jensen_reg_ops, &s->sysctl, "jensen-sysctl",
                JENSEN_SYSCTL_BASE, 256 * MiB);
    jensen_map(s, &s->spare_reg, &jensen_reg_ops, &s->spare, "jensen-spare",
                JENSEN_SPARE_BASE, 256 * MiB);
}

static void jensen_class_init(ObjectClass *oc, void *data)
{
    static const char * const cpu_types[] = {
        ALPHA_CPU_TYPE_NAME("ev4"), NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "DECpc AXP 150 / DEC 2000 (Jensen, experimental)";
    mc->init = jensen_init;
    mc->default_cpu_type = ALPHA_CPU_TYPE_NAME("ev4");
    mc->valid_cpu_types = cpu_types;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "jensen.ram";
    mc->max_cpus = 1;
    mc->no_floppy = true;
    mc->no_parallel = true;
}

static const TypeInfo jensen_machine_info = {
    .name = TYPE_JENSEN_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(JensenMachineState),
    .class_init = jensen_class_init,
};

static void jensen_register_types(void)
{
    type_register_static(&jensen_machine_info);
}

type_init(jensen_register_types)
