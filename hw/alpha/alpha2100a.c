/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * AlphaServer 2100A (Lynx) EV4-family firmware bring-up experiment.
 * Hardware address definitions and sparse I/O lane behavior follow Linux
 * arch/alpha/include/asm/core_t2.h, based on Sable I/O Specification 1.3.
 * Original interface attribution: Jay A Estabrook (jestabro@amt.tay1.dec.com).
 * Implemented with QEMU ISA, AddressSpace, CPU and clock interfaces;
 * no Linux implementation is copied. No DEC firmware is distributed here.
 * This is not yet a complete T2/T3 chipset or an operating-system machine.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/isa/isa.h"
#include "hw/intc/i8259.h"
#include "hw/rtc/mc146818rtc.h"
#include "hw/input/i8042.h"
#include "hw/timer/i8254.h"
#include "hw/char/serial-isa.h"
#include "hw/i2c/i2c.h"
#include "hw/nvram/eeprom_at24c.h"
#include "system/system.h"
#include "system/physmem.h"
#include "system/reset.h"
#include "cpu.h"
#include "internals.h"
#include "trace.h"

#define TYPE_ALPHA2100A_MACHINE MACHINE_TYPE_NAME("alpha2100a")
OBJECT_DECLARE_SIMPLE_TYPE(Alpha2100AMachine, ALPHA2100A_MACHINE)
#define LYNX_IO 0x3a0000000ULL
#define LYNX_CSR 0x380000000ULL
#define LYNX_CONF 0x390000000ULL
#define SRM_BASE 0x8000
#define T3_IOCSR_REV_MASK (7ULL << 33)
#define T3_IOCSR_REV (4ULL << 33)

struct Alpha2100AMachine {
    MachineState parent_obj;
    AlphaCPU *cpu;
    MemoryRegion isa_io, isa_mem, sparse_io, csr, pci_conf;
    AddressSpace io_as;
    uint64_t csr_regs[16][128];
    uint8_t *firmware;
    gsize firmware_size;
    QEMUTimer *interval_timer;
    int64_t interval_epoch;
    uint64_t interval_ticks;
    bool interval_pending;
    uint64_t ic_regs[2][16];
    uint64_t irq_levels, irq_in_service;
    qemu_irq *pic_irqs;
};

static uint64_t lynx_irq_pending(Alpha2100AMachine *s)
{
    /* AIR=0x40 selects the controller mask; one means disabled. */
    return s->irq_levels & ~s->ic_regs[0][0x40 / 8] & ~s->irq_in_service;
}

static void lynx_irq_update(Alpha2100AMachine *s)
{
    qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpu), ALPHA_CPU_INPUT_IRQ1),
                 lynx_irq_pending(s) != 0);
}

static void lynx_irq_set(void *opaque, int bit, int level)
{
    Alpha2100AMachine *s = opaque;

    if (level) {
        s->irq_levels |= BIT_ULL(bit);
    } else {
        s->irq_levels &= ~BIT_ULL(bit);
    }
    lynx_irq_update(s);
}

static void lynx_isa_irq(void *opaque, int irq, int level)
{
    /* Built-in Lynx devices use IC positions, rather than PIC vectors. */
    static const int bits[16] = {
        -1, 6, -1, 8, 15, 12, 7, 9, -1, 16, 17, 18, 3, -1, 21, 22,
    };

    Alpha2100AMachine *s = opaque;

    qemu_set_irq(s->pic_irqs[irq], level);
    if (bits[irq] >= 0) {
        lynx_irq_set(opaque, bits[irq], level);
    }
}

static void lynx_interval_tick(void *opaque)
{
    Alpha2100AMachine *s = opaque;

    /* CPU specification 4.12/7.2.1.6: CPU0 interval interrupt is 1024 Hz. */
    s->interval_pending = true;
    qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpu), ALPHA_CPU_INPUT_IRQ4), 1);
    s->interval_ticks++;
    timer_mod(s->interval_timer, s->interval_epoch +
              s->interval_ticks * NANOSECONDS_PER_SECOND / 1024);
}

static uint64_t lynx_io_read(void *opaque, hwaddr addr, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    unsigned port = addr >> 5, len = ((addr >> 3) & 3) + 1;
    unsigned shift = len == 4 ? 0 : (port & 3) * 8;
    uint64_t value = 0xffffffff;
    if (len == 1) {
        value = address_space_ldub(&s->io_as, port, MEMTXATTRS_UNSPECIFIED, NULL);
    } else if (len == 2) {
        value = address_space_lduw_le(&s->io_as, port, MEMTXATTRS_UNSPECIFIED, NULL);
    } else if (len == 4) {
        value = address_space_ldl_le(&s->io_as, port, MEMTXATTRS_UNSPECIFIED, NULL);
    }
    trace_alpha2100a_io(false, port, len, value);
    return value << shift;
}

static void lynx_io_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    unsigned port = addr >> 5, len = ((addr >> 3) & 3) + 1;
    value >>= len == 4 ? 0 : (port & 3) * 8;
    trace_alpha2100a_io(true, port, len, value);
    if (len == 1) {
        address_space_stb(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    } else if (len == 2) {
        address_space_stw_le(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    } else if (len == 4) {
        address_space_stl_le(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    }
}

static uint64_t lynx_csr_read(void *opaque, hwaddr addr, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    unsigned block = (addr >> 24) & 15, reg = (addr & 0xffffff) >> 5;
    uint64_t value = reg < 128 ? s->csr_regs[block][reg] : UINT64_MAX;

    if (block >= 14 && reg == 0) {
        /* IOCSR<35:33> is the read-only T3 revision, not writable config. */
        value = (value & ~T3_IOCSR_REV_MASK) | T3_IOCSR_REV;
    } else if (block >= 14 && reg == 0x480 / 32) {
        uint64_t pending = block == 14 ? lynx_irq_pending(s) : 0;

        value = pending ? ctz64(pending) : 0x80;
        if (pending) {
            s->irq_in_service |= BIT_ULL(value);
            lynx_irq_update(s);
        }
    } else if (block >= 14 && reg == 0x4a0 / 32) {
        unsigned air = s->csr_regs[block][0x460 / 32];

        value = air < 0x80 ? s->ic_regs[block - 14][air / 8] : UINT64_MAX;
    } else if (block == 0 && reg == 12) {
        value = (value & ~BIT_ULL(32)) |
                (s->interval_pending ? BIT_ULL(32) : 0);
    }
    if (size == 4) {
        value = (value >> ((addr & 4) * 8)) & UINT32_MAX;
    }
    trace_alpha2100a_csr(false, addr, value);
    return value;
}

static void lynx_csr_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    trace_alpha2100a_csr(true, addr, value);
    unsigned block = (addr >> 24) & 15, reg = (addr & 0xffffff) >> 5;
    uint64_t written = size == 4 ? value << ((addr & 4) * 8) : value;

    /* CMIC SIC bit 32 is a latched interrupt with write-one-to-clear. */
    if (block == 0 && reg == 12 && (written & BIT_ULL(32))) {
        s->interval_pending = false;
        qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpu), ALPHA_CPU_INPUT_IRQ4), 0);
    }
    if (block >= 14 && reg == 0x480 / 32) {
        if (block == 14 && value < 64) {
            s->irq_in_service &= ~BIT_ULL(value);
            lynx_irq_update(s);
        }
        return;
    }
    if (reg < 128) {
        uint64_t *storage = &s->csr_regs[block][reg];

        if (block >= 14 && reg == 0x4a0 / 32) {
            unsigned air = s->csr_regs[block][0x460 / 32];

            if (air >= 0x80) {
                return;
            }
            storage = &s->ic_regs[block - 14][air / 8];
        }
        if (size == 4) {
            uint64_t mask = (uint64_t)UINT32_MAX << ((addr & 4) * 8);
            *storage = (*storage & ~mask) | (written & mask);
        } else {
            *storage = value;
        }
        if (block >= 14 && reg == 0) {
            *storage = (*storage & ~T3_IOCSR_REV_MASK) | T3_IOCSR_REV;
        }
        if (block >= 14 && reg == 0x4a0 / 32) {
            lynx_irq_update(s);
        }
    }
}

static uint64_t lynx_conf_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0xffffffff;
}

static void lynx_conf_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
}

static const MemoryRegionOps lynx_io_ops = {
    .read = lynx_io_read, .write = lynx_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4, .valid.max_access_size = 8,
    .impl.min_access_size = 4, .impl.max_access_size = 8,
};
static const MemoryRegionOps lynx_csr_ops = {
    .read = lynx_csr_read, .write = lynx_csr_write,
    .impl.min_access_size = 4, .impl.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4, .valid.max_access_size = 8,
};
static const MemoryRegionOps lynx_conf_ops = {
    .read = lynx_conf_read, .write = lynx_conf_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4, .valid.max_access_size = 8,
};

static void lynx_reset(MachineState *machine, ResetType type)
{
    Alpha2100AMachine *s = ALPHA2100A_MACHINE(machine);
    uint8_t scratch[SRM_BASE] = { 0 };

    qemu_devices_reset(type);
    /* Replace the skipped SROM's initial low-RAM state on every reset. */
    physical_memory_write(0, scratch, sizeof(scratch));
    physical_memory_write(SRM_BASE, s->firmware, s->firmware_size);
    memset(s->csr_regs, 0, sizeof(s->csr_regs));
    memset(s->ic_regs, 0, sizeof(s->ic_regs));
    s->ic_regs[0][0x40 / 8] = s->ic_regs[1][0x40 / 8] = UINT64_MAX;
    s->irq_levels = s->irq_in_service = 0;
    /* CMIC CBCTL slot identifier uses a one-based code in bits 7:5. */
    s->csr_regs[0][6] = 1 << 5;
    cpu_reset(CPU(s->cpu));
    s->cpu->env.ipr.pal_base = SRM_BASE;
    cpu_set_pc(CPU(s->cpu), SRM_BASE | R_PC_PAL_MODE_MASK);
    s->interval_pending = false;
    s->interval_epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->interval_ticks = 1;
    timer_mod(s->interval_timer, s->interval_epoch +
              NANOSECONDS_PER_SECOND / 1024);
}

static void lynx_init(MachineState *machine)
{
    Alpha2100AMachine *s = ALPHA2100A_MACHINE(machine);
    ISABus *isa;
    qemu_irq *irqs;
    ISADevice *rtc, *pcf;
    static const uint8_t erased_eeprom[256] = { [0 ... 255] = 0xff };
    Clock *refclk;
    g_autofree char *firmware = NULL;
    g_autoptr(GError) gerr = NULL;
    MemoryRegion *sysmem = get_system_memory();
    if (machine->ram_size < 32 * MiB || machine->ram_size > GiB) {
        error_report("AlphaServer 2100A experiment requires 32 MiB to 1 GiB RAM");
        exit(1);
    }
    s->cpu = ALPHA_CPU(object_new(machine->cpu_type));
    refclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(refclk, 275000000);
    qdev_connect_clock_in(DEVICE(s->cpu), "cpu-refclk", refclk);
    qdev_realize_and_unref(DEVICE(s->cpu), NULL, &error_fatal);
    s->interval_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lynx_interval_tick, s);
    memory_region_add_subregion(sysmem, 0, machine->ram);
    memory_region_init(&s->isa_io, OBJECT(machine), "lynx.isa-io", 0x10000);
    memory_region_init(&s->isa_mem, OBJECT(machine), "lynx.isa-mem", 0x100000000ULL);
    address_space_init(&s->io_as, &s->isa_io, "lynx.isa-io");
    memory_region_init_io(&s->sparse_io, OBJECT(machine), &lynx_io_ops, s,
                          "lynx.sparse-io", 0x200000);
    memory_region_add_subregion(sysmem, LYNX_IO, &s->sparse_io);
    memory_region_init_io(&s->csr, OBJECT(machine), &lynx_csr_ops, s,
                          "lynx.csr", 0x10000000);
    memory_region_add_subregion(sysmem, LYNX_CSR, &s->csr);
    memory_region_init_io(&s->pci_conf, OBJECT(machine), &lynx_conf_ops, s,
                          "lynx.pci-config", 0x10000000);
    memory_region_add_subregion(sysmem, LYNX_CONF, &s->pci_conf);
    isa = isa_bus_new(NULL, &s->isa_mem, &s->isa_io, &error_fatal);
    s->pic_irqs = i8259_init(isa, NULL);
    irqs = qemu_allocate_irqs(lynx_isa_irq, s, 16);
    isa_bus_register_input_irqs(isa, irqs);
    isa_create_simple(isa, "i8042");
    isa_create_simple(isa, "isa-pit");
    rtc = isa_new(TYPE_MC146818_RTC);
    qdev_prop_set_int32(DEVICE(rtc), "base_year", 1900);
    isa_realize_and_unref(rtc, isa, &error_fatal);
    serial_hds_isa_init(isa, 0, 2);
    pcf = isa_create_simple(isa, "alpha2100a-pcf8584");
    qdev_connect_gpio_out(DEVICE(pcf), 0, qemu_allocate_irq(lynx_irq_set, s, 23));
    /* Blank I/O-board EEPROM; firmware must handle its invalid checksum. */
    at24c_eeprom_init_rom(I2C_BUS(qdev_get_child_bus(DEVICE(pcf), "lynx-i2c")),
                         0x56, 256, erased_eeprom, sizeof(erased_eeprom));
    firmware = machine->firmware ?
        qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware) : NULL;
    if (!firmware || !g_file_get_contents(firmware, (char **)&s->firmware,
                                         &s->firmware_size, &gerr)) {
        error_report("AlphaServer 2100A: supply extracted EV4 SRM with -bios%s%s",
                     gerr ? ": " : "", gerr ? gerr->message : "");
        exit(1);
    }
    if (!s->firmware_size || s->firmware_size > 4 * MiB ||
        s->firmware_size < 8 || ldl_le_p(s->firmware) != 0x207f0000) {
        error_report("AlphaServer 2100A requires an extracted EV4 SRM image");
        exit(1);
    }
}

static void lynx_finalize(Object *obj)
{
    Alpha2100AMachine *s = ALPHA2100A_MACHINE(obj);
    g_free(s->firmware);
    timer_free(s->interval_timer);
    if (s->io_as.root) {
        address_space_destroy(&s->io_as);
    }
}

static void lynx_class_init(ObjectClass *oc, const void *data)
{
    static const char *const cpu_types[] = { ALPHA_CPU_TYPE_NAME("ev4"),
        ALPHA_CPU_TYPE_NAME("ev45"), NULL };
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "DEC AlphaServer 2100A (Lynx, experimental firmware bring-up)";
    mc->init = lynx_init;
    mc->reset = lynx_reset;
    mc->default_cpu_type = ALPHA_CPU_TYPE_NAME("ev45");
    mc->valid_cpu_types = cpu_types;
    mc->default_ram_id = "alpha2100a.ram";
    mc->default_ram_size = 64 * MiB;
    mc->max_cpus = 1;
    mc->no_floppy = true;
    mc->no_parallel = true;
}
static const TypeInfo lynx_type = {
    .name = TYPE_ALPHA2100A_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Alpha2100AMachine),
    .class_init = lynx_class_init,
    .instance_finalize = lynx_finalize,
};
static void lynx_register(void)
{
    type_register_static(&lynx_type);
}
type_init(lynx_register)
