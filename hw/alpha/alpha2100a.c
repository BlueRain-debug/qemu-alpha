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
#include "hw/display/vga.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "hw/core/sysbus.h"
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
#define CBCTL_ID_MASK ((7ULL << 37) | (7ULL << 5))

typedef struct LynxInterval {
    Alpha2100AMachine *machine;
    QEMUTimer *timer;
    uint64_t ticks;
    unsigned id;
    bool pending;
} LynxInterval;

struct Alpha2100AMachine {
    MachineState parent_obj;
    AlphaCPU *cpus[4];
    MemoryRegion isa_io, isa_mem, sparse_io, sparse_mem, dense_mem;
    MemoryRegion csr, pci_conf, vga_rom, mem_diag, mem_test_store;
    uint16_t *mem_edc;
    uint64_t mem_size_strap;
    bool mem_uncorrectable;
    PCIBus *pci_bus;
    bool s3_928;
    char *s3_romfile;
    AddressSpace io_as, mem_as;
    uint64_t csr_regs[16][128];
    uint8_t nvram[8192], nvram_page;
    uint8_t *firmware;
    gsize firmware_size;
    LynxInterval intervals[4];
    int64_t interval_epoch;
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
    /* CIRQ is wired to all CPU modules; PAL HIER selects the recipient. */
    for (unsigned i = 0; i < MACHINE(s)->smp.cpus; i++) {
        qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[i]), ALPHA_CPU_INPUT_IRQ1),
                     lynx_irq_pending(s) != 0);
    }
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
    LynxInterval *t = opaque;
    Alpha2100AMachine *s = t->machine;

    /* CPU specification 4.12: stagger the four 1024 Hz local clocks. */
    t->pending = true;
    qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[t->id]),
                                 ALPHA_CPU_INPUT_IRQ4), 1);
    t->ticks++;
    timer_mod(t->timer, s->interval_epoch +
              (t->ticks * 4 + t->id) * NANOSECONDS_PER_SECOND / 4096);
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
    if (port == 0xc00) {
        value = s->nvram_page;
    } else if (port >= 0x800 && port + len <= 0x900) {
        value = 0;
        for (unsigned i = 0; i < len; i++) {
            value |= (uint32_t)s->nvram[s->nvram_page * 256 + port - 0x800 + i]
                     << (i * 8);
        }
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
    if (port == 0xc00) {
        s->nvram_page = value & 31;
        return;
    } else if (port >= 0x800 && port + len <= 0x900) {
        for (unsigned i = 0; i < len; i++) {
            s->nvram[s->nvram_page * 256 + port - 0x800 + i] = value >> (i * 8);
        }
        return;
    }
    if (len == 1) {
        address_space_stb(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    } else if (len == 2) {
        address_space_stw_le(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    } else if (len == 4) {
        address_space_stl_le(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    }
}

static uint64_t lynx_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    unsigned len = ((addr >> 3) & 3) + 1;
    hwaddr busaddr = (addr >> 5) | (s->csr_regs[14][7] & 0xf8000000);
    unsigned shift = len == 4 ? 0 : (busaddr & 3) * 8;
    uint8_t data[4] = { 0xff, 0xff, 0xff, 0xff };

    address_space_read(&s->mem_as, busaddr, MEMTXATTRS_UNSPECIFIED, data, len);
    return (uint64_t)(ldl_le_p(data) & (UINT32_MAX >> ((4 - len) * 8))) << shift;
}

static void lynx_mem_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    Alpha2100AMachine *s = opaque;
    unsigned len = ((addr >> 3) & 3) + 1;
    hwaddr busaddr = (addr >> 5) | (s->csr_regs[14][7] & 0xf8000000);
    unsigned shift = len == 4 ? 0 : (busaddr & 3) * 8;
    uint8_t data[4];

    stl_le_p(data, value >> shift);
    address_space_write(&s->mem_as, busaddr, MEMTXATTRS_UNSPECIFIED, data, len);
}

/* Limited CSIC diagnostic data path. This is not a complete DRAM/ECC model.
 * Native diagnostic transactions establish the zero/one code and three data
 * columns below. Other injected syndromes remain uncorrectable; the full
 * 128-bit generator matrix, timing and cache writeback still need a spec.
 */
static uint16_t lynx_mem_code(const uint8_t *line, unsigned slice)
{
    bool ones = true;
    uint16_t code = 0xc14;

    for (unsigned i = 0; i < 4; i++) {
        ones &= ldl_le_p(line + (i * 2 + slice) * 4) == UINT32_MAX;
    }
    if (ones) {
        return 0x3eb;
    }
    uint32_t lo = ldl_le_p(line + slice * 4);
    uint32_t hi = ldl_le_p(line + (2 + slice) * 4);
    code ^= (lo & 1) ? 0x37 : 0;
    code ^= (lo & 2) ? 0x19 : 0;
    code ^= (hi & 1) ? 0x8b0 : 0;
    return code;
}

static void lynx_mem_capture(Alpha2100AMachine *s, hwaddr line_addr,
                              uint8_t *line, bool read)
{
    uint64_t control = s->csr_regs[8][6];
    uint64_t latch = 0;
    uint64_t flags = 0, syndrome_latch = 0;
    uint16_t *codes = s->mem_edc + (line_addr / 32) * 2;

    for (unsigned slice = 0; slice < 2; slice++) {
        uint32_t ctl = control >> (slice * 32);
        uint16_t generated = lynx_mem_code(line, slice);
        uint16_t code;
        uint16_t syndrome;
        uint32_t status = 0, correction = 0;
        unsigned word = slice;

        if (!read) {
            codes[slice] = (ctl & BIT(13)) ? (ctl >> 16) & 0xfff : generated;
        }
        code = codes[slice];
        latch |= (uint64_t)(code * 0x10001) << (slice * 32);
        syndrome = ((ctl & BIT(12)) ? ctl & 0xfff : code) ^ generated;
        if (read && syndrome) {
            switch (syndrome) {
            case 0x37: correction = 1; status = 0x17; break;
            case 0x19: correction = 2; status = 0x29; break;
            case 0x2e: correction = 3; status = 0x3e; break;
            case 0x8b0: correction = 1; word += 2; status = 0x8b0; break;
            default: break;
            }
            if (correction) {
                stl_le_p(line + word * 4, ldl_le_p(line + word * 4) ^ correction);
            }
            if (!(ctl & BIT(31))) {
                s->mem_uncorrectable |= !correction;
                flags |= (uint64_t)(correction ? 0x40001 : 0x10001)
                         << (slice * 32);
                syndrome_latch |= (uint64_t)status << (slice * 32);
            }
        }
    }
    if (!(control & BIT_ULL(31)) || !(control & BIT_ULL(63))) {
        s->csr_regs[8][4] = latch;
    }
    if (flags && !(s->csr_regs[8][0] & 0x100000001ULL)) {
        s->csr_regs[8][0] |= flags;
        s->csr_regs[8][5] = syndrome_latch;
        unsigned commander = current_cpu ? (current_cpu->cpu_index + 1) & 3 : 1;
        uint64_t physical = 0x20000000 + line_addr;
        uint32_t command = 0xe2000000 | (physical >> 26);
        uint32_t row = commander << 21;
        s->csr_regs[8][1] =
            (uint64_t)((flags & BIT_ULL(32)) ? physical >> 2 : command) << 32 |
            ((flags & 1) ? physical >> 2 : command);
        s->csr_regs[8][2] = (uint64_t)row << 32 | row;
        if (!(flags & 0x4000000040000ULL)) {
            return; /* uncorrectable-only reads terminate with CACK_HERR */
        }
        for (unsigned i = 0; i < MACHINE(s)->smp.cpus; i++) {
            s->csr_regs[i][12] |= 4;
            qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[i]),
                                         ALPHA_CPU_INPUT_IRQ0), 1);
        }
    }
}

static MemTxResult lynx_diag_read(void *opaque, hwaddr addr, uint64_t *value,
                                  unsigned size, MemTxAttrs attrs)
{
    Alpha2100AMachine *s = opaque;
    uint8_t *ram = memory_region_get_ram_ptr(&s->mem_test_store);
    uint8_t line[32];

    memcpy(line, ram + (addr & ~31ULL), sizeof(line));
    s->mem_uncorrectable = false;
    lynx_mem_capture(s, addr & ~31ULL, line, true);
    *value = size == 4 ? ldl_le_p(line + (addr & 31)) :
                        ldq_le_p(line + (addr & 31));
    return s->mem_uncorrectable ? MEMTX_ERROR : MEMTX_OK;
}

static void lynx_diag_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    Alpha2100AMachine *s = opaque;
    uint8_t *ram = memory_region_get_ram_ptr(&s->mem_test_store);
    if (size == 4) {
        stl_le_p(ram + addr, value);
    } else {
        stq_le_p(ram + addr, value);
    }
    lynx_mem_capture(s, addr & ~31ULL, ram + (addr & ~31ULL), false);
    memory_region_set_dirty(&s->mem_test_store, addr, size);
}

static const MemoryRegionOps lynx_diag_ops = {
    .read_with_attrs = lynx_diag_read, .write = lynx_diag_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4, .valid.max_access_size = 8,
};

static uint64_t lynx_csr_read(void *opaque, hwaddr addr, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    unsigned block = (addr >> 24) & 15, reg = (addr & 0xffffff) >> 5;
    uint64_t value = reg < 128 ? s->csr_regs[block][reg] : UINT64_MAX;

    if (block == 8 && reg == 3) {
        value = (value & ~0xf0000000f0ULL) | s->mem_size_strap;
    } else if (block < 4 && block >= MACHINE(s)->smp.cpus) {
        value = 0;
    } else if (block < 4 && reg == 6) {
        /* CBCTL commander ID is the reader, even for another CPU's CSR.
         * CPU module specification table 37, fields 7:5 and 39:37.
         */
        unsigned id = current_cpu ? current_cpu->cpu_index : 0;
        unsigned code = (id + 1) & 3;

        value = (value & ~CBCTL_ID_MASK) | (uint64_t)code << 37 | code << 5;
    } else if (block < 4 && reg == 15) {
        /* C4-c supports four CPUs and expansion I/O (CPU spec table 46). */
        unsigned rev = 0x0a | (!strcmp(MACHINE(s)->cpu_type,
                                       ALPHA_CPU_TYPE_NAME("ev45")) ? 0x10 : 0);
        value = (value & ~0x1f0000001fULL) | (uint64_t)rev << 32 | rev;
    } else if (block >= 14 && reg == 0) {
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
    } else if (block < 4 && reg == 12) {
        value = (value & ~(BIT_ULL(32) | BIT_ULL(35))) |
                (s->intervals[block].pending ? BIT_ULL(32) : 0) |
                (s->csr_regs[block][11] & BIT_ULL(35));
    }
    if (size == 4) {
        value = (value >> ((addr & 4) * 8)) & UINT32_MAX;
    }
    trace_alpha2100a_csr(false, addr, value);
    trace_alpha2100a_csr_access(false, addr, value,
                              current_cpu ? ALPHA_CPU(current_cpu)->env.pc : 0,
                              current_cpu ? current_cpu->cpu_index : 0);
    return value;
}

static void lynx_csr_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    trace_alpha2100a_csr(true, addr, value);
    trace_alpha2100a_csr_access(true, addr, value,
                              current_cpu ? ALPHA_CPU(current_cpu)->env.pc : 0,
                              current_cpu ? current_cpu->cpu_index : 0);
    unsigned block = (addr >> 24) & 15, reg = (addr & 0xffffff) >> 5;
    uint64_t written = size == 4 ? value << ((addr & 4) * 8) : value;

    if (block < 4) {
        if (block >= MACHINE(s)->smp.cpus) {
            return;
        }
        if (reg == 12) {
            if (written & 4) {
                s->csr_regs[block][12] &= ~4ULL;
                qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[block]),
                                             ALPHA_CPU_INPUT_IRQ0), 0);
            }
            /* SIC is read status/write-one-to-clear, not a holding register. */
            if (written & BIT_ULL(32)) {
                s->intervals[block].pending = false;
                qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[block]),
                                             ALPHA_CPU_INPUT_IRQ4), 0);
            }
            if (written & BIT_ULL(35)) {
                s->csr_regs[block][11] &= ~BIT_ULL(35);
                qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[block]),
                                             ALPHA_CPU_INPUT_IRQ5), 0);
            }
            return;
        }
        if (reg == 1 || reg == 3 || reg == 5 || reg == 7) {
            s->csr_regs[block][reg] &= ~written;
            return;
        }
    }
    if (block == 8 && reg == 0) {
        s->csr_regs[block][0] &= ~written;
        return;
    }
    if (block >= 14 && (reg == 1 || reg == 4)) {
        s->csr_regs[block][reg] &= ~written;
        return;
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
        uint64_t halted = *storage & BIT_ULL(35);

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
        if (block == 14 && reg == 0x260 / 32) {
            /* HAE0_4 supplies PCI AD[31:30] for the 1 GiB dense aperture. */
            memory_region_set_alias_offset(&s->dense_mem, (*storage & 3) << 30);
        }
        if (block == 8 && reg == 3) {
            *storage = (*storage & ~0xf0000000f0ULL) | s->mem_size_strap;
            memory_region_set_enabled(&s->mem_diag, *storage & BIT_ULL(31));
        }
        if (block < 4 && reg == 11) {
            /* IPIR bit 0 is level controlled; bit 35 latches a node halt. */
            *storage = (*storage & ~BIT_ULL(35)) |
                       ((halted | written) & BIT_ULL(35));
            qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[block]),
                                         ALPHA_CPU_INPUT_IRQ3), *storage & 1);
            qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[block]),
                                         ALPHA_CPU_INPUT_IRQ5),
                         !!(*storage & BIT_ULL(35)));
        }
        if (block >= 14 && reg == 0) {
            *storage = (*storage & ~T3_IOCSR_REV_MASK) | T3_IOCSR_REV;
        }
        if (block >= 14 && reg == 0x4a0 / 32) {
            lynx_irq_update(s);
        }
    }
}

static uint32_t lynx_conf_addr(Alpha2100AMachine *s, hwaddr addr)
{
    uint32_t a = addr >> 5;

    if (!(s->csr_regs[14][0x240 / 32] & 0x40000000)) {
        uint32_t select = a >> 11;
        if (!is_power_of_2(select) || select > BIT(9)) {
            return UINT32_MAX;
        }
        a = ((ctz32(select) << 3) | ((a >> 8) & 7)) << 8 | (a & 255);
    }
    return a;
}

static uint64_t lynx_conf_read(void *opaque, hwaddr addr, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    uint32_t a = lynx_conf_addr(s, addr);
    unsigned len = ((addr >> 3) & 3) + 1;
    unsigned shift = len == 4 ? 0 : ((addr >> 5) & 3) * 8;

    return (uint64_t)pci_data_read(s->pci_bus, a, len) << shift;
}

static void lynx_conf_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    Alpha2100AMachine *s = opaque;
    uint32_t a = lynx_conf_addr(s, addr);
    unsigned len = ((addr >> 3) & 3) + 1;
    unsigned shift = len == 4 ? 0 : ((addr >> 5) & 3) * 8;

    if (a != UINT32_MAX) {
        pci_data_write(s->pci_bus, a, value >> shift, len);
    }
}

static int lynx_pci_map_irq(PCIDevice *dev, int pin)
{
    unsigned slot = PCI_SLOT(dev->devfn);
    /* Native Lynx slots 6..9 use IC positions 32..47. */
    return slot >= 6 && slot <= 9 ? 32 + (slot - 6) * 4 + pin : 28;
}

static void lynx_pci_set_irq(void *opaque, int irq, int level)
{
    lynx_irq_set(opaque, irq, level);
}

static const MemoryRegionOps lynx_io_ops = {
    .read = lynx_io_read, .write = lynx_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4, .valid.max_access_size = 8,
    .impl.min_access_size = 4, .impl.max_access_size = 8,
};
static const MemoryRegionOps lynx_mem_ops = {
    .read = lynx_mem_read, .write = lynx_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4, .valid.max_access_size = 4,
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
    memory_region_set_enabled(&s->mem_diag, false);
    memory_region_set_alias_offset(&s->dense_mem, 0);
    memset(s->mem_edc, 0, machine->ram_size / 32 * 2 * sizeof(uint16_t));
    memset(s->ic_regs, 0, sizeof(s->ic_regs));
    s->ic_regs[0][0x40 / 8] = s->ic_regs[1][0x40 / 8] = UINT64_MAX;
    s->irq_levels = s->irq_in_service = 0;
    s->nvram_page = 0;
    s->interval_epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (unsigned i = 0; i < machine->smp.cpus; i++) {
        LynxInterval *t = &s->intervals[i];

        /* The skipped SROM enables each installed processor's backup cache. */
        s->csr_regs[i][0] = 0x100000001ULL;
        cpu_reset(CPU(s->cpus[i]));
        s->cpus[i]->env.ipr.pal_base = SRM_BASE;
        cpu_set_pc(CPU(s->cpus[i]), SRM_BASE | R_PC_PAL_MODE_MASK);
        t->pending = false;
        t->ticks = 1;
        timer_mod(t->timer, s->interval_epoch +
                  (4 + i) * NANOSECONDS_PER_SECOND / 4096);
    }
}

static void lynx_init(MachineState *machine)
{
    Alpha2100AMachine *s = ALPHA2100A_MACHINE(machine);
    ISABus *isa;
    qemu_irq *irqs;
    ISADevice *rtc, *pcf;
    DeviceState *host;
    static const uint8_t io_eeprom[256] = { [0xfe] = 0x40, [0xff] = 0x40 };
    static const uint8_t mem_eeprom[256] = { 0 };
    static const uint8_t cpu_eeprom[256] = { [0xfe] = 0x20, [0xff] = 0x20 };
    Clock *refclk;
    g_autofree char *firmware = NULL;
    g_autoptr(GError) gerr = NULL;
    MemoryRegion *sysmem = get_system_memory();
    if (machine->ram_size < 32 * MiB || machine->ram_size > 512 * MiB) {
        error_report("AlphaServer 2100A experiment requires 32 MiB to 512 MiB RAM");
        exit(1);
    }
    memset(s->nvram, 0xff, sizeof(s->nvram));
    refclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(refclk, 275000000);
    for (unsigned i = 0; i < machine->smp.cpus; i++) {
        LynxInterval *t = &s->intervals[i];

        s->cpus[i] = ALPHA_CPU(object_new(machine->cpu_type));
        qdev_connect_clock_in(DEVICE(s->cpus[i]), "cpu-refclk", refclk);
        qdev_realize_and_unref(DEVICE(s->cpus[i]), NULL, &error_fatal);
        t->machine = s;
        t->id = i;
        t->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lynx_interval_tick, t);
    }
    memory_region_add_subregion(sysmem, 0, machine->ram);
    /* MCONF straps: DRAM density bits 5:4, number-of-banks bits 7:6.
     * This initial model has one installed memory module, at most 512 MiB.
     */
    unsigned strap;
    switch (machine->ram_size / MiB) {
    case 32: strap = 0x40; break;
    case 64: strap = 0x10; break;
    case 128: strap = 0x50; break;
    case 256: strap = 0x20; break;
    case 512: strap = 0x60; break;
    default:
        error_report("Alpha2100A memory module supports 32/64/128/256/512 MiB");
        exit(1);
    }
    s->mem_size_strap = (uint64_t)strap << 32 | strap;
    s->mem_edc = g_new0(uint16_t, machine->ram_size / 32 * 2);
    /* SRM runs in dirty backup-cache lines while testing its first DRAM bank.
     * The cacheless SROM shortcut needs separate backing for that temporary
     * diagnostic aperture, otherwise the test overwrites the running console.
     * This is not a coherent backup-cache or a complete bank-remapping model.
     */
    memory_region_init_ram_flags_nomigrate(&s->mem_test_store, OBJECT(machine),
                           "lynx.memory-test-store", machine->ram_size, 0,
                           &error_fatal);
    memory_region_init_io(&s->mem_diag, OBJECT(machine), &lynx_diag_ops, s,
                          "lynx.memory-diagnostic", machine->ram_size);
    memory_region_set_enabled(&s->mem_diag, false);
    memory_region_add_subregion_overlap(sysmem, 0x20000000,
                                        &s->mem_diag, 1);
    memory_region_init(&s->isa_io, OBJECT(machine), "lynx.isa-io", 0x10000);
    memory_region_init(&s->isa_mem, OBJECT(machine), "lynx.isa-mem", 0x100000000ULL);
    address_space_init(&s->io_as, &s->isa_io, "lynx.isa-io");
    address_space_init(&s->mem_as, &s->isa_mem, "lynx.isa-mem");
    memory_region_init_io(&s->sparse_mem, OBJECT(machine), &lynx_mem_ops, s,
                          "lynx.sparse-mem", 0x100000000ULL);
    memory_region_add_subregion(sysmem, 0x200000000ULL, &s->sparse_mem);
    memory_region_init_alias(&s->dense_mem, OBJECT(machine), "lynx.dense-mem",
                             &s->isa_mem, 0, GiB);
    memory_region_add_subregion(sysmem, 0x3c0000000ULL, &s->dense_mem);
    memory_region_init_io(&s->sparse_io, OBJECT(machine), &lynx_io_ops, s,
                          "lynx.sparse-io", 0x200000);
    memory_region_add_subregion(sysmem, LYNX_IO, &s->sparse_io);
    memory_region_init_io(&s->csr, OBJECT(machine), &lynx_csr_ops, s,
                          "lynx.csr", 0x10000000);
    memory_region_add_subregion(sysmem, LYNX_CSR, &s->csr);
    memory_region_init_io(&s->pci_conf, OBJECT(machine), &lynx_conf_ops, s,
                          "lynx.pci-config", 0x10000000);
    memory_region_add_subregion(sysmem, LYNX_CONF, &s->pci_conf);
    host = qdev_new("alpha2100a-pci-host");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(host), &error_fatal);
    s->pci_bus = pci_register_root_bus(host, "pci", lynx_pci_set_irq,
                                      lynx_pci_map_irq, s, &s->isa_mem,
                                      &s->isa_io, PCI_DEVFN(6, 0), 64,
                                      TYPE_PCI_BUS);
    PCI_HOST_BRIDGE(host)->bus = s->pci_bus;
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
    /* Synthetic identities, not dumps of proprietary DEC EEPROMs.
     * CPU specification 4.15.1: byte FE bits 7:5 identify board class.
     * Byte FF is the byte sum of the preceding contents.
     */
    at24c_eeprom_init_rom(I2C_BUS(qdev_get_child_bus(DEVICE(pcf), "lynx-i2c")),
                         0x56, 256, io_eeprom, sizeof(io_eeprom));
    at24c_eeprom_init_rom(I2C_BUS(qdev_get_child_bus(DEVICE(pcf), "lynx-i2c")),
                         0x50, 256, mem_eeprom, sizeof(mem_eeprom));
    at24c_eeprom_init_rom(I2C_BUS(qdev_get_child_bus(DEVICE(pcf), "lynx-i2c")),
                         0x54, 256, cpu_eeprom, sizeof(cpu_eeprom));
    if (machine->enable_graphics) {
        if (s->s3_928) {
            ISADevice *vga = isa_create_simple(isa, "jensen-s3-928");
            const char *romfile = s->s3_romfile ?: "vgabios.bin";

            memory_region_init_rom(&s->vga_rom, OBJECT(vga), "lynx.vga-rom",
                                   64 * KiB, &error_fatal);
            memory_region_add_subregion(&s->isa_mem, 0xc0000, &s->vga_rom);
            if (rom_add_file_mr(romfile, &s->vga_rom, -1) < 0) {
                exit(1);
            }
        } else {
            PCIDevice *vga;
            if (vga_interface_type == VGA_CIRRUS) {
                vga = pci_create_simple(s->pci_bus, PCI_DEVFN(6, 0),
                                        "alpha2100a-cirrus-5430");
            } else {
                vga = pci_vga_init(s->pci_bus);
            }
            /* SRM's x86 interpreter also scans the legacy C0000 aperture. */
            if (vga) {
                memory_region_init_rom(&s->vga_rom, OBJECT(vga), "lynx.vga-rom",
                                       64 * KiB, &error_fatal);
                memory_region_add_subregion(&s->isa_mem, 0xc0000, &s->vga_rom);
                if (rom_add_file_mr(vga_interface_type == VGA_CIRRUS ?
                                    "vgabios-cirrus.bin" : "vgabios.bin",
                                    &s->vga_rom, -1) < 0) {
                    exit(1);
                }
            }
        }
    }
    if (machine->smp.cpus > 1) {
        static const unsigned cpu_addr[] = { 0x54, 0x55, 0x57, 0x51 };
        for (unsigned i = 1; i < machine->smp.cpus; i++) {
            at24c_eeprom_init_rom(I2C_BUS(qdev_get_child_bus(DEVICE(pcf),
                                 "lynx-i2c")), cpu_addr[i], 256,
                                 cpu_eeprom, sizeof(cpu_eeprom));
        }
    }
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
    g_free(s->mem_edc);
    g_free(s->firmware);
    g_free(s->s3_romfile);
    for (unsigned i = 0; i < 4; i++) {
        timer_free(s->intervals[i].timer);
    }
    if (s->mem_as.root) {
        address_space_destroy(&s->mem_as);
    }
    if (s->io_as.root) {
        address_space_destroy(&s->io_as);
    }
}

static bool lynx_get_s3(Object *obj, Error **errp)
{
    return ALPHA2100A_MACHINE(obj)->s3_928;
}

static void lynx_set_s3(Object *obj, bool value, Error **errp)
{
    ALPHA2100A_MACHINE(obj)->s3_928 = value;
}

static char *lynx_get_rom(Object *obj, Error **errp)
{
    return g_strdup(ALPHA2100A_MACHINE(obj)->s3_romfile ?: "");
}

static void lynx_set_rom(Object *obj, const char *value, Error **errp)
{
    Alpha2100AMachine *s = ALPHA2100A_MACHINE(obj);
    g_free(s->s3_romfile);
    s->s3_romfile = *value ? g_strdup(value) : NULL;
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
    mc->max_cpus = 4;
    mc->default_display = "cirrus";
    object_class_property_add_bool(oc, "s3-928", lynx_get_s3, lynx_set_s3);
    object_class_property_add_str(oc, "s3-romfile", lynx_get_rom, lynx_set_rom);
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
static void lynx_cirrus_class_init(ObjectClass *oc, const void *data)
{
    /* SRM supports 5430/5434, not 5446. The shared Cirrus realization
     * already selects the 5430's 2 MiB layout from the class device ID.
     * Keep this board-local subtype separate from the default PCI 5446.
     */
    PCI_DEVICE_CLASS(oc)->device_id = 0x00a0;
    DEVICE_CLASS(oc)->desc = "Alpha2100A Cirrus CL-GD5430";
}

static const TypeInfo lynx_cirrus_type = {
    .name = "alpha2100a-cirrus-5430",
    .parent = "cirrus-vga",
    .class_init = lynx_cirrus_class_init,
};

static const TypeInfo lynx_host_type = {
    .name = "alpha2100a-pci-host",
    .parent = TYPE_PCI_HOST_BRIDGE,
};

static void lynx_register(void)
{
    type_register_static(&lynx_cirrus_type);
    type_register_static(&lynx_host_type);
    type_register_static(&lynx_type);
}
type_init(lynx_register)
