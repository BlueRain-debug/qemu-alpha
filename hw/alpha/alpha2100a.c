/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * AlphaServer 2100A (Lynx) EV4-family firmware bring-up experiment.
 * Hardware address definitions and sparse I/O lane behavior follow Linux
 * arch/alpha/include/asm/core_t2.h, based on Sable I/O Specification 1.3.
 * Original interface attribution: Jay A Estabrook (jestabro@amt.tay1.dec.com).
 * Implemented with QEMU ISA, AddressSpace, CPU and clock interfaces;
 * no Linux implementation is copied. No DEC firmware is distributed here.
 * ESC DMA register compatibility was cross-checked against 86Box src/dma.c:
 * Copyright 2008-2020 Sarah Walker.
 * Copyright 2016-2020 Miran Grca.
 * Copyright 2017-2020 Fred N. van Kempen.
 * Independently implemented from Intel 82374EB/SB, sections 3.2 and 6.7.
 * This is not yet a complete T2/T3 chipset or an operating-system machine.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
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
#include "hw/rtc/mc146818rtc_regs.h"
#include "hw/input/i8042.h"
#include "hw/block/fdc.h"
#include "hw/dma/i8257.h"
#include "hw/timer/i8254.h"
#include "hw/timer/i8254_internal.h"
#include "hw/char/serial-isa.h"
#include "hw/i2c/i2c.h"
#include "hw/nvram/eeprom_at24c.h"
#include "hw/display/vga.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci-bridge/dec_pci.h"
#include "hw/core/sysbus.h"
#include "system/system.h"
#include "system/physmem.h"
#include "system/reset.h"
#include "cpu.h"
#include "internals.h"
#include "trace.h"

#define TYPE_LYNX_IOMMU "alpha2100a-iommu"
#define TYPE_LYNX_DMA "alpha2100a-isa-dma"
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
    bool pending;
} LynxInterval;

/* Diagnostic bookkeeping for the documented direct-mapped B-cache.
 * Architectural RAM and LL/SC are still handled by QEMU's normal fast path.
 * This is not a full cache-coherent SMP or instruction-cache implementation.
 */
typedef struct LynxCacheLine {
    hwaddr tag;
    bool valid, dirty, corrected;
    uint8_t data[32], original[32];
} LynxCacheLine;

struct Alpha2100AMachine {
    MachineState parent_obj;
    AlphaCPU *cpus[4];
    MemoryRegion isa_io, isa_mem, sparse_io, sparse_mem, dense_mem;
    MemoryRegion csr, pci_conf, vga_rom, mem_diag, mem_test_store;
    uint16_t *mem_edc, *ram_edc;
    LynxCacheLine *bcache[4];
    /* Conservative observer hint, not hardware state: once a CPU has held
     * corrected data, EDC-disable must not bypass its cached copy. Keeping
     * the hint until reset avoids tracking a count across every eviction.
     */
    bool cache_correction[4];
    uint64_t mem_size_strap;
    bool mem_uncorrectable;
    PCIBus *pci_bus;
    bool s3_928;
    char *s3_romfile;
    AddressSpace io_as, mem_as, dma_as;
    I8257State *isa_dma[2];
    int64_t pit2_elapsed_ns;
    /* Extension register state; the i8257 transfer scheduler is still
     * limited to legacy counts. Ring-buffer stop execution is not modeled.
     */
    uint8_t dma_count_high[8];
    uint8_t dma_stop[8][3];
    IOMMUMemoryRegion dma_iommu;
    uint64_t csr_regs[16][128];
    uint8_t nvram[8192], nvram_page, rtc_index;
    char *nvram_filename;
    FILE *nvram_file;
    MC146818RtcState *rtc;
    ISADevice *pit;
    uint8_t system_control;
    uint8_t *firmware;
    gsize firmware_size;
    hwaddr firmware_base;
    bool firmware_is_arc;
    LynxInterval intervals[4];
    unsigned interval_phase;
    QEMUTimer *interval_timer;
    uint32_t interval_half_ticks;
    uint64_t interval_edges;
    int64_t interval_epoch;
    uint64_t ic_regs[2][16];
    uint64_t irq_levels, irq_latched, irq_in_service;
    qemu_irq *pic_irqs;
};

static uint64_t lynx_irq_pending(Alpha2100AMachine *s)
{
    /* AIR=0x40 selects the controller mask; one means disabled. */
    uint64_t level_mode = s->ic_regs[0][0x50 / 8];
    uint64_t requests = (s->irq_levels & level_mode) |
                        (s->irq_latched & ~level_mode);

    return requests & ~s->ic_regs[0][0x40 / 8] & ~s->irq_in_service;
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
        if (!(s->irq_levels & BIT_ULL(bit))) {
            s->irq_latched |= BIT_ULL(bit);
        }
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
    unsigned cpu = s->interval_phase;

    /* CPU module specification 4.12/table 48: consecutive SQW edges go to
     * slots 0, 1, 2, 3, even when a slot is empty. Each installed CPU sees
     * half the RTC square-wave frequency. NT programs this via RS/SQWE.
     */
    s->interval_phase = (cpu + 1) & 3;
    if (cpu < MACHINE(s)->smp.cpus) {
        s->intervals[cpu].pending = true;
        qemu_set_irq(qdev_get_gpio_in(DEVICE(s->cpus[cpu]),
                                     ALPHA_CPU_INPUT_IRQ4), 1);
    }
    s->interval_edges++;
    timer_mod(s->interval_timer, s->interval_epoch +
              periodic_clock_to_ns(s->interval_edges * s->interval_half_ticks));
}

static void lynx_interval_update(Alpha2100AMachine *s)
{
    unsigned a = s->rtc->cmos_data[RTC_REG_A];
    unsigned b = s->rtc->cmos_data[RTC_REG_B];
    uint32_t half_ticks = (b & REG_B_SQWE) && (a & 0x70) <= 0x20 ?
                         periodic_period_to_clock(a & 15) / 2 : 0;

    if (s->interval_half_ticks == half_ticks) {
        return;
    }
    s->interval_half_ticks = half_ticks;
    timer_del(s->interval_timer);
    if (half_ticks) {
        s->interval_epoch = qemu_clock_get_ns(rtc_clock);
        s->interval_edges = 1;
        timer_mod(s->interval_timer, s->interval_epoch +
                  periodic_clock_to_ns(half_ticks));
    }
}

/* One profile contains the 8 KiB configuration EEPROM followed by CMOS
 * bytes 0x0e..0x7f. Clock/calendar registers are regenerated from -rtc;
 * QEMU's century byte (0x32) is excluded from profile restoration.
 */
#define LYNX_PROFILE_SIZE (8192 + 114)

static void lynx_profile_write(Alpha2100AMachine *s, unsigned offset,
                                const uint8_t *data, unsigned size)
{
    if (s->nvram_file &&
        (fseek(s->nvram_file, offset, SEEK_SET) ||
         fwrite(data, 1, size, s->nvram_file) != size || fflush(s->nvram_file))) {
        error_report("Alpha2100A: cannot persist NVRAM profile '%s'",
                     s->nvram_filename);
        exit(EXIT_FAILURE);
    }
}

static void lynx_profile_init(Alpha2100AMachine *s)
{
    uint8_t image[LYNX_PROFILE_SIZE];
    bool create = false;

    if (!s->nvram_filename) {
        return;
    }
    s->nvram_file = g_fopen(s->nvram_filename, "r+b");
    if (!s->nvram_file && errno == ENOENT) {
        s->nvram_file = g_fopen(s->nvram_filename, "w+b");
        create = true;
    }
    if (!s->nvram_file) {
        error_report("Alpha2100A: cannot open NVRAM profile '%s': %s",
                     s->nvram_filename, strerror(errno));
        exit(EXIT_FAILURE);
    }
    if (create) {
        memcpy(image, s->nvram, sizeof(s->nvram));
        for (unsigned reg = 14; reg < 128; reg++) {
            image[8192 + reg - 14] = mc146818rtc_get_cmos_data(s->rtc, reg);
        }
        lynx_profile_write(s, 0, image, sizeof(image));
    } else {
        if (fseek(s->nvram_file, 0, SEEK_END) ||
            ftell(s->nvram_file) != sizeof(image) ||
            fseek(s->nvram_file, 0, SEEK_SET) ||
            fread(image, 1, sizeof(image), s->nvram_file) != sizeof(image)) {
            error_report("Alpha2100A: NVRAM profile must contain %u bytes",
                         (unsigned)sizeof(image));
            exit(EXIT_FAILURE);
        }
        memcpy(s->nvram, image, sizeof(s->nvram));
        for (unsigned reg = 14; reg < 128; reg++) {
            if (reg != 0x32) {
                mc146818rtc_set_cmos_data(s->rtc, reg, image[8192 + reg - 14]);
            }
        }
    }
}

/* The shared PIT leaves mode-0 GATE inhibition unimplemented. ARC uses
 * timer 2 with GATE low to verify both count bytes. Exclude the inhibited
 * interval from the counter's time base, then resume from the retained
 * count when GATE rises. Timer 2 has no independently scheduled IRQ.
 */
static void lynx_pit_sync(Alpha2100AMachine *s)
{
    PITChannelState *timer = &PIT_COMMON(s->pit)->channels[2];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (timer->mode == 0 && !timer->gate) {
        timer->count_load_time = now - s->pit2_elapsed_ns;
    } else {
        s->pit2_elapsed_ns = MAX(0, now - timer->count_load_time);
    }
}

static unsigned lynx_pit2_count(Alpha2100AMachine *s)
{
    PITChannelState *timer = &PIT_COMMON(s->pit)->channels[2];
    uint64_t ticks = muldiv64(s->pit2_elapsed_ns, PIT_FREQ,
                              NANOSECONDS_PER_SECOND);

    return (timer->count - ticks) & 0xffff;
}

/* ESC extension decoding; channel 4 is the inter-controller cascade. */
static int lynx_dma_count_channel(unsigned port)
{
    if (port >= 0x401 && port <= 0x407 && (port & 1)) {
        return (port - 0x401) / 2;
    }
    if (port >= 0x4c6 && port <= 0x4ce && (port & 3) == 2) {
        return 5 + (port - 0x4c6) / 4;
    }
    return -1;
}

static int lynx_dma_page_channel(unsigned port)
{
    static const int map[8] = { -1, 2, 3, 1, -1, -1, -1, 0 };
    int channel = map[port & 7];

    return channel < 0 ? -1 : channel + (port & 8 ? 4 : 0);
}

static uint64_t lynx_io_read(void *opaque, hwaddr addr, unsigned size)
{
    Alpha2100AMachine *s = opaque;
    unsigned port = addr >> 5, len = ((addr >> 3) & 3) + 1;
    unsigned shift = len == 4 ? 0 : (port & 3) * 8;
    uint64_t value = 0xffffffff;
    MemTxResult result = MEMTX_DECODE_ERROR;
    PITChannelState *pit2 = &PIT_COMMON(s->pit)->channels[2];
    int count_channel = lynx_dma_count_channel(port);

    if ((port >= 0x40 && port <= 0x43) || port == 0x61) {
        lynx_pit_sync(s);
    }
    if (port == 0x42 && len == 1 && pit2->mode == 0 && !pit2->gate &&
        !pit2->count_latched && !pit2->status_latched) {
        /* Do not let host time spent inside the port callback count as a
         * timer clock while GATE is low. Preserve the normal byte phase.
         */
        unsigned state = pit2->read_state;

        value = (lynx_pit2_count(s) >> (state == 1 || state == 3 ? 0 : 8)) & 255;
        if (state == 3 || state == 4) {
            pit2->read_state = state == 3 ? 4 : 3;
        }
        result = MEMTX_OK;
    } else if (len == 1) {
        value = address_space_ldub(&s->io_as, port, MEMTXATTRS_UNSPECIFIED, &result);
    } else if (len == 2) {
        value = address_space_lduw_le(&s->io_as, port, MEMTXATTRS_UNSPECIFIED, &result);
    } else if (len == 4) {
        value = address_space_ldl_le(&s->io_as, port, MEMTXATTRS_UNSPECIFIED, &result);
    }
    if (result != MEMTX_OK) {
        /* Empty EISA slots must not appear as a board with ID 00000000. */
        value = UINT32_MAX >> ((4 - len) * 8);
    }
    if (len == 1 && count_channel >= 0) {
        I8257State *dma = s->isa_dma[count_channel >> 2];
        I8257Regs *r = &dma->regs[count_channel & 3];
        uint32_t count = ((uint32_t)s->dma_count_high[count_channel] << 16) |
                         r->base[1];

        value = ((count - (r->now[1] >> dma->dshift)) >> 16) & 0xff;
    } else if (len == 1 && port >= 0x4e0 && port <= 0x4ff) {
        unsigned channel = (port - 0x4e0) / 4;
        unsigned byte = port & 3;

        value = channel == 4 || byte == 3 ? 0xff : s->dma_stop[channel][byte];
    } else if (port == 0x61 && len == 1) {
        PITChannelInfo timer, refresh;

        /* ESC system control: PIT2 OUT is bit 5, refresh PIT1 OUT is bit 4.
         * Repeated reads at the same virtual time must not toggle either.
         * No parity/channel-check error sources are modeled on this board.
         */
        pit_get_channel_info(PIT_COMMON(s->pit), 2, &timer);
        pit_get_channel_info(PIT_COMMON(s->pit), 1, &refresh);
        if (pit2->mode == 0 && !pit2->gate) {
            timer.out = s->pit2_elapsed_ns >=
                muldiv64(pit2->count, NANOSECONDS_PER_SECOND, PIT_FREQ);
        }
        value = s->system_control | (timer.out << 5) | (refresh.out << 4);
    } else if (len == 1 && (port == 0x0f || port == 0xde)) {
        /* The ESC adds readback of the four legacy DMA channel masks. */
        value = s->isa_dma[port == 0xde]->mask & 0x0f;
    } else if (port == 0xc00) {
        value = s->nvram_page;
    } else if (port >= 0xc80 && port + len <= 0xc84) {
        /* Sable/Lynx system-board EISA ID: DEC2A01. */
        static const uint8_t id[4] = { 0x10, 0xa3, 0x2a, 0x01 };

        value = 0;
        for (unsigned i = 0; i < len; i++) {
            value |= (uint32_t)id[port - 0xc80 + i] << (i * 8);
        }
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
    PITChannelState *pit2 = &PIT_COMMON(s->pit)->channels[2];
    int64_t old_load;
    bool count_latched = pit2->count_latched;
    bool status_latched = pit2->status_latched;

    value >>= len == 4 ? 0 : (port & 3) * 8;
    trace_alpha2100a_io(true, port, len, value);
    if ((port >= 0x40 && port <= 0x43) || port == 0x61) {
        lynx_pit_sync(s);
    }
    old_load = pit2->count_load_time;
    if (port == 0x61 && len == 1) {
        s->system_control = value & 0x0f;
        pit_set_gate(PIT_COMMON(s->pit), 2, value & 1);
        return;
    }
    if (len == 1) {
        int channel = lynx_dma_count_channel(port);

        if (channel >= 0) {
            s->dma_count_high[channel] = value;
            return;
        }
        if (port >= 0x4e0 && port <= 0x4ff) {
            unsigned ch = (port - 0x4e0) / 4;
            unsigned byte = port & 3;

            if (ch != 4 && byte != 3) {
                s->dma_stop[ch][byte] = value & (byte ? 0xff : 0xfc);
            }
            return;
        }
        if ((port >= 0x80 && port <= 0x8f) || port < 8 ||
            (port >= 0xc0 && port <= 0xce && !(port & 1))) {
            /* Programming any legacy address byte selects compatibility
             * mode and clears the high page (82374EB/SB 6.7.1). Ordinary
             * count writes likewise clear the third count byte (3.2.2).
             */
            if (port >= 0x80 && port <= 0x8f) {
                channel = lynx_dma_page_channel(port);
            } else {
                unsigned dshift = port >= 0xc0;
                unsigned reg = (port - (dshift ? 0xc0 : 0)) >> dshift;

                channel = (reg >> 1) + dshift * 4;
                if (reg & 1) {
                    s->dma_count_high[channel] = 0;
                    channel = -1;
                }
            }
            if (channel >= 0) {
                s->isa_dma[channel >> 2]->regs[channel & 3].pageh = 0;
            }
        } else if (port == 0x0d || port == 0xda) {
            unsigned controller = port == 0xda;

            for (unsigned i = 0; i < 4; i++) {
                s->isa_dma[controller]->regs[i].pageh = 0;
            }
        }
    }
    if (port == 0x70) {
        s->rtc_index = value & 127;
    } else if (port == 0x71 && s->rtc_index >= 14 && s->rtc_index != 0x32) {
        uint8_t byte = value;

        lynx_profile_write(s, 8192 + s->rtc_index - 14, &byte, 1);
    }
    if (port == 0xc00) {
        s->nvram_page = value & 31;
        return;
    } else if (port >= 0xc80 && port + len <= 0xc84) {
        return;
    } else if (port >= 0x800 && port + len <= 0x900) {
        unsigned offset = s->nvram_page * 256 + port - 0x800;

        for (unsigned i = 0; i < len; i++) {
            s->nvram[s->nvram_page * 256 + port - 0x800 + i] = value >> (i * 8);
        }
        lynx_profile_write(s, offset, s->nvram + offset, len);
        return;
    }
    if (len == 1) {
        address_space_stb(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    } else if (len == 2) {
        address_space_stw_le(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    } else if (len == 4) {
        address_space_stl_le(&s->io_as, port, value, MEMTXATTRS_UNSPECIFIED, NULL);
    }
    if (port == 0x71 &&
        (s->rtc_index == RTC_REG_A || s->rtc_index == RTC_REG_B)) {
        lynx_interval_update(s);
    }
    if (port == 0x42 && pit2->count_load_time != old_load) {
        s->pit2_elapsed_ns = 0;
    }
    if (port == 0x43 && pit2->mode == 0 && !pit2->gate) {
        if (!count_latched && pit2->count_latched) {
            pit2->latched_count = lynx_pit2_count(s);
        }
        if (!status_latched && pit2->status_latched) {
            unsigned out = s->pit2_elapsed_ns >=
                muldiv64(pit2->count, NANOSECONDS_PER_SECOND, PIT_FREQ);

            pit2->status = (pit2->status & 0x7f) | out << 7;
        }
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
    /* Only these generator columns have been established. An arbitrary
     * stack/heap line must not be assigned a fabricated zero-data code.
     * Recognize the known linear basis around all-zero and all-one data.
     */
    uint32_t fill = ldl_le_p(line + (4 + slice) * 4);
    uint16_t code;

    if (fill != 0 && fill != UINT32_MAX) {
        return UINT16_MAX;
    }
    code = fill ? 0x3eb : 0xc14;
    for (unsigned i = 0; i < 4; i++) {
        uint32_t delta = ldl_le_p(line + (i * 2 + slice) * 4) ^ fill;
        uint32_t known = i == 0 ? 3 : i == 1 ? 1 : 0;

        if (delta & ~known) {
            return UINT16_MAX;
        }
        if (i == 0) {
            code ^= (delta & 1) ? 0x37 : 0;
            code ^= (delta & 2) ? 0x19 : 0;
        } else if (i == 1) {
            code ^= (delta & 1) ? 0x8b0 : 0;
        }
    }
    return code;
}

static void lynx_mem_capture(Alpha2100AMachine *s, hwaddr physical,
                              uint8_t *line, bool read, uint16_t *codes)
{
    uint64_t control = s->csr_regs[8][6];
    uint64_t latch = 0, latch_mask = 0;
    uint64_t flags = 0, syndrome_latch = 0;
    uint16_t generated_codes[2] = { lynx_mem_code(line, 0),
                                    lynx_mem_code(line, 1) };

    if (generated_codes[0] == UINT16_MAX ||
        generated_codes[1] == UINT16_MAX) {
        /* The full 128-bit generator matrix remains unimplemented. Keep
         * unknown data outside this limited diagnostic injection model.
         */
        if (!read) {
            codes[0] = codes[1] = UINT16_MAX;
        }
        return;
    }
    for (unsigned slice = 0; slice < 2; slice++) {
        uint32_t ctl = control >> (slice * 32);
        uint16_t generated = generated_codes[slice];
        uint16_t code;
        uint16_t syndrome;
        uint32_t status = 0, correction = 0;
        unsigned word = slice;

        if (!read) {
            codes[slice] = (ctl & BIT(13)) ?
                           ((ctl >> 16) & 0xfff) | 0x8000 : generated;
        } else if (codes[slice] == 0xffff) {
            codes[slice] = generated;
        }
        code = codes[slice] & 0xfff;
        latch |= (uint64_t)(code * 0x10001) << (slice * 32);
        /* MEDCC capture inhibit is independent from error reporting.
         * Native diagnostics set bit 29 after the controlled writeback.
         * Normal stack/victim traffic must not replace that EDC snapshot.
         */
        if (!(ctl & (BIT(29) | BIT(31)))) {
            latch_mask |= (uint64_t)UINT32_MAX << (slice * 32);
        }
        syndrome = (ctl & BIT(31)) ? 0 :
                   (ctl & BIT(12)) ? (ctl & 0xfff) ^ generated :
                   (codes[slice] & 0x8000) ? code ^ generated : 0;
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
                /* A fault takes a new snapshot even when normal capture
                 * is inhibited; MERR below holds it until acknowledgement.
                 */
                latch_mask |= (uint64_t)UINT32_MAX << (slice * 32);
                s->mem_uncorrectable |= !correction;
                flags |= (uint64_t)(correction ? 0x40001 : 0x10001)
                         << (slice * 32);
                syndrome_latch |= (uint64_t)status << (slice * 32);
            }
        }
    }
    if (flags) {
        /* Both CSIC slices latch the same bus transaction on an error. */
        latch_mask = UINT64_MAX;
    }
    if ((!read || flags) && latch_mask &&
        !(s->csr_regs[8][0] & 0x100000001ULL)) {
        trace_alpha2100a_capture(read, physical, latch, control,
            current_cpu ? ALPHA_CPU(current_cpu)->env.pc : 0);
        s->csr_regs[8][4] = (s->csr_regs[8][4] & ~latch_mask) |
                             (latch & latch_mask);
    }
    if (flags && !(s->csr_regs[8][0] & 0x100000001ULL)) {
        s->csr_regs[8][0] |= flags;
        s->csr_regs[8][5] = syndrome_latch;
        unsigned commander = current_cpu ? (current_cpu->cpu_index + 1) & 3 : 1;
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

static uint8_t *lynx_ram_line(Alpha2100AMachine *s, hwaddr addr,
                              uint16_t **codes)
{
    if (addr < MACHINE(s)->ram_size) {
        *codes = s->ram_edc + (addr / 32) * 2;
        return memory_region_get_ram_ptr(MACHINE(s)->ram) + addr;
    }
    addr -= 0x20000000;
    assert(addr < MACHINE(s)->ram_size);
    *codes = s->mem_edc + (addr / 32) * 2;
    return memory_region_get_ram_ptr(&s->mem_test_store) + addr;
}

static MemTxResult lynx_cache_data(Alpha2100AMachine *s, hwaddr addr,
                                   unsigned size, bool write, uint64_t *value)
{
    unsigned cpu = current_cpu ? current_cpu->cpu_index : 0;
    uint64_t biu = s->cpus[cpu]->env.ipr.biu_ctl;
    unsigned capacity = extract64(biu, 28, 3) == 5 ? 4 * MiB : MiB;
    bool cached = current_cpu && (biu & 1) &&
                  !(biu & BIT_ULL(32 + (addr >> 32)));
    bool diagnostic = s->csr_regs[8][3] & BIT_ULL(31);
    hwaddr base = addr & ~31ULL;
    uint16_t *codes;
    uint8_t *ram = lynx_ram_line(s, base, &codes);
    LynxCacheLine *line = &s->bcache[cpu][(addr & (capacity - 1)) / 32];
    uint8_t data[32];
    bool adjusted = false;

    /* QEMU RAM remains authoritative for DMA, atomic and other-CPU writes.
     * Never let a retained diagnostic correction hide a later RAM update.
     * Such a write also replaces the old injected EDC for this abstraction.
     */
    if (cached && line->valid && line->tag == base && line->corrected &&
        memcmp(line->original, ram, sizeof(line->original))) {
        line->valid = false;
        codes[0] = codes[1] = 0xffff;
    }
    if (s->csr_regs[8][6] &
        (BIT_ULL(12) | BIT_ULL(44) | BIT_ULL(13) | BIT_ULL(45))) {
        trace_alpha2100a_cache(write, addr, cached,
            line->valid && line->tag == base, s->csr_regs[8][6],
            current_cpu ? ALPHA_CPU(current_cpu)->env.pc : 0);
    }
    s->mem_uncorrectable = false;
    if (!cached) {
        memcpy(data, ram, sizeof(data));
        if (diagnostic) {
            lynx_mem_capture(s, base, data, !write, codes);
            adjusted = !write && memcmp(data, ram, sizeof(data)) != 0;
        }
    } else {
        if (!line->valid || (line->tag != base && !(biu & 8))) {
            if (line->valid && line->dirty && diagnostic) {
                uint16_t *victim_codes;
                lynx_ram_line(s, line->tag, &victim_codes);
                lynx_mem_capture(s, line->tag, line->data, false, victim_codes);
            }
            line->tag = base;
            line->valid = true;
            line->dirty = line->corrected = false;
            memcpy(line->data, ram, sizeof(line->data));
            memcpy(line->original, ram, sizeof(line->original));
            if (diagnostic && !write) {
                lynx_mem_capture(s, base, line->data, true, codes);
                line->corrected = memcmp(line->data, ram, sizeof(line->data)) != 0;
                if (line->corrected) {
                    s->cache_correction[cpu] = true;
                    s->cpus[cpu]->env.memory_data_active = true;
                }
            }
        }
        if (write) {
            memcpy(line->data, ram, sizeof(line->data));
            line->dirty = true;
            line->corrected = false;
            if (lynx_mem_code(ram, 0) == UINT16_MAX ||
                lynx_mem_code(ram, 1) == UINT16_MAX) {
                /* Discard unmodelled cache bookkeeping after the write.
                 * Keep RAM authoritative instead of retaining a dirty line
                 * which has no meaningful code in this limited model.
                 */
                codes[0] = codes[1] = UINT16_MAX;
                line->valid = line->dirty = false;
            }
        }
        adjusted = line->corrected;
        memcpy(data, adjusted ? line->data : ram, sizeof(data));
    }
    if (!write && adjusted) {
        switch (size) {
        case 1: *value = data[addr & 31]; break;
        case 2: *value = lduw_le_p(data + (addr & 31)); break;
        case 4: *value = ldl_le_p(data + (addr & 31)); break;
        case 8: *value = ldq_le_p(data + (addr & 31)); break;
        }
        trace_alpha2100a_correction(addr, size, *value, cpu,
            current_cpu ? ALPHA_CPU(current_cpu)->env.pc : 0);
    }
    if (cached && s->mem_uncorrectable) {
        line->valid = false; /* CACK_HERR did not complete a valid fill. */
    }
    return s->mem_uncorrectable ? MEMTX_ERROR : MEMTX_OK;
}

static MemTxResult lynx_ram_data(void *opaque, hwaddr addr, unsigned size,
                                 bool write, uint64_t *value)
{
    Alpha2100AMachine *s = opaque;
    unsigned cpu = current_cpu ? current_cpu->cpu_index : 0;
    unsigned capacity;
    LynxCacheLine *line;
    uint16_t *codes;
    uint8_t *ram;
    bool tracked_victim;

    if (addr >= MACHINE(s)->ram_size ||
        size > MACHINE(s)->ram_size - addr) {
        return MEMTX_OK; /* I/O and the test aperture have their own callbacks. */
    }
    capacity = extract64(s->cpus[cpu]->env.ipr.biu_ctl, 28, 3) == 5 ?
               4 * MiB : MiB;
    line = &s->bcache[cpu][(addr & (capacity - 1)) / 32];
    codes = s->ram_edc + (addr / 32) * 2;
    ram = memory_region_get_ram_ptr(MACHINE(s)->ram) + (addr & ~31ULL);
    /* A healthy B-cache read hit has no controller transaction. This is
     * per-CPU bookkeeping: secondary PAL spin loops need no global lock.
     * Corrected copies still need the external-RAM consistency check below.
     */
    if (!write && line->valid && line->tag == (addr & ~31ULL) &&
        !line->corrected && (s->cpus[cpu]->env.ipr.biu_ctl & 1) &&
        !(s->cpus[cpu]->env.ipr.biu_ctl & BIT_ULL(32 + (addr >> 32)))) {
        return MEMTX_OK;
    }
    tracked_victim = line->valid &&
        (line->dirty || line->corrected ||
         (lynx_mem_code(line->data, 0) != UINT16_MAX &&
          lynx_mem_code(line->data, 1) != UINT16_MAX));
    /* Unmodelled data with no injected code/correction has no diagnostic
     * side effects. In particular, normal PAL interrupt stacks must not
     * serialize all CPUs through the BQL on every word access.
     */
    if ((codes[0] == UINT16_MAX || !(codes[0] & 0x8000)) &&
        (codes[1] == UINT16_MAX || !(codes[1] & 0x8000)) &&
        !tracked_victim &&
        (lynx_mem_code(ram, 0) == UINT16_MAX ||
         lynx_mem_code(ram, 1) == UINT16_MAX)) {
        return MEMTX_OK;
    }
    BQL_LOCK_GUARD();
    return lynx_cache_data(s, addr, size, write, value);
}

static MemTxResult lynx_diag_read(void *opaque, hwaddr addr, uint64_t *value,
                                  unsigned size, MemTxAttrs attrs)
{
    Alpha2100AMachine *s = opaque;
    uint8_t *ram = memory_region_get_ram_ptr(&s->mem_test_store);

    *value = size == 4 ? ldl_le_p(ram + addr) : ldq_le_p(ram + addr);
    return lynx_cache_data(s, 0x20000000 + addr, size, false, value);
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
    lynx_cache_data(s, 0x20000000 + addr, size, true, &value);
    memory_region_set_dirty(&s->mem_test_store, addr, size);
}

static const MemoryRegionOps lynx_diag_ops = {
    .read_with_attrs = lynx_diag_read, .write = lynx_diag_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4, .valid.max_access_size = 8,
    .impl.min_access_size = 4, .impl.max_access_size = 8,
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
        /* IOCSR<35:33> is the read-only revision. PINT<6> reflects the
         * interrupt-controller output, even when PAL masks CPU interrupts.
         * Original ARC polls this pin before acknowledging a floppy IRQ.
         */
        value = (value & ~(T3_IOCSR_REV_MASK | BIT_ULL(6))) | T3_IOCSR_REV;
        if (block == 14 && lynx_irq_pending(s)) {
            value |= BIT_ULL(6);
        }
    } else if (block >= 14 && reg == 0x480 / 32) {
        uint64_t pending = block == 14 ? lynx_irq_pending(s) : 0;

        value = pending ? ctz64(pending) : 0x80;
        if (pending) {
            /* IC mode register 0x50: one is level-sensitive, zero is
             * latched. A claimed ISA edge must not recur merely because
             * the device leaves its request high until a deferred read.
             */
            s->irq_latched &= ~BIT_ULL(value);
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
        if (block == 8 && reg == 6) {
            bool active = !(*storage & BIT_ULL(31)) ||
                          !(*storage & BIT_ULL(63));

            /* Pre-injection ordinary reads also flush clean cache lines.
             * Observe them while either CSIC slice checks data, even before
             * forcing is enabled. Unknown, side-effect-free data bypasses
             * the BQL in lynx_ram_data.
             */
            for (unsigned i = 0; i < MACHINE(s)->smp.cpus; i++) {
                s->cpus[i]->env.memory_data_active = active ||
                                                     s->cache_correction[i];
            }
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
            *storage = (*storage & ~(T3_IOCSR_REV_MASK | BIT_ULL(6))) |
                       T3_IOCSR_REV;
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

/* T2/T3 PCI target windows, I/O specification 5.2.1.1. A PTE maps one
 * 8 KiB page, with valid in bit 0 and the PFN in bits 19:1. There is no
 * emulated IOTLB: every software DMA transaction sees the current registers
 * and page table. Error-status/interrupt details remain unimplemented.
 */
static IOMMUTLBEntry lynx_dma_translate(IOMMUMemoryRegion *mr, hwaddr addr,
                                       IOMMUAccessFlags flags, int index)
{
    Alpha2100AMachine *s = container_of(mr, Alpha2100AMachine, dma_iommu);
    IOMMUTLBEntry entry = {
        .target_as = &address_space_memory,
        .iova = addr & ~0x1fffULL,
        .addr_mask = 0x1fff,
        .perm = IOMMU_NONE,
    };

    if (addr > UINT32_MAX) {
        return entry;
    }
    for (unsigned i = 0; i < 2; i++) {
        uint64_t base = s->csr_regs[14][10 + i * 3];
        uint64_t mask = (s->csr_regs[14][11 + i * 3] & 0x7ff00000) | 0xfffff;
        uint64_t translated = s->csr_regs[14][12 + i * 3];
        uint64_t start = base & 0xfff00000;
        uint64_t end = ((base & 0xfff) << 20) | 0xfffff;
        bool sg = base & BIT(18);

        if (!(base & BIT(19)) || addr < start || addr > end) {
            continue;
        }
        if (sg) {
            MemTxResult result;
            hwaddr pte_addr = (((translated & 0x3ffffe00) << 1) & ~(mask >> 10)) |
                              (((addr & mask) >> 10) & ~7ULL);
            uint64_t pte = address_space_ldq_le(&address_space_memory, pte_addr,
                                               MEMTXATTRS_UNSPECIFIED, &result);

            if (result != MEMTX_OK || !(pte & 1) || (pte >> 20)) {
                trace_alpha2100a_dma(addr, 0, i + 1, sg, false);
                return entry;
            }
            entry.translated_addr = (pte & 0xffffe) << 12;
        } else {
            entry.translated_addr = ((translated & 0x7ff00000) & ~mask) |
                                    (addr & mask & ~0x1fffULL);
        }
        entry.perm = IOMMU_RW;
        trace_alpha2100a_dma(addr, entry.translated_addr, i + 1, sg, true);
        return entry;
    }
    trace_alpha2100a_dma(addr, 0, 0, false, false);
    return entry;
}

static AddressSpace *lynx_dma_address_space(PCIBus *bus, void *opaque, int devfn)
{
    Alpha2100AMachine *s = opaque;

    return &s->dma_as;
}

static const PCIIOMMUOps lynx_dma_ops = {
    .get_address_space = lynx_dma_address_space,
};

static void lynx_iommu_class_init(ObjectClass *oc, const void *data)
{
    IOMMU_MEMORY_REGION_CLASS(oc)->translate = lynx_dma_translate;
}

static const TypeInfo lynx_iommu_type = {
    .name = TYPE_LYNX_IOMMU,
    .parent = TYPE_IOMMU_MEMORY_REGION,
    .class_init = lynx_iommu_class_init,
};

/* The 82374's legacy channels issue PCI DMA through T3, not directly to
 * CPU physical RAM. Reuse the i8257 registers, requests and scheduling; only
 * the board-local memory interface differs. Enhanced ESC descriptors and
 * EISA chaining are not modeled yet.
 */
typedef struct LynxDMA {
    I8257State parent_obj;
    Alpha2100AMachine *machine;
} LynxDMA;

static int lynx_isa_dma_memory(IsaDma *obj, int channel, void *buf, int pos,
                               int len, bool write)
{
    LynxDMA *d = (LynxDMA *)obj;
    I8257State *dma = &d->parent_obj;
    I8257Regs *r = &dma->regs[channel & 3];
    hwaddr addr = (uint64_t)(r->pageh & 0x7f) << 24 |
                  (uint32_t)r->page << 16 | r->now[0];

    if (!(r->mode & 0x0c)) {
        if (!write) {
            memset(buf, 0, len);
        }
        return len;
    }
    if (r->mode & 0x20) {
        /* Address decrement preserves byte order within a 16-bit transfer. */
        unsigned unit = 1 << dma->dshift;

        for (int done = 0; done < len; done += unit) {
            unsigned count = MIN(unit, len - done);

            if (address_space_rw(&d->machine->dma_as, addr - pos - done,
                                 MEMTXATTRS_UNSPECIFIED, (uint8_t *)buf + done,
                                 count, write) != MEMTX_OK) {
                return done;
            }
        }
    } else if (address_space_rw(&d->machine->dma_as, addr + pos,
                                MEMTXATTRS_UNSPECIFIED, buf, len, write) !=
               MEMTX_OK) {
        return 0;
    }
    return len;
}

static int lynx_isa_dma_read(IsaDma *obj, int channel, void *buf, int pos, int len)
{
    return lynx_isa_dma_memory(obj, channel, buf, pos, len, false);
}

static int lynx_isa_dma_write(IsaDma *obj, int channel, void *buf, int pos, int len)
{
    return lynx_isa_dma_memory(obj, channel, buf, pos, len, true);
}

static void lynx_dma_class_init(ObjectClass *oc, const void *data)
{
    IsaDmaClass *dc = ISADMA_CLASS(oc);

    dc->read_memory = lynx_isa_dma_read;
    dc->write_memory = lynx_isa_dma_write;
}

static const TypeInfo lynx_dma_type = {
    .name = TYPE_LYNX_DMA,
    .parent = TYPE_I8257,
    .instance_size = sizeof(LynxDMA),
    .class_init = lynx_dma_class_init,
};

static void lynx_isa_dma_init(Alpha2100AMachine *s, ISABus *isa)
{
    ISADevice *channels[2];

    for (unsigned i = 0; i < 2; i++) {
        ISADevice *dev = isa_new(TYPE_LYNX_DMA);

        ((LynxDMA *)dev)->machine = s;
        object_property_add_child(OBJECT(s), "dma[*]", OBJECT(dev));
        qdev_prop_set_int32(DEVICE(dev), "base", i ? 0xc0 : 0);
        qdev_prop_set_int32(DEVICE(dev), "page-base", i ? 0x88 : 0x80);
        qdev_prop_set_int32(DEVICE(dev), "pageh-base", i ? 0x488 : 0x480);
        qdev_prop_set_int32(DEVICE(dev), "dshift", i);
        isa_realize_and_unref(dev, isa, &error_fatal);
        channels[i] = dev;
        s->isa_dma[i] = I8257(dev);
    }
    isa_bus_dma(isa, ISADMA(channels[0]), ISADMA(channels[1]));
}

static int lynx_pci_map_irq(PCIDevice *dev, int pin)
{
    unsigned slot = PCI_SLOT(dev->devfn);
    /* Root IDSEL4 is the Lynx VGA source, not the Demi-Lynx NCR slot. */
    if (slot == 4) {
        return 24;
    }
    /* Native primary expansion slots 6..9 use IC positions 32..47. */
    return slot >= 6 && slot <= 9 ? 32 + (slot - 6) * 4 + pin : 28;
}

static int lynx_secondary_map_irq(PCIDevice *dev, int pin)
{
    unsigned slot = PCI_SLOT(dev->devfn);

    /* Built-in NCR810 is secondary IDSEL1, wired to IC28 on all pins.
     * Expansion IDSEL6..9 are physical slots 0..3, IC48..63. These are
     * board wires, not the rotating swizzle used by a plug-in bridge.
     */
    return slot >= 6 && slot <= 9 ? 48 + (slot - 6) * 4 + pin : 28;
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
    s->pit2_elapsed_ns = 0;
    memset(s->dma_count_high, 0, sizeof(s->dma_count_high));
    for (unsigned controller = 0; controller < 2; controller++) {
        for (unsigned channel = 0; channel < 4; channel++) {
            s->isa_dma[controller]->regs[channel].pageh = 0;
        }
    }
    if (s->firmware_is_arc) {
        unsigned mode = mc146818rtc_get_cmos_data(s->rtc, 0x0b);

        /* ARC interprets the TOY in binary, 24-hour mode with epoch 1980.
         * Convert the initial RTC fields before changing DM; subsequent
         * hardware resets already retain binary format.
         */
        if (!(mode & 4)) {
            static const uint8_t fields[] = { 0, 2, 4, 6, 7, 8, 9, 0x32 };

            for (unsigned i = 0; i < ARRAY_SIZE(fields); i++) {
                unsigned reg = fields[i];
                unsigned value = mc146818rtc_get_cmos_data(s->rtc, reg);

                mc146818rtc_set_cmos_data(s->rtc, reg,
                                         (value >> 4) * 10 + (value & 15));
            }
        }
        mc146818rtc_set_cmos_data(s->rtc, 0x0b, mode | 6);
    }
    /* Replace the skipped SROM's initial low-RAM state on every reset. */
    physical_memory_write(0, scratch, sizeof(scratch));
    physical_memory_write(s->firmware_base, s->firmware, s->firmware_size);
    memset(s->csr_regs, 0, sizeof(s->csr_regs));
    memory_region_set_enabled(&s->mem_diag, false);
    memory_region_set_alias_offset(&s->dense_mem, 0);
    memset(s->mem_edc, 0xff, machine->ram_size / 32 * 2 * sizeof(uint16_t));
    memset(s->ram_edc, 0xff, machine->ram_size / 32 * 2 * sizeof(uint16_t));
    for (unsigned i = 0; i < machine->smp.cpus; i++) {
        memset(s->bcache[i], 0, 4 * MiB / 32 * sizeof(LynxCacheLine));
        s->cache_correction[i] = false;
    }
    memset(s->ic_regs, 0, sizeof(s->ic_regs));
    s->ic_regs[0][0x40 / 8] = s->ic_regs[1][0x40 / 8] = UINT64_MAX;
    s->irq_levels = s->irq_latched = s->irq_in_service = 0;
    s->nvram_page = 0;
    s->system_control = 0;
    s->interval_phase = 0;
    timer_del(s->interval_timer);
    s->interval_half_ticks = 0;
    for (unsigned i = 0; i < machine->smp.cpus; i++) {
        LynxInterval *t = &s->intervals[i];

        /* The skipped SROM enables each installed processor's backup cache. */
        s->csr_regs[i][0] = 0x100000001ULL;
        cpu_reset(CPU(s->cpus[i]));
        s->cpus[i]->env.memory_data_active = true;
        s->cpus[i]->env.ipr.pal_base = s->firmware_base;
        if (s->firmware_is_arc) {
            /* SROM passes total RAM bytes in A1. ARC's PAL bootstrap also
             * uses A1!=0 to distinguish a cold start from warm re-entry;
             * zero incorrectly skips heap initialization on the first boot.
             */
            alpha_cpu_set_reg_value(&s->cpus[i]->env, 17, machine->ram_size);
        }
        cpu_set_pc(CPU(s->cpus[i]), s->firmware_base | R_PC_PAL_MODE_MASK);
        t->pending = false;
    }
    lynx_interval_update(s);
}

static void lynx_init(MachineState *machine)
{
    Alpha2100AMachine *s = ALPHA2100A_MACHINE(machine);
    ISABus *isa;
    qemu_irq *irqs;
    ISADevice *rtc, *pcf, *fdc;
    DriveInfo *floppies[MAX_FD];
    DeviceState *host;
    PCIDevice *bridge;
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
    firmware = machine->firmware ?
        qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware) : NULL;
    if (!firmware || !g_file_get_contents(firmware, (char **)&s->firmware,
                                         &s->firmware_size, &gerr)) {
        error_report("AlphaServer 2100A: supply extracted EV4 firmware with -bios%s%s",
                     gerr ? ": " : "", gerr ? gerr->message : "");
        exit(1);
    }
    if (s->firmware_size >= 8 && s->firmware_size <= 4 * MiB &&
        ldl_le_p(s->firmware) == 0x207f0000) {
        s->firmware_base = SRM_BASE;
    } else if (s->firmware_size >= 32 && s->firmware_size <= 4 * MiB &&
               ldl_le_p(s->firmware + 24) == 0x4000 &&
               ldl_le_p(s->firmware + 28) == 0xfeceface) {
        /* The original ARC bootstrap enters its linked firmware body at
         * KSEG0+0x704100. Load the whole PAL/firmware image at 0x700000,
         * as the SROM handoff does; the header's 0x4000 is not a load base.
         * On-board flash and an SRM->ARC handoff remain unimplemented.
         */
        s->firmware_base = 0x700000;
        s->firmware_is_arc = true;
    } else {
        error_report("AlphaServer 2100A requires an extracted EV4 SRM or ARC image");
        exit(1);
    }
    memset(s->nvram, 0xff, sizeof(s->nvram));
    refclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(refclk, 275000000);
    for (unsigned i = 0; i < machine->smp.cpus; i++) {
        s->cpus[i] = ALPHA_CPU(object_new(machine->cpu_type));
        s->bcache[i] = g_new0(LynxCacheLine, 4 * MiB / 32);
        s->cpus[i]->memory_data = lynx_ram_data;
        s->cpus[i]->memory_data_opaque = s;
        qdev_connect_clock_in(DEVICE(s->cpus[i]), "cpu-refclk", refclk);
        qdev_realize_and_unref(DEVICE(s->cpus[i]), NULL, &error_fatal);
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
    s->mem_edc = g_new(uint16_t, machine->ram_size / 32 * 2);
    s->ram_edc = g_new(uint16_t, machine->ram_size / 32 * 2);
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
    s->pci_bus = pci_register_root_bus(host, "pci", lynx_pci_set_irq,
                                      lynx_pci_map_irq, s, &s->isa_mem,
                                      &s->isa_io, PCI_DEVFN(6, 0), 64,
                                      TYPE_PCI_BUS);
    PCI_HOST_BRIDGE(host)->bus = s->pci_bus;
    memory_region_init_iommu(&s->dma_iommu, sizeof(s->dma_iommu),
                             TYPE_LYNX_IOMMU, OBJECT(machine), "lynx.dma", 4 * GiB);
    address_space_init(&s->dma_as, MEMORY_REGION(&s->dma_iommu), "lynx.pci-dma");
    pci_setup_iommu(s->pci_bus, &lynx_dma_ops, s);
    /* Realize the host with its child bus already present. PCI's bus-realize
     * callback installs the machine-done hook which creates DMA aliases.
     */
    sysbus_realize_and_unref(SYS_BUS_DEVICE(host), &error_fatal);
    /* Real Lynx has a DECchip 21050 at primary IDSEL3. Firmware assigns
     * the NCR interrupt according to this topology; placing it at root
     * IDSEL4 incorrectly gets the built-in VGA vector (0x98).
     */
    bridge = pci_new(PCI_DEVFN(3, 0), "alpha2100a-dec21050");
    pci_bridge_map_irq(PCI_BRIDGE(bridge), "pci.1", lynx_secondary_map_irq);
    pci_realize_and_unref(bridge, s->pci_bus, &error_fatal);
    pci_bus_irqs(&PCI_BRIDGE(bridge)->sec_bus, lynx_pci_set_irq, s, 64);
    isa = isa_bus_new(NULL, &s->isa_mem, &s->isa_io, &error_fatal);
    s->pic_irqs = i8259_init(isa, NULL);
    irqs = qemu_allocate_irqs(lynx_isa_irq, s, 16);
    isa_bus_register_input_irqs(isa, irqs);
    isa_create_simple(isa, "i8042");
    /* PIT has no default I/O base; the ISA helper assigns 0x40 and IRQ0. */
    s->pit = i8254_pit_init(isa, 0x40, 0, NULL);
    lynx_isa_dma_init(s, isa);
    fdc = isa_new(TYPE_ISA_FDC);
    qdev_prop_set_bit(DEVICE(fdc), "strict-pio", true);
    isa_realize_and_unref(fdc, isa, &error_fatal);
    for (unsigned i = 0; i < MAX_FD; i++) {
        floppies[i] = drive_get(IF_FLOPPY, 0, i);
    }
    isa_fdc_init_drives(fdc, floppies);
    rtc = isa_new(TYPE_MC146818_RTC);
    qdev_prop_set_int32(DEVICE(rtc), "base_year",
                        s->firmware_is_arc ? 1980 : 1900);
    isa_realize_and_unref(rtc, isa, &error_fatal);
    s->rtc = MC146818_RTC(rtc);
    s->interval_timer = timer_new_ns(rtc_clock, lynx_interval_tick, s);
    lynx_profile_init(s);
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

}

static void lynx_finalize(Object *obj)
{
    Alpha2100AMachine *s = ALPHA2100A_MACHINE(obj);
    g_free(s->mem_edc);
    g_free(s->ram_edc);
    for (unsigned i = 0; i < 4; i++) {
        g_free(s->bcache[i]);
    }
    g_free(s->firmware);
    g_free(s->s3_romfile);
    if (s->nvram_file) {
        fclose(s->nvram_file);
    }
    g_free(s->nvram_filename);
    timer_free(s->interval_timer);
    if (s->mem_as.root) {
        address_space_destroy(&s->mem_as);
    }
    if (s->dma_as.root) {
        address_space_destroy(&s->dma_as);
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

static char *lynx_get_nvram(Object *obj, Error **errp)
{
    return g_strdup(ALPHA2100A_MACHINE(obj)->nvram_filename ?: "");
}

static void lynx_set_nvram(Object *obj, const char *value, Error **errp)
{
    Alpha2100AMachine *s = ALPHA2100A_MACHINE(obj);

    if (s->rtc) {
        error_setg(errp, "nvram-file must be set before machine initialization");
        return;
    }
    g_free(s->nvram_filename);
    s->nvram_filename = *value ? g_strdup(value) : NULL;
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
    object_class_property_add_str(oc, "nvram-file", lynx_get_nvram, lynx_set_nvram);
    object_class_property_set_description(oc, "nvram-file",
        "8306-byte configuration EEPROM/CMOS profile; clock follows -rtc");
    mc->no_floppy = false;
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

static void lynx_bridge_class_init(ObjectClass *oc, const void *data)
{
    /* 21050-compatible bridge configuration/windows/reset, using QEMU's
     * existing DEC bridge. Chip-specific arbitration/timing is not modeled.
     */
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->device_id = 0x0001;
    DEVICE_CLASS(oc)->desc = "Alpha2100A DECchip 21050-compatible PCI bridge";
}

static const TypeInfo lynx_bridge_type = {
    .name = "alpha2100a-dec21050",
    .parent = TYPE_DEC_PCI_BRIDGE,
    .class_init = lynx_bridge_class_init,
};

static const TypeInfo lynx_host_type = {
    .name = "alpha2100a-pci-host",
    .parent = TYPE_PCI_HOST_BRIDGE,
};

static void lynx_register(void)
{
    type_register_static(&lynx_iommu_type);
    type_register_static(&lynx_dma_type);
    type_register_static(&lynx_cirrus_type);
    type_register_static(&lynx_host_type);
    type_register_static(&lynx_bridge_type);
    type_register_static(&lynx_type);
}
type_init(lynx_register)
