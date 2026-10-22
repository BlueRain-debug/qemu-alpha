/*
 * Experimental OMAP4 DMA4 controller for the Windows ARM HAL.
 * Register layout: TI OMAP4 TRM and Linux arch/arm/mach-omap2/dma.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/dma.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_OMAP4_DMA "omap4-dma"
OBJECT_DECLARE_SIMPLE_TYPE(Omap4DmaState, OMAP4_DMA)

#define DMA_ENABLE BIT(7)
#define DMA_BLOCK  BIT(5)
#define DMA_ERROR  BIT(8)
#define DMA_CHANNELS 32
#define DMA_CH_BASE 0x80
#define DMA_CH_SIZE 0x60

enum {
    CCR, CLNK, CICR, CSR, CSDP, CEN, CFN, CSSA, CDSA,
    CSEI, CSFI, CDEI, CDFI, CSAC, CDAC, CCEN, CCFN, COLOR,
};

typedef struct Omap4DmaChannel {
    Omap4DmaState *parent;
    QEMUTimer *timer;
    unsigned index;
    uint32_t reg[DMA_CH_SIZE / 4];
} Omap4DmaChannel;

struct Omap4DmaState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq[4];
    uint32_t irqstatus[4], irqenable[4];
    uint32_t sysconfig, gcr;
    Omap4DmaChannel channel[DMA_CHANNELS];
};

static void omap4_dma_irq(Omap4DmaState *s)
{
    for (unsigned i = 0; i < 4; i++) {
        qemu_set_irq(s->irq[i], !!(s->irqstatus[i] & s->irqenable[i]));
    }
}

static void omap4_dma_finish(Omap4DmaChannel *c, uint32_t status)
{
    Omap4DmaState *s = c->parent;

    c->reg[CCR] &= ~DMA_ENABLE;
    c->reg[CSR] |= status;
    if (c->reg[CSR] & c->reg[CICR]) {
        for (unsigned i = 0; i < 4; i++) {
            s->irqstatus[i] |= BIT(c->index);
        }
    }
    trace_omap4_dma_complete(c->index, c->reg[CSR]);
    omap4_dma_irq(s);
}

static int32_t omap4_dma_step(Omap4DmaChannel *c, bool dest, unsigned width)
{
    unsigned mode = (c->reg[CCR] >> (dest ? 14 : 12)) & 3;

    switch (mode) {
    case 0:
        return 0;
    case 1:
        return width;
    default:
        return width + (int16_t)c->reg[dest ? CDEI : CSEI] - 1;
    }
}

static void omap4_dma_run(void *opaque)
{
    Omap4DmaChannel *c = opaque;
    uint32_t *r = c->reg;
    unsigned width = 1 << (r[CSDP] & 3);

    if (!(r[CCR] & DMA_ENABLE)) {
        return;
    }
    /*
     * Peripheral requests, linked descriptors and endian conversion are absent.
     * Report an error instead of completing an unsupported transfer.
     */
    if ((r[CCR] & (0x1f | (3 << 19) | BIT(17))) ||
        (r[CLNK] & BIT(15)) || (r[CSDP] & (BIT(19) | BIT(21))) ||
        width > 4 || !r[CEN] || !r[CFN]) {
        qemu_log_mask(LOG_UNIMP, "omap4-dma: unsupported channel %u setup\n",
                      c->index);
        omap4_dma_finish(c, DMA_ERROR);
        return;
    }

    /* Limit work per callback, including large transfers. */
    for (unsigned budget = 0; budget < 1024; budget++) {
        uint8_t data[4];
        MemTxResult result = MEMTX_OK;

        if (r[CCR] & BIT(16)) {
            stl_le_p(data, r[COLOR]);
        } else {
            result = dma_memory_read(&address_space_memory, r[CSAC], data,
                                     width, MEMTXATTRS_UNSPECIFIED);
        }
        if (result == MEMTX_OK) {
            result = dma_memory_write(&address_space_memory, r[CDAC], data,
                                      width, MEMTXATTRS_UNSPECIFIED);
        }
        if (result != MEMTX_OK) {
            omap4_dma_finish(c, DMA_ERROR);
            return;
        }
        r[CSAC] += omap4_dma_step(c, false, width);
        r[CDAC] += omap4_dma_step(c, true, width);
        if (++r[CCEN] == r[CEN]) {
            r[CCEN] = 0;
            if (((r[CCR] >> 12) & 3) == 3) {
                r[CSAC] += r[CSFI] - (int16_t)r[CSEI];
            }
            if (((r[CCR] >> 14) & 3) == 3) {
                r[CDAC] += r[CDFI] - (int16_t)r[CDEI];
            }
            if (++r[CCFN] == r[CFN]) {
                omap4_dma_finish(c, DMA_BLOCK);
                return;
            }
        }
    }
    timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
}

static void omap4_dma_reset(DeviceState *dev)
{
    Omap4DmaState *s = OMAP4_DMA(dev);

    memset(s->irqstatus, 0, sizeof(s->irqstatus));
    memset(s->irqenable, 0, sizeof(s->irqenable));
    s->sysconfig = 0;
    s->gcr = 0;
    for (unsigned i = 0; i < DMA_CHANNELS; i++) {
        timer_del(s->channel[i].timer);
        memset(s->channel[i].reg, 0, sizeof(s->channel[i].reg));
    }
    omap4_dma_irq(s);
}

static uint64_t omap4_dma_read(void *opaque, hwaddr offset, unsigned size)
{
    Omap4DmaState *s = opaque;
    uint32_t value = 0;

    if (offset >= DMA_CH_BASE &&
        offset < DMA_CH_BASE + DMA_CHANNELS * DMA_CH_SIZE) {
        unsigned channel = (offset - DMA_CH_BASE) / DMA_CH_SIZE;
        unsigned reg = ((offset - DMA_CH_BASE) % DMA_CH_SIZE) / 4;

        value = s->channel[channel].reg[reg];
    } else if (offset >= 8 && offset < 0x18) {
        value = s->irqstatus[(offset - 8) / 4];
    } else if (offset >= 0x18 && offset < 0x28) {
        value = s->irqenable[(offset - 0x18) / 4];
    } else {
        switch (offset) {
        case 0:
            value = 0x40;
            break;
        case 0x28:
            value = 1; /* RESETDONE */
            break;
        case 0x2c:
            value = s->sysconfig;
            break;
        case 0x78:
            value = s->gcr;
            break;
        }
    }
    trace_omap4_dma_read(offset, value);
    return value;
}

static void omap4_dma_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    Omap4DmaState *s = opaque;

    trace_omap4_dma_write(offset, value);
    if (offset >= DMA_CH_BASE &&
        offset < DMA_CH_BASE + DMA_CHANNELS * DMA_CH_SIZE) {
        unsigned channel = (offset - DMA_CH_BASE) / DMA_CH_SIZE;
        unsigned reg = ((offset - DMA_CH_BASE) % DMA_CH_SIZE) / 4;
        Omap4DmaChannel *c = &s->channel[channel];

        if (reg == CSR) {
            c->reg[CSR] &= ~value;
        } else if (reg < CSAC || reg > CCFN) {
            uint32_t old = c->reg[reg];

            c->reg[reg] = value;
            if (reg == CEN) {
                c->reg[reg] &= 0xffffff;
            } else if (reg == CFN) {
                c->reg[reg] &= 0xffff;
            } else if (reg == CCR) {
                if (!(value & DMA_ENABLE)) {
                    timer_del(c->timer);
                } else if (!(old & DMA_ENABLE)) {
                    c->reg[CSAC] = c->reg[CSSA];
                    c->reg[CDAC] = c->reg[CDSA];
                    c->reg[CCEN] = c->reg[CCFN] = 0;
                    timer_mod(c->timer,
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
                }
            }
        }
    } else if (offset >= 8 && offset < 0x18) {
        s->irqstatus[(offset - 8) / 4] &= ~value;
    } else if (offset >= 0x18 && offset < 0x28) {
        s->irqenable[(offset - 0x18) / 4] = value;
    } else if (offset == 0x2c) {
        if (value & BIT(1)) {
            omap4_dma_reset(DEVICE(s));
        }
        s->sysconfig = value & ~BIT(1);
    } else if (offset == 0x78) {
        s->gcr = value;
    }
    omap4_dma_irq(s);
}

static const MemoryRegionOps omap4_dma_ops = {
    .read = omap4_dma_read,
    .write = omap4_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void omap4_dma_init(Object *obj)
{
    Omap4DmaState *s = OMAP4_DMA(obj);

    memory_region_init_io(&s->mmio, obj, &omap4_dma_ops, s,
                          TYPE_OMAP4_DMA, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);
    for (unsigned i = 0; i < 4; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[i]);
    }
    for (unsigned i = 0; i < DMA_CHANNELS; i++) {
        Omap4DmaChannel *c = &s->channel[i];

        c->parent = s;
        c->index = i;
        c->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, omap4_dma_run, c);
    }
}

static void omap4_dma_finalize(Object *obj)
{
    Omap4DmaState *s = OMAP4_DMA(obj);

    for (unsigned i = 0; i < DMA_CHANNELS; i++) {
        timer_free(s->channel[i].timer);
    }
}

static const VMStateDescription vmstate_omap4_dma = {
    .name = TYPE_OMAP4_DMA,
    .unmigratable = true,
};

static void omap4_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_omap4_dma;
    device_class_set_legacy_reset(dc, omap4_dma_reset);
}

static const TypeInfo omap4_dma_info = {
    .name = TYPE_OMAP4_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Omap4DmaState),
    .instance_init = omap4_dma_init,
    .instance_finalize = omap4_dma_finalize,
    .class_init = omap4_dma_class_init,
};

static void omap4_dma_register_types(void)
{
    type_register_static(&omap4_dma_info);
}

type_init(omap4_dma_register_types)
