/*
 * Experimental OMAP4 dual-mode timers and clock controls for Windows ARM.
 * Register layout reference: include/clocksource/timer-ti-dm.h in Linux.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_OMAP_DMTIMER "omap4-dmtimer"
OBJECT_DECLARE_SIMPLE_TYPE(OmapDmtimerState, OMAP_DMTIMER)

#define OMAP_ST       BIT(0)
#define OMAP_AR       BIT(1)
#define OMAP_PRE      BIT(5)
#define OMAP_CE       BIT(6)
#define OMAP_MATCH    BIT(0)
#define OMAP_OVERFLOW BIT(1)
#define OMAP_SYS_HZ   38400000

typedef struct OmapDmtimerChannel {
    MemoryRegion mmio;
    QEMUTimer *timer;
    qemu_irq irq;
    unsigned index;
    uint32_t control, count, load, match;
    uint32_t status, irqenable, wakeup, sysconfig, tsicr;
    uint32_t extra[5];
    uint32_t frequency;
    uint32_t event;
    int64_t epoch;
} OmapDmtimerChannel;

typedef struct OmapDmtimerClock {
    MemoryRegion mmio;
    OmapDmtimerState *parent;
    unsigned index;
} OmapDmtimerClock;

struct OmapDmtimerState {
    SysBusDevice parent_obj;
    OmapDmtimerChannel channel[4];
    OmapDmtimerClock clock[2];
    uint32_t clock_control[4];
};

static uint32_t omap_rate(OmapDmtimerChannel *c)
{
    return c->frequency >> ((c->control & OMAP_PRE) ?
                            ((c->control >> 2) & 7) + 1 : 0);
}

static uint64_t omap_ticks(OmapDmtimerChannel *c)
{
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - c->epoch;

    return (c->control & OMAP_ST) && elapsed > 0 ?
           muldiv64(elapsed, omap_rate(c), NANOSECONDS_PER_SECOND) : 0;
}

static uint32_t omap_count_at(OmapDmtimerChannel *c, uint64_t ticks)
{
    uint64_t first = (1ULL << 32) - c->count;

    if (ticks < first) {
        return c->count + ticks;
    }
    if (!(c->control & OMAP_AR)) {
        return 0;
    }
    return c->load + (ticks - first) % ((1ULL << 32) - c->load);
}

static uint32_t omap_count(OmapDmtimerChannel *c)
{
    return omap_count_at(c, omap_ticks(c));
}

static void omap_irq(OmapDmtimerChannel *c)
{
    qemu_set_irq(c->irq, !!(c->status & c->irqenable));
}

static void omap_anchor(OmapDmtimerChannel *c)
{
    c->count = omap_count(c);
    c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void omap_rearm(OmapDmtimerChannel *c, bool match_now)
{
    uint32_t count;
    uint64_t ticks, delta, match_delta;

    timer_del(c->timer);
    if (!(c->control & OMAP_ST)) {
        return;
    }
    ticks = omap_ticks(c);
    count = omap_count_at(c, ticks);
    /* The comparator also observes a match programmed in the current tick. */
    if (match_now && (c->control & OMAP_CE) && c->match == count) {
        c->status |= OMAP_MATCH;
    }
    if (match_now) {
        trace_omap_dmtimer_compare(c->index, count, c->match, c->status);
    }
    delta = (1ULL << 32) - count;
    c->event = OMAP_OVERFLOW;
    if ((c->control & OMAP_CE) && c->match > count) {
        match_delta = (uint64_t)c->match - count;
        if (match_delta < delta) {
            delta = match_delta;
            c->event = OMAP_MATCH;
        }
    }
    timer_mod(c->timer, c->epoch + muldiv64_round_up(ticks + delta,
                      NANOSECONDS_PER_SECOND, omap_rate(c)));
}

static void omap_expire(void *opaque)
{
    OmapDmtimerChannel *c = opaque;

    c->status |= c->event;
    if (c->event & OMAP_OVERFLOW) {
        if (!(c->control & OMAP_AR)) {
            c->count = 0;
            c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            c->control &= ~OMAP_ST;
        } else if ((c->control & OMAP_CE) && c->match == c->load) {
            c->status |= OMAP_MATCH;
        }
    }
    trace_omap_dmtimer_expire(c->index, c->status, c->irqenable);
    omap_irq(c);
    omap_rearm(c, false);
}

static void omap_channel_reset(OmapDmtimerChannel *c)
{
    timer_del(c->timer);
    c->control = c->count = c->load = c->match = 0;
    c->status = c->irqenable = c->wakeup = c->sysconfig = c->tsicr = 0;
    memset(c->extra, 0, sizeof(c->extra));
    c->event = 0;
    c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    omap_irq(c);
}

/* Normalize the OMAP4 v2 interrupt and functional register layout to v1. */
static hwaddr omap_offset(OmapDmtimerChannel *c, hwaddr offset)
{
    if (c->index < 2 || offset < 0x20) {
        return offset;
    }
    switch (offset) {
    case 0x20:
        return 0x108; /* IRQ_EOI */
    case 0x24:
        return 0x100; /* IRQSTATUS_RAW */
    case 0x28:
        return 0x18;
    case 0x2c:
        return 0x1c;
    case 0x30:
        return 0x104; /* IRQENABLE_CLR */
    default:
        return offset - 0x14;
    }
}

static uint64_t omap_read(void *opaque, hwaddr offset, unsigned size)
{
    OmapDmtimerChannel *c = opaque;

    switch (omap_offset(c, offset)) {
    case 0:
        return c->index < 2 ? 0x21 : 0x4fff1301;
    case 0x10:
        return c->sysconfig;
    case 0x14:
        return 1;
    case 0x18:
        return c->index < 2 ? c->status : c->status & c->irqenable;
    case 0x100:
        return c->status;
    case 0x108:
        return 0;
    case 0x1c:
    case 0x104:
        return c->irqenable;
    case 0x20:
        return c->wakeup;
    case 0x24:
        return c->control;
    case 0x28:
        return omap_count(c);
    case 0x2c:
        return c->load;
    case 0x30:
    case 0x34:
        return 0; /* Trigger / immediate posted-write completion */
    case 0x38:
        return c->match;
    case 0x3c:
    case 0x44:
        return 0; /* Capture inputs are not connected. */
    case 0x40:
        return c->tsicr;
    /* The 1 ms compensation registers are retained but not applied. */
    case 0x48:
    case 0x4c:
    case 0x50:
    case 0x54:
    case 0x58:
        return c->extra[(omap_offset(c, offset) - 0x48) / 4];
    }
    qemu_log_mask(LOG_UNIMP, "omap-dmtimer: channel %u read at 0x%"
                  HWADDR_PRIx "\n", c->index, offset);
    return 0;
}

static void omap_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    OmapDmtimerChannel *c = opaque;
    hwaddr reg = omap_offset(c, offset);

    trace_omap_dmtimer_write(c->index, offset, value);
    if (reg == 0x24 || reg == 0x28 || reg == 0x2c || reg == 0x30) {
        omap_anchor(c);
    }
    switch (reg) {
    case 0x10:
        if (value & (c->index < 2 ? 2 : 1)) {
            omap_channel_reset(c);
        } else {
            c->sysconfig = value;
        }
        break;
    case 0x18:
        c->status &= ~(value & 7);
        break;
    case 0x100:
        c->status |= value & 7;
        break;
    case 0x1c:
        if (c->index < 2) {
            c->irqenable = value & 7;
        } else {
            c->irqenable |= value & 7;
        }
        break;
    case 0x104:
        c->irqenable &= ~(value & 7);
        break;
    case 0x108:
        break;
    case 0x20:
        c->wakeup = value & 7;
        break;
    case 0x24:
        c->control = value & 0x7fff;
        break;
    case 0x28:
        c->count = value;
        break;
    case 0x2c:
        c->load = value;
        break;
    case 0x30:
        c->count = c->load;
        break;
    case 0x38:
        c->match = value;
        break;
    case 0x40:
        c->tsicr = value;
        break;
    case 0x48:
    case 0x4c:
    case 0x50:
    case 0x54:
    case 0x58:
        c->extra[(omap_offset(c, offset) - 0x48) / 4] = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "omap-dmtimer: channel %u write at 0x%"
                      HWADDR_PRIx "\n", c->index, offset);
    }
    omap_rearm(c, reg == 0x24 || reg == 0x28 || reg == 0x30 || reg == 0x38);
    omap_irq(c);
}

static int omap_clock_channel(OmapDmtimerClock *clock, hwaddr offset)
{
    if (clock->index == 0) {
        return offset == 0x840 ? 0 : -1;
    }
    return offset >= 0x438 && offset <= 0x448 && !(offset & 7) ?
           1 + (offset - 0x438) / 8 : -1;
}

static uint64_t omap_clock_read(void *opaque, hwaddr offset, unsigned size)
{
    OmapDmtimerClock *clock = opaque;
    int i = omap_clock_channel(clock, offset);

    return i >= 0 ? clock->parent->clock_control[i] & ~0x30000 : 0;
}

static void omap_clock_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    OmapDmtimerClock *clock = opaque;
    OmapDmtimerState *s = clock->parent;
    int i = omap_clock_channel(clock, offset);

    trace_omap_dmtimer_clock(clock->index, offset, value);
    if (i >= 0) {
        OmapDmtimerChannel *c = &s->channel[i];

        omap_anchor(c);
        s->clock_control[i] = value;
        c->frequency = value & BIT(24) ? 32768 : OMAP_SYS_HZ;
        omap_rearm(c, false);
    } else {
        qemu_log_mask(LOG_UNIMP, "omap-dmtimer: clock %u write at 0x%"
                      HWADDR_PRIx "\n", clock->index, offset);
    }
}

static const MemoryRegionOps omap_ops = {
    .read = omap_read, .write = omap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps omap_clock_ops = {
    .read = omap_clock_read, .write = omap_clock_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void omap_reset(DeviceState *dev)
{
    OmapDmtimerState *s = OMAP_DMTIMER(dev);

    memset(s->clock_control, 0, sizeof(s->clock_control));
    for (unsigned i = 0; i < 4; i++) {
        s->channel[i].frequency = OMAP_SYS_HZ;
        omap_channel_reset(&s->channel[i]);
    }
}

static int omap_post_load(void *opaque, int version)
{
    OmapDmtimerState *s = opaque;

    for (unsigned i = 0; i < 4; i++) {
        if (s->channel[i].frequency != 32768 &&
            s->channel[i].frequency != OMAP_SYS_HZ) {
            return -EINVAL;
        }
    }
    for (unsigned i = 0; i < 4; i++) {
        omap_irq(&s->channel[i]);
        omap_rearm(&s->channel[i], false);
    }
    return 0;
}

static const VMStateDescription vmstate_omap_channel = {
    .name = "omap4-dmtimer/channel", .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(control, OmapDmtimerChannel),
        VMSTATE_UINT32(count, OmapDmtimerChannel),
        VMSTATE_UINT32(load, OmapDmtimerChannel),
        VMSTATE_UINT32(match, OmapDmtimerChannel),
        VMSTATE_UINT32(status, OmapDmtimerChannel),
        VMSTATE_UINT32(irqenable, OmapDmtimerChannel),
        VMSTATE_UINT32(wakeup, OmapDmtimerChannel),
        VMSTATE_UINT32(sysconfig, OmapDmtimerChannel),
        VMSTATE_UINT32(tsicr, OmapDmtimerChannel),
        VMSTATE_UINT32_ARRAY(extra, OmapDmtimerChannel, 5),
        VMSTATE_UINT32(frequency, OmapDmtimerChannel),
        VMSTATE_UINT32(event, OmapDmtimerChannel),
        VMSTATE_INT64(epoch, OmapDmtimerChannel),
        VMSTATE_TIMER_PTR(timer, OmapDmtimerChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_omap = {
    .name = TYPE_OMAP_DMTIMER, .version_id = 1, .minimum_version_id = 1,
    .post_load = omap_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(channel, OmapDmtimerState, 4, 1,
                              vmstate_omap_channel, OmapDmtimerChannel),
        VMSTATE_UINT32_ARRAY(clock_control, OmapDmtimerState, 4),
        VMSTATE_END_OF_LIST()
    }
};

static void omap_init(Object *obj)
{
    OmapDmtimerState *s = OMAP_DMTIMER(obj);
    SysBusDevice *busdev = SYS_BUS_DEVICE(obj);

    for (unsigned i = 0; i < 4; i++) {
        OmapDmtimerChannel *c = &s->channel[i];

        c->index = i;
        c->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, omap_expire, c);
        memory_region_init_io(&c->mmio, obj, &omap_ops, c,
                               "omap4-dmtimer", 0x1000);
        sysbus_init_mmio(busdev, &c->mmio);
        sysbus_init_irq(busdev, &c->irq);
    }
    for (unsigned i = 0; i < 2; i++) {
        s->clock[i].parent = s;
        s->clock[i].index = i;
        memory_region_init_io(&s->clock[i].mmio, obj, &omap_clock_ops,
                               &s->clock[i], "omap4-timer-clock", 0x1000);
        sysbus_init_mmio(busdev, &s->clock[i].mmio);
    }
}

static void omap_finalize(Object *obj)
{
    OmapDmtimerState *s = OMAP_DMTIMER(obj);

    for (unsigned i = 0; i < 4; i++) {
        timer_free(s->channel[i].timer);
    }
}

static void omap_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_omap;
    device_class_set_legacy_reset(dc, omap_reset);
}

static const TypeInfo omap_info = {
    .name = TYPE_OMAP_DMTIMER, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OmapDmtimerState),
    .instance_init = omap_init, .instance_finalize = omap_finalize,
    .class_init = omap_class_init,
};

static void omap_register_types(void)
{
    type_register_static(&omap_info);
}
type_init(omap_register_types)
