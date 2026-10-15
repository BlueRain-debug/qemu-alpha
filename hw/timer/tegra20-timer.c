/*
 * Tegra20 timer and RTC countdown interfaces used by Windows RT.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This is the opt-in virt compatibility subset, not a Tegra SoC model.
 * Register semantics are documented by the Linux Tegra clocksource and RTC
 * drivers. Timers run on QEMU virtual time, including the 1 MHz counter.
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "system/rtc.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_TEGRA20_TIMER "tegra20-timer"
OBJECT_DECLARE_SIMPLE_TYPE(Tegra20TimerState, TEGRA20_TIMER)

#define TEGRA_ENABLE BIT(31)
#define TEGRA_PERIODIC BIT(30)
#define TEGRA_COUNT_MASK 0x1fffffff
#define TEGRA_RTC_COUNT_MASK 0x3fffffff
#define TEGRA_CHANNELS 6

/* Channels 0..3 are TIMER1..4, 4..5 are RTC seconds/milliseconds countdown. */
typedef struct TegraTimerChannel {
    Tegra20TimerState *owner;
    QEMUTimer *timer;
    uint32_t control;
    bool pending;
    int64_t deadline;
    unsigned index;
} TegraTimerChannel;

struct Tegra20TimerState {
    SysBusDevice parent_obj;
    MemoryRegion timer_mmio;
    MemoryRegion rtc_mmio;
    qemu_irq irq[5];
    TegraTimerChannel channel[TEGRA_CHANNELS];
    int64_t counter_epoch;
    int64_t rtc_offset_ms;
    uint32_t usec_config;
    uint32_t freeze;
    uint32_t rtc_mask;
    uint32_t rtc_status;
    uint32_t rtc_shadow;
    uint32_t rtc_alarm[3];
};

static const unsigned timer_offsets[4] = { 0x00, 0x08, 0x50, 0x58 };

static void tegra_update_irq(Tegra20TimerState *s)
{
    for (unsigned i = 0; i < 4; i++) {
        qemu_set_irq(s->irq[i], s->channel[i].pending);
    }
    qemu_set_irq(s->irq[4], !!(s->rtc_status & s->rtc_mask));
}

static int64_t tegra_period(TegraTimerChannel *c)
{
    if (c->index < 4) {
        /* Tegra GPT expires after n + 1 input ticks, including load zero. */
        return ((int64_t)(c->control & TEGRA_COUNT_MASK) + 1) * 1000;
    }
    return MAX(c->control & TEGRA_RTC_COUNT_MASK, 1) *
           (c->index == 4 ? 1000000000LL : 1000000LL);
}

static void tegra_tick(void *opaque)
{
    TegraTimerChannel *c = opaque;
    Tegra20TimerState *s = c->owner;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    c->pending = true;
    if (c->index >= 4) {
        s->rtc_status |= BIT(c->index - 1);
    }
    trace_tegra20_timer_expire(c->index, c->control);
    if (c->control & TEGRA_PERIODIC) {
        int64_t period = tegra_period(c);
        c->deadline += (MAX(now - c->deadline, 0) / period + 1) * period;
        timer_mod(c->timer, c->deadline);
    } else {
        c->control &= ~TEGRA_ENABLE;
    }
    tegra_update_irq(s);
}

static void tegra_program(TegraTimerChannel *c, uint32_t value)
{
    c->control = value;
    timer_del(c->timer);
    if (value & TEGRA_ENABLE) {
        c->deadline = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + tegra_period(c);
        timer_mod(c->timer, c->deadline);
    }
    trace_tegra20_timer_program(c->index, value, c->deadline);
}

static uint64_t tegra_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    Tegra20TimerState *s = opaque;

    if (offset == 0x10) {
        return (uint32_t)((qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                           s->counter_epoch) / 1000);
    }
    if (offset == 0x14) {
        return s->usec_config;
    }
    if (offset == 0x4c) {
        return s->freeze;
    }
    for (unsigned i = 0; i < 4; i++) {
        TegraTimerChannel *c = &s->channel[i];
        if (offset == timer_offsets[i]) {
            return c->control;
        }
        if (offset == timer_offsets[i] + 4) {
            int64_t ns = c->deadline - qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            uint32_t count = timer_pending(c->timer) && ns > 0 ?
                             (ns - 1) / 1000 : 0;
            return (count & TEGRA_COUNT_MASK) |
                   (c->pending ? TEGRA_PERIODIC : 0);
        }
    }
    qemu_log_mask(LOG_UNIMP, "%s: unimplemented offset 0x%" HWADDR_PRIx "\n",
                  __func__, offset);
    return 0;
}

static void tegra_timer_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    Tegra20TimerState *s = opaque;

    if (offset == 0x14) {
        /* This compatibility device supplies the firmware's 1 MHz clock. */
        s->usec_config = value;
        return;
    }
    if (offset == 0x4c) {
        s->freeze = value;
        return;
    }
    for (unsigned i = 0; i < 4; i++) {
        if (offset == timer_offsets[i]) {
            tegra_program(&s->channel[i], value &
                          (TEGRA_ENABLE | TEGRA_PERIODIC | TEGRA_COUNT_MASK));
            return;
        }
        if (offset == timer_offsets[i] + 4) {
            if (value & TEGRA_PERIODIC) {
                s->channel[i].pending = false;
                tegra_update_irq(s);
            }
            return;
        }
    }
    qemu_log_mask(LOG_UNIMP, "%s: unimplemented offset 0x%" HWADDR_PRIx "\n",
                  __func__, offset);
}

static uint64_t tegra_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    Tegra20TimerState *s = opaque;
    int64_t ms = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + s->rtc_offset_ms;

    switch (offset) {
    case 0x00: /* Control */
    case 0x04: /* BUSY: writes complete synchronously. */
        return 0;
    case 0x08:
        return ms / 1000;
    case 0x0c:
        return s->rtc_shadow;
    case 0x10:
        s->rtc_shadow = ms / 1000;
        return ms % 1000;
    case 0x14: case 0x18: case 0x1c:
        return s->rtc_alarm[(offset - 0x14) / 4];
    case 0x20: case 0x24:
        return s->channel[4 + (offset - 0x20) / 4].control;
    case 0x28:
        return s->rtc_mask;
    case 0x2c:
        return s->rtc_status;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented offset 0x%" HWADDR_PRIx
                      "\n", __func__, offset);
        return 0;
    }
}

static void tegra_rtc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    Tegra20TimerState *s = opaque;

    switch (offset) {
    case 0x00:
        break;
    case 0x08:
        s->rtc_offset_ms = (int64_t)(uint32_t)value * 1000 -
                           qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
        break;
    case 0x14: case 0x18: case 0x1c:
        /* Absolute alarms are not used by the WinRT HAL compatibility path. */
        s->rtc_alarm[(offset - 0x14) / 4] = value;
        break;
    case 0x20: case 0x24:
        tegra_program(&s->channel[4 + (offset - 0x20) / 4], value);
        break;
    case 0x28:
        s->rtc_mask = value & 0x1f;
        tegra_update_irq(s);
        break;
    case 0x2c:
        s->rtc_status &= ~value;
        tegra_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented offset 0x%" HWADDR_PRIx
                      "\n", __func__, offset);
    }
}

static const MemoryRegionOps tegra_timer_ops = {
    .read = tegra_timer_read,
    .write = tegra_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const MemoryRegionOps tegra_rtc_ops = {
    .read = tegra_rtc_read,
    .write = tegra_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void tegra_reset(DeviceState *dev)
{
    Tegra20TimerState *s = TEGRA20_TIMER(dev);
    struct tm tm;

    for (unsigned i = 0; i < TEGRA_CHANNELS; i++) {
        timer_del(s->channel[i].timer);
        s->channel[i].control = 0;
        s->channel[i].pending = false;
        s->channel[i].deadline = 0;
    }
    s->counter_epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    qemu_get_timedate(&tm, 0);
    s->rtc_offset_ms = (int64_t)mktimegm(&tm) * 1000 -
                       qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
    s->usec_config = 0x0b;
    s->freeze = s->rtc_mask = s->rtc_status = s->rtc_shadow = 0;
    memset(s->rtc_alarm, 0, sizeof(s->rtc_alarm));
    tegra_update_irq(s);
}

static int tegra_post_load(void *opaque, int version_id)
{
    tegra_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_tegra_channel = {
    .name = "tegra20-timer/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(control, TegraTimerChannel),
        VMSTATE_BOOL(pending, TegraTimerChannel),
        VMSTATE_INT64(deadline, TegraTimerChannel),
        VMSTATE_TIMER_PTR(timer, TegraTimerChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_tegra = {
    .name = "tegra20-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = tegra_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(channel, Tegra20TimerState, TEGRA_CHANNELS,
                             1, vmstate_tegra_channel, TegraTimerChannel),
        VMSTATE_INT64(counter_epoch, Tegra20TimerState),
        VMSTATE_INT64(rtc_offset_ms, Tegra20TimerState),
        VMSTATE_UINT32(usec_config, Tegra20TimerState),
        VMSTATE_UINT32(freeze, Tegra20TimerState),
        VMSTATE_UINT32(rtc_mask, Tegra20TimerState),
        VMSTATE_UINT32(rtc_status, Tegra20TimerState),
        VMSTATE_UINT32(rtc_shadow, Tegra20TimerState),
        VMSTATE_UINT32_ARRAY(rtc_alarm, Tegra20TimerState, 3),
        VMSTATE_END_OF_LIST()
    }
};

static void tegra_init(Object *obj)
{
    Tegra20TimerState *s = TEGRA20_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    for (unsigned i = 0; i < 5; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    for (unsigned i = 0; i < TEGRA_CHANNELS; i++) {
        s->channel[i].owner = s;
        s->channel[i].index = i;
        s->channel[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          tegra_tick, &s->channel[i]);
    }
    memory_region_init_io(&s->timer_mmio, obj, &tegra_timer_ops, s,
                          "tegra20.timer", 0x1000);
    memory_region_init_io(&s->rtc_mmio, obj, &tegra_rtc_ops, s,
                          "tegra20.rtc", 0x1000);
    sysbus_init_mmio(sbd, &s->timer_mmio);
    sysbus_init_mmio(sbd, &s->rtc_mmio);
}

static void tegra_finalize(Object *obj)
{
    Tegra20TimerState *s = TEGRA20_TIMER(obj);
    for (unsigned i = 0; i < TEGRA_CHANNELS; i++) {
        timer_free(s->channel[i].timer);
    }
}

static void tegra_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, tegra_reset);
    dc->vmsd = &vmstate_tegra;
    dc->desc = "Tegra20 timer and RTC countdown compatibility interfaces";
}

static const TypeInfo tegra_info = {
    .name = TYPE_TEGRA20_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Tegra20TimerState),
    .instance_init = tegra_init,
    .instance_finalize = tegra_finalize,
    .class_init = tegra_class_init,
};

static void tegra_register_types(void)
{
    type_register_static(&tegra_info);
}

type_init(tegra_register_types)
