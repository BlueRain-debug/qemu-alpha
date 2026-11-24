/*
 * SDM845 TSENS v2 frontend for local OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Two controllers report a constant 25 degrees C, with sensor enables,
 * signed thresholds, latched alarms, clear/mask registers and native IRQs.
 * This is a virtual temperature source, not a physical thermal model.
 * Register layout: Linux drivers/thermal/qcom/tsens-v2.c and sdm845.dtsi.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "trace.h"

#define TYPE_QCOM845_TSENS "qcom845-tsens"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Tsens, QCOM845_TSENS)
#define TSENS_SIZE 0x1000
#define TSENS_TEMP 250 /* Signed 12-bit, deci-Celsius. */
#define TSENS_PERIOD_NS (2 * SCALE_MS)

typedef struct Tsens845Region {
    Qcom845Tsens *owner;
    bool tm;
    MemoryRegion mmio;
} Tsens845Region;

struct Qcom845Tsens {
    SysBusDevice parent_obj;
    Tsens845Region regions[2]; /* SROT, TM */
    qemu_irq irq[2]; /* upper/lower, critical */
    QEMUTimer timer;
    unsigned sensors;
    unsigned index;
    uint32_t srot[TSENS_SIZE / 4];
    uint32_t tm[TSENS_SIZE / 4];
};

static unsigned tsens845_enabled(Qcom845Tsens *s)
{
    uint32_t ctrl = s->srot[1];

    return (ctrl & 3) == 1 ? (ctrl >> 3) & (BIT(s->sensors) - 1) : 0;
}

static void tsens845_irq(Qcom845Tsens *s)
{
    uint32_t alarms = s->tm[0x08 / 4] & ~s->tm[0x10 / 4];
    uint32_t enables = s->tm[0x04 / 4];

    qemu_set_irq(s->irq[0], ((enables & 1) && (alarms & 0xffff)) ||
                           ((enables & 2) && (alarms & 0xffff0000)));
    qemu_set_irq(s->irq[1], (enables & 4) &&
                 (s->tm[0x14 / 4] & ~s->tm[0x1c / 4] & 0xffff));
}

static int tsens845_signed(uint32_t value)
{
    return (int)((value & 0xfff) ^ 0x800) - 0x800;
}

static uint32_t tsens845_status(Qcom845Tsens *s, unsigned sensor)
{
    uint32_t thresholds = s->tm[0x20 / 4 + sensor];
    uint32_t result = TSENS_TEMP | BIT(21);

    if (!(tsens845_enabled(s) & BIT(sensor))) {
        return 0;
    }
    if (TSENS_TEMP <= tsens845_signed(thresholds)) {
        result |= BIT(17);
    }
    if (TSENS_TEMP >= tsens845_signed(thresholds >> 12)) {
        result |= BIT(18);
    }
    if (TSENS_TEMP >= tsens845_signed(s->tm[0x60 / 4 + sensor])) {
        result |= BIT(19);
    }
    return result;
}

static void tsens845_sample(void *opaque)
{
    Qcom845Tsens *s = opaque;

    for (unsigned i = 0; i < s->sensors; i++) {
        uint32_t status = tsens845_status(s, i);

        if (status & BIT(17)) {
            s->tm[0x08 / 4] |= BIT(i);
        }
        if (status & BIT(18)) {
            s->tm[0x08 / 4] |= BIT(i + 16);
        }
        if (status & BIT(19)) {
            s->tm[0x14 / 4] |= BIT(i);
        }
    }
    /* A held CLEAR bit keeps its corresponding latch reset. */
    s->tm[0x08 / 4] &= ~s->tm[0x0c / 4];
    s->tm[0x14 / 4] &= ~s->tm[0x18 / 4];
    tsens845_irq(s);
    if (tsens845_enabled(s)) {
        timer_mod_ns(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     TSENS_PERIOD_NS);
    } else {
        timer_del(&s->timer);
    }
}

static uint64_t tsens845_read(void *opaque, hwaddr offset, unsigned size)
{
    Tsens845Region *region = opaque;
    Qcom845Tsens *s = region->owner;
    uint32_t value;

    if (!region->tm) {
        value = s->srot[offset / 4];
    } else if (offset >= 0xa0 && offset < 0xa0 + 4 * s->sensors) {
        value = tsens845_status(s, (offset - 0xa0) / 4);
    } else if (offset == 0xe4) {
        value = tsens845_enabled(s) != 0;
    } else {
        value = s->tm[offset / 4];
    }
    trace_qcom845_tsens_access(s->index, region->tm, false, offset, value);
    return value;
}

static void tsens845_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    Tsens845Region *region = opaque;
    Qcom845Tsens *s = region->owner;

    trace_qcom845_tsens_access(s->index, region->tm, true, offset, value);
    if (!region->tm) {
        if (offset == 4 || offset == 8 ||
            (offset >= 0x60 && offset < 0x60 + 4 * s->sensors)) {
            s->srot[offset / 4] = value;
        }
        if (offset == 4 && (value & 2)) {
            s->tm[0x08 / 4] = 0;
            s->tm[0x14 / 4] = 0;
        }
    } else if (offset == 4 || offset == 0xc || offset == 0x10 ||
               offset == 0x18 || offset == 0x1c ||
               (offset >= 0x20 && offset < 0x20 + 4 * s->sensors) ||
               (offset >= 0x60 && offset < 0x60 + 4 * s->sensors)) {
        s->tm[offset / 4] = value;
    } else {
        return; /* Status, reserved and unimplemented watchdog registers. */
    }
    tsens845_sample(s);
}

static const MemoryRegionOps tsens845_ops = {
    .read = tsens845_read,
    .write = tsens845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void tsens845_reset(DeviceState *dev)
{
    Qcom845Tsens *s = QCOM845_TSENS(dev);

    memset(s->srot, 0, sizeof(s->srot));
    memset(s->tm, 0, sizeof(s->tm));
    s->srot[0] = 0x20000000; /* TSENS v2.0 frontend. */
    /* The virtual firmware handoff has already enabled temperature mode. */
    s->srot[1] = BIT(21) | ((BIT(s->sensors) - 1) << 3) | 1;
    s->srot[2] = 1;
    s->tm[0x10 / 4] = UINT32_MAX;
    s->tm[0x1c / 4] = UINT32_MAX;
    for (unsigned i = 0; i < s->sensors; i++) {
        s->tm[0x20 / 4 + i] = 0x7ff800;
        s->tm[0x60 / 4 + i] = 0x7ff;
    }
    tsens845_sample(s);
}

static void tsens845_init(Object *obj)
{
    Qcom845Tsens *s = QCOM845_TSENS(obj);

    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, tsens845_sample, s);
    for (unsigned i = 0; i < 2; i++) {
        s->regions[i].owner = s;
        s->regions[i].tm = i == 1;
        memory_region_init_io(&s->regions[i].mmio, obj, &tsens845_ops,
                              &s->regions[i], TYPE_QCOM845_TSENS, TSENS_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->regions[i].mmio);
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
}

static void tsens845_finalize(Object *obj)
{
    timer_del(&QCOM845_TSENS(obj)->timer);
}

static const VMStateDescription tsens845_vmstate = {
    .name = TYPE_QCOM845_TSENS,
    .unmigratable = true,
};

static void tsens845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, tsens845_reset);
    dc->desc = "Experimental SDM845 TSENS v2 virtual temperature source";
    dc->vmsd = &tsens845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo tsens845_type = {
    .name = TYPE_QCOM845_TSENS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Tsens),
    .instance_init = tsens845_init,
    .instance_finalize = tsens845_finalize,
    .class_init = tsens845_class_init,
};

static void tsens845_register_types(void)
{
    type_register_static(&tsens845_type);
}
type_init(tsens845_register_types)

void qcom845_tsens_create(DeviceState *gic)
{
    static const hwaddr bases[2][2] = {
        { 0x0c222000, 0x0c263000 }, { 0x0c223000, 0x0c265000 },
    };
    static const unsigned intids[2][2] = { { 538, 540 }, { 539, 541 } };

    for (unsigned i = 0; i < 2; i++) {
        Qcom845Tsens *s = QCOM845_TSENS(qdev_new(TYPE_QCOM845_TSENS));
        SysBusDevice *dev = SYS_BUS_DEVICE(s);

        s->index = i;
        s->sensors = i == 0 ? 13 : 8;
        sysbus_realize_and_unref(dev, &error_fatal);
        for (unsigned j = 0; j < 2; j++) {
            sysbus_mmio_map(dev, j, bases[i][j]);
            sysbus_connect_irq(dev, j, qdev_get_gpio_in(gic, intids[i][j] - 32));
        }
    }
}
