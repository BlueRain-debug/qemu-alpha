/*
 * Experimental Qualcomm GPT/DGT subset for the virt Windows ARM platform.
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

#define TYPE_QCOM_GPT "qcom-gpt"
OBJECT_DECLARE_SIMPLE_TYPE(QcomGptState, QCOM_GPT)

typedef struct QcomGptChannel {
    QEMUTimer *timer;
    qemu_irq irq;
    uint32_t count;
    uint32_t match;
    uint32_t enable;
    uint32_t clock_ctl;
    uint32_t input_frequency;
    uint32_t frequency;
    int64_t epoch;
    bool pending;
} QcomGptChannel;

struct QcomGptState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    QcomGptChannel channel[2];
};

static uint32_t qcom_gpt_count(QcomGptChannel *c)
{
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - c->epoch;

    return c->count + ((c->enable & 1) && elapsed > 0 ?
        muldiv64(elapsed, c->frequency, NANOSECONDS_PER_SECOND) : 0);
}

static void qcom_gpt_rearm(QcomGptChannel *c)
{
    uint32_t delta;

    timer_del(c->timer);
    if (!(c->enable & 1) || c->pending) {
        return;
    }
    delta = c->match - qcom_gpt_count(c);
    timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              muldiv64_round_up(delta ? delta : (1ULL << 32),
                               NANOSECONDS_PER_SECOND, c->frequency));
}

static void qcom_gpt_expire(void *opaque)
{
    QcomGptChannel *c = opaque;

    c->pending = true;
    qemu_set_irq(c->irq, 1);
    if (c->enable & 2) {
        c->count = 0;
        c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static QcomGptChannel *qcom_gpt_channel(QcomGptState *s, hwaddr *offset)
{
    if (*offset < 0x14) {
        return &s->channel[0];
    }
    if (*offset >= 0x24 && *offset < 0x38) {
        *offset -= 0x24;
        return &s->channel[1];
    }
    return NULL;
}

static uint64_t qcom_gpt_read(void *opaque, hwaddr offset, unsigned size)
{
    QcomGptChannel *c = qcom_gpt_channel(opaque, &offset);

    if (c) {
        switch (offset) {
        case 0: return c->match;
        case 4: return qcom_gpt_count(c);
        case 8: return c->enable;
        case 12: return 0;
        case 16: return c->clock_ctl;
        }
    }
    qemu_log_mask(LOG_UNIMP, "qcom-gpt: unsupported read at 0x%" HWADDR_PRIx
                  "\n", offset);
    return 0;
}

static void qcom_gpt_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    QcomGptChannel *c = qcom_gpt_channel(opaque, &offset);

    if (c) {
        switch (offset) {
        case 0:
            c->match = value;
            c->pending = false;
            qemu_set_irq(c->irq, 0);
            break;
        case 8:
            c->count = qcom_gpt_count(c);
            c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            c->enable = value & 3;
            break;
        case 12:
            c->count = 0;
            c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            c->pending = false;
            qemu_set_irq(c->irq, 0);
            break;
        case 16:
            c->count = qcom_gpt_count(c);
            c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            c->clock_ctl = value;
            c->frequency = c->input_frequency / ((value & 3) + 1);
            break;
        default:
            return;
        }
        qemu_set_irq(c->irq, c->pending && (c->enable & 1));
        qcom_gpt_rearm(c);
    } else {
        qemu_log_mask(LOG_UNIMP, "qcom-gpt: unsupported write at 0x%"
                      HWADDR_PRIx "\n", offset);
    }
}

static const MemoryRegionOps qcom_gpt_ops = {
    .read = qcom_gpt_read,
    .write = qcom_gpt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void qcom_gpt_reset(DeviceState *dev)
{
    QcomGptState *s = QCOM_GPT(dev);

    for (unsigned i = 0; i < 2; i++) {
        QcomGptChannel *c = &s->channel[i];

        timer_del(c->timer);
        c->count = c->enable = c->clock_ctl = 0;
        c->frequency = c->input_frequency;
        c->match = UINT32_MAX;
        c->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        c->pending = false;
        qemu_set_irq(c->irq, 0);
    }
}

static int qcom_gpt_post_load(void *opaque, int version)
{
    QcomGptState *s = opaque;

    for (unsigned i = 0; i < 2; i++) {
        qemu_set_irq(s->channel[i].irq,
                     s->channel[i].pending && (s->channel[i].enable & 1));
        qcom_gpt_rearm(&s->channel[i]);
    }
    return 0;
}

static const VMStateDescription vmstate_qcom_gpt_channel = {
    .name = "qcom-gpt/channel", .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(count, QcomGptChannel),
        VMSTATE_UINT32(match, QcomGptChannel),
        VMSTATE_UINT32(enable, QcomGptChannel),
        VMSTATE_UINT32(clock_ctl, QcomGptChannel),
        VMSTATE_UINT32(frequency, QcomGptChannel),
        VMSTATE_INT64(epoch, QcomGptChannel),
        VMSTATE_BOOL(pending, QcomGptChannel),
        VMSTATE_TIMER_PTR(timer, QcomGptChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_qcom_gpt = {
    .name = TYPE_QCOM_GPT, .version_id = 1, .minimum_version_id = 1,
    .post_load = qcom_gpt_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(channel, QcomGptState, 2, 1,
                              vmstate_qcom_gpt_channel, QcomGptChannel),
        VMSTATE_END_OF_LIST()
    }
};

static void qcom_gpt_init(Object *obj)
{
    QcomGptState *s = QCOM_GPT(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &qcom_gpt_ops, s, TYPE_QCOM_GPT, 0x1000);
    sysbus_init_mmio(dev, &s->mmio);
    for (unsigned i = 0; i < 2; i++) {
        s->channel[i].input_frequency = i ? 27000000 : 32768;
        s->channel[i].frequency = s->channel[i].input_frequency;
        s->channel[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          qcom_gpt_expire, &s->channel[i]);
        sysbus_init_irq(dev, &s->channel[i].irq);
    }
}

static void qcom_gpt_finalize(Object *obj)
{
    QcomGptState *s = QCOM_GPT(obj);

    for (unsigned i = 0; i < 2; i++) {
        timer_free(s->channel[i].timer);
    }
}

static void qcom_gpt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_qcom_gpt;
    device_class_set_legacy_reset(dc, qcom_gpt_reset);
}

static const TypeInfo qcom_gpt_info = {
    .name = TYPE_QCOM_GPT, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(QcomGptState),
    .instance_init = qcom_gpt_init,
    .instance_finalize = qcom_gpt_finalize,
    .class_init = qcom_gpt_class_init,
};

static void qcom_gpt_register_types(void)
{
    type_register_static(&qcom_gpt_info);
}
type_init(qcom_gpt_register_types)
