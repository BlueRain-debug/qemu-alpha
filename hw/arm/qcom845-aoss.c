/*
 * SDM845 AOSS QMP link frontend for local OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A bounded mailbox, link/channel handshake and APSS doorbell. The only
 * implemented service is a software QDSS clock vote. Unknown messages remain
 * pending; no remote AOP firmware, rails or sleep sequencing is emulated.
 * Layout reference: Linux drivers/soc/qcom/qcom_aoss.c; native QCpep845
 * uses the same descriptor and the APSS_SHARED doorbell at 0x1799000c.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

#define TYPE_QCOM845_AOSS "qcom845-aoss"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Aoss, QCOM845_AOSS)
#define AOSS_SIZE 0x1000
#define AOSS_UP 0x0000ffff
#define AOSS_DOWN 0xffff0000
#define AOSS_TX 0x100
#define AOSS_RX 0x200
#define AOSS_MBOX_SIZE 0x100

typedef struct Aoss845Region {
    Qcom845Aoss *owner;
    bool doorbell;
    MemoryRegion mmio;
} Aoss845Region;

struct Qcom845Aoss {
    SysBusDevice parent_obj;
    Aoss845Region regions[2];
    qemu_irq irq;
    uint32_t words[AOSS_SIZE / 4];
    bool qdss_vote;
};

static bool aoss845_state(uint32_t value)
{
    return value == AOSS_UP || value == AOSS_DOWN;
}

static bool aoss845_message(Qcom845Aoss *s)
{
    uint32_t length = s->words[AOSS_TX / 4];
    char message[AOSS_MBOX_SIZE];
    unsigned at = 0;

    /* The header counts bytes; QCpep845 sends a 34-byte QDSS request. */
    if (!length || length > AOSS_MBOX_SIZE - 4) {
        return false;
    }
    for (unsigned i = 0; i < length; i++) {
        unsigned char byte = s->words[(AOSS_TX + 4 + i) / 4] >> ((i & 3) * 8);

        if (!byte) {
            break;
        }
        if (byte != ' ' && byte != '\t' && byte != '\r' && byte != '\n') {
            message[at++] = byte;
        }
    }
    message[at] = 0;
    if (!strcmp(message, "{class:clock,res:qdss,val:0}")) {
        s->qdss_vote = false;
    } else if (!strcmp(message, "{class:clock,res:qdss,val:1}")) {
        s->qdss_vote = true;
    } else {
        return false;
    }
    s->words[AOSS_TX / 4] = 0;
    return true;
}

static void aoss845_kick(Qcom845Aoss *s)
{
    uint32_t *w = s->words;
    uint32_t link = w[0x24 / 4], channel = w[0x2c / 4];
    bool changed = false;

    if (!aoss845_state(link) || !aoss845_state(channel)) {
        qemu_log_mask(LOG_GUEST_ERROR, "qcom845-aoss: invalid link state\n");
        return;
    }
    if (w[0x28 / 4] != link) {
        w[0x28 / 4] = link;
        changed = true;
    }
    if (link == AOSS_DOWN || w[0x10 / 4] == AOSS_UP) {
        uint32_t remote = link == AOSS_UP ? channel : AOSS_DOWN;

        if (w[0x14 / 4] != remote || w[0x30 / 4] != remote) {
            w[0x14 / 4] = remote;
            w[0x30 / 4] = remote;
            changed = true;
        }
    }
    if (link == AOSS_UP && w[0x10 / 4] == AOSS_UP &&
        channel == AOSS_UP && w[0x18 / 4] == AOSS_UP && w[AOSS_TX / 4]) {
        uint32_t bytes = w[AOSS_TX / 4];
        bool accepted = aoss845_message(s);

        trace_qcom845_aoss_message(bytes, w[(AOSS_TX + 4) / 4],
                                   w[(AOSS_TX + 8) / 4], accepted);
        changed |= accepted;
    }
    if (changed) {
        qemu_irq_pulse(s->irq); /* Native PEP _CRS declares an edge IRQ. */
    }
}

static uint64_t aoss845_read(void *opaque, hwaddr offset, unsigned size)
{
    Aoss845Region *region = opaque;
    uint32_t value = region->doorbell ? 0 : region->owner->words[offset / 4];

    trace_qcom845_aoss_access(region->doorbell, false, offset, value);
    return value;
}

static void aoss845_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    Aoss845Region *region = opaque;
    Qcom845Aoss *s = region->owner;

    trace_qcom845_aoss_access(region->doorbell, true, offset, value);
    if (region->doorbell) {
        if (offset == 0xc && (value & 1)) {
            aoss845_kick(s);
        }
    } else if (offset == 0x10 || offset == 0x18 || offset == 0x24 ||
               offset == 0x2c ||
               (offset >= AOSS_TX && offset < AOSS_TX + AOSS_MBOX_SIZE)) {
        s->words[offset / 4] = value;
    }
}

static const MemoryRegionOps aoss845_ops = {
    .read = aoss845_read,
    .write = aoss845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aoss845_reset(DeviceState *dev)
{
    Qcom845Aoss *s = QCOM845_AOSS(dev);

    memset(s->words, 0, sizeof(s->words));
    s->qdss_vote = false;
    s->words[0] = 0x4d41494c;
    s->words[1] = 1;
    s->words[0x0c / 4] = AOSS_UP;
    s->words[0x10 / 4] = AOSS_DOWN;
    s->words[0x14 / 4] = AOSS_DOWN;
    s->words[0x18 / 4] = AOSS_DOWN;
    s->words[0x1c / 4] = AOSS_MBOX_SIZE;
    s->words[0x20 / 4] = AOSS_RX;
    s->words[0x24 / 4] = AOSS_DOWN;
    s->words[0x28 / 4] = AOSS_DOWN;
    s->words[0x2c / 4] = AOSS_DOWN;
    s->words[0x30 / 4] = AOSS_DOWN;
    s->words[0x34 / 4] = AOSS_MBOX_SIZE;
    s->words[0x38 / 4] = AOSS_TX;
    qemu_set_irq(s->irq, 0);
}

static void aoss845_init(Object *obj)
{
    Qcom845Aoss *s = QCOM845_AOSS(obj);

    for (unsigned i = 0; i < 2; i++) {
        s->regions[i].owner = s;
        s->regions[i].doorbell = i == 1;
        memory_region_init_io(&s->regions[i].mmio, obj, &aoss845_ops,
                              &s->regions[i], TYPE_QCOM845_AOSS, AOSS_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->regions[i].mmio);
    }
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription aoss845_vmstate = {
    .name = TYPE_QCOM845_AOSS,
    .unmigratable = true,
};

static void aoss845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, aoss845_reset);
    dc->desc = "Experimental SDM845 AOSS QMP link and APSS doorbell";
    dc->vmsd = &aoss845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo aoss845_type = {
    .name = TYPE_QCOM845_AOSS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Aoss),
    .instance_init = aoss845_init,
    .class_init = aoss845_class_init,
};

static void aoss845_register_types(void)
{
    type_register_static(&aoss845_type);
}
type_init(aoss845_register_types)

void qcom845_aoss_create(DeviceState *gic)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_AOSS));

    sysbus_realize_and_unref(dev, &error_fatal);
    sysbus_mmio_map(dev, 0, 0x0c300000);
    sysbus_mmio_map(dev, 1, 0x17990000);
    sysbus_connect_irq(dev, 0, qdev_get_gpio_in(gic, 421 - 32));
}
