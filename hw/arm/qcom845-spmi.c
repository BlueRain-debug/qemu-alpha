/*
 * SDM845 SPMI PMIC Arbiter v5: discovery and PMIC revision reads.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This is a separate device from MSM8998's v3 arbiter. No rail/voltage,
 * GPIO or interrupt controller functionality is claimed by this subset.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

#define TYPE_QCOM845_SPMI "qcom845-spmi"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Spmi, QCOM845_SPMI)

#define ARB_CORE 0x40000
#define ARB_APID_MAP (ARB_CORE + 0x900)
#define ARB_OWNER 0xa700
#define ARB_APIDS 512
#define ARB_WR_CHANNELS 0x200000
#define ARB_RD_CHANNELS 0x2200000
#define ARB_WR_STRIDE 0x10000
#define ARB_RD_STRIDE 0x80
#define ARB_RD_EE_STRIDE 0x10000
#define ARB_DONE BIT(0)
#define ARB_FAILURE BIT(1)

static const struct {
    uint8_t slave;
    uint8_t subtype;
    uint8_t major;
    uint8_t fab;
} revision_ids[] = {
    { 0, 0x14, 2, 0 },       /* PM8998 */
    { 2, 0x15, 2, 0x11 },   /* PMI8998 */
    { 4, 0x18, 1, 0 },      /* PM8005 */
};

typedef struct Spmi845Channel {
    uint32_t status;
    uint64_t data;
} Spmi845Channel;

struct Qcom845Spmi {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    Spmi845Channel read[ARRAY_SIZE(revision_ids)];
    Spmi845Channel write[ARRAY_SIZE(revision_ids)];
};

static Spmi845Channel *spmi845_channel(Qcom845Spmi *s, hwaddr *offset,
                                     unsigned *apid, bool *observer)
{
    if (*offset >= ARB_RD_CHANNELS && *offset < ARB_RD_CHANNELS + 0x100000) {
        hwaddr local = (*offset - ARB_RD_CHANNELS) % ARB_RD_EE_STRIDE;

        *apid = local / ARB_RD_STRIDE;
        *offset = local % ARB_RD_STRIDE;
        *observer = true;
    } else if (*offset >= ARB_WR_CHANNELS && *offset < ARB_RD_CHANNELS) {
        *offset -= ARB_WR_CHANNELS;
        *apid = *offset / ARB_WR_STRIDE;
        *offset %= ARB_WR_STRIDE;
        *observer = false;
    } else {
        return NULL;
    }
    if (*apid >= ARRAY_SIZE(revision_ids)) {
        return NULL;
    }
    return *observer ? &s->read[*apid] : &s->write[*apid];
}

static uint8_t spmi845_revision(unsigned apid, unsigned reg)
{
    switch (reg) {
    case 3:
        return revision_ids[apid].major;
    case 4:
        return 0x51;
    case 5:
        return revision_ids[apid].subtype;
    case 0xf2:
        return revision_ids[apid].fab;
    default:
        return 0;
    }
}

static uint64_t spmi845_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom845Spmi *s = opaque;
    Spmi845Channel *channel;
    unsigned apid;
    bool observer;
    uint32_t value = 0;
    hwaddr address = offset;

    if (offset == ARB_CORE) {
        value = 0x50000000;
    } else if (offset >= ARB_APID_MAP &&
               offset < ARB_APID_MAP + ARB_APIDS * 4) {
        apid = (offset - ARB_APID_MAP) / 4;
        if (apid < ARRAY_SIZE(revision_ids)) {
            value = (revision_ids[apid].slave << 16) | (1U << 8);
        }
    } else if (offset >= ARB_OWNER && offset < ARB_OWNER + ARB_APIDS * 4) {
        value = 0; /* HLOS execution environment owns these revision APIDs. */
    } else {
        channel = spmi845_channel(s, &offset, &apid, &observer);
        if (channel) {
            switch (offset) {
            case 0x08:
                value = channel->status;
                break;
            case 0x18:
                value = channel->data;
                break;
            case 0x1c:
                value = channel->data >> 32;
                break;
            }
        } else {
            qemu_log_mask(LOG_UNIMP, "qcom845-spmi: read at 0x%"HWADDR_PRIx
                          " not implemented\n", address);
        }
    }
    trace_qcom845_spmi_access(false, address, value);
    return value;
}

static void spmi845_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    Qcom845Spmi *s = opaque;
    Spmi845Channel *channel;
    unsigned apid;
    bool observer;

    trace_qcom845_spmi_access(true, offset, value);
    channel = spmi845_channel(s, &offset, &apid, &observer);
    if (!channel || offset != 0) {
        qemu_log_mask(LOG_UNIMP, "qcom845-spmi: write not implemented\n");
        return;
    }
    channel->status = ARB_DONE | ARB_FAILURE;
    channel->data = 0;
    if (observer && (value >> 27) == 1) { /* EXT_READL */
        /* v5 selects the peripheral through the APID, not the command. */
        unsigned address = (value >> 4) & 0xff;
        unsigned bytes = (value & 7) + 1;

        if (address + bytes > 256) {
            return;
        }
        for (unsigned i = 0; i < bytes; i++) {
            channel->data |= (uint64_t)spmi845_revision(apid,
                                        address + i) << (i * 8);
        }
        channel->status = ARB_DONE;
    }
}

static const MemoryRegionOps spmi845_ops = {
    .read = spmi845_read,
    .write = spmi845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void spmi845_reset(DeviceState *dev)
{
    Qcom845Spmi *s = QCOM845_SPMI(dev);

    for (unsigned i = 0; i < ARRAY_SIZE(revision_ids); i++) {
        s->read[i] = (Spmi845Channel) { .status = ARB_DONE };
        s->write[i] = (Spmi845Channel) { .status = ARB_DONE };
    }
}

static void spmi845_init(Object *obj)
{
    Qcom845Spmi *s = QCOM845_SPMI(obj);

    memory_region_init_io(&s->mmio, obj, &spmi845_ops, s,
                          TYPE_QCOM845_SPMI, QCOM845_SPMI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription spmi845_vmstate = {
    .name = TYPE_QCOM845_SPMI,
    .unmigratable = true,
};

static void spmi845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, spmi845_reset);
    dc->desc = "Experimental SDM845 SPMI v5 revision transport";
    dc->vmsd = &spmi845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo spmi845_type = {
    .name = TYPE_QCOM845_SPMI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Spmi),
    .instance_init = spmi845_init,
    .class_init = spmi845_class_init,
};

static void spmi845_register_types(void)
{
    type_register_static(&spmi845_type);
}
type_init(spmi845_register_types)

void qcom845_spmi_create(void)
{
    SysBusDevice *bus = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_SPMI));

    sysbus_realize_and_unref(bus, &error_fatal);
    sysbus_mmio_map(bus, 0, QCOM845_SPMI_BASE);
}
