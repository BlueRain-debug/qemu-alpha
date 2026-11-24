/*
 * SDM845 CPRh v4.5 firmware handoff register frontend for OEMSK850.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Native PEP's APC0/APC1 tables supply a 400 mV base and 4 mV steps.
 * Seed the configured OSM handoff that generic ArmVirt firmware cannot
 * provide. Register writes persist, but there is no voltage feedback,
 * physical PMIC, CPU frequency scaling or ring-oscillator measurement.
 * Layout: Qualcomm's public cpr3-regulator.c (CPRh v4.5).
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"

#define TYPE_QCOM845_CPRH "qcom845-cprh"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Cprh, QCOM845_CPRH)
#define CPRH_SIZE 0x4000
#define CPRH_CTL 0x3a80
#define CPRH_STATUS0 0x3a84
#define CPRH_STATUS1 0x3a88

struct Qcom845Cprh {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint32_t regs[CPRH_SIZE / 4];
};

static uint64_t cprh845_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom845Cprh *s = opaque;

    return s->regs[offset / 4];
}

static void cprh845_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    Qcom845Cprh *s = opaque;

    if (offset != 0 && offset != 8 && offset != CPRH_STATUS0 &&
        offset != CPRH_STATUS1) {
        s->regs[offset / 4] = value;
    }
}

static const MemoryRegionOps cprh845_ops = {
    .read = cprh845_read,
    .write = cprh845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void cprh845_reset(DeviceState *dev)
{
    Qcom845Cprh *s = QCOM845_CPRH(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = 0x40050000;
    s->regs[CPRH_CTL / 4] = 1 | ((400 / 4) << 1) | (4 << 25);
}

static void cprh845_init(Object *obj)
{
    Qcom845Cprh *s = QCOM845_CPRH(obj);

    memory_region_init_io(&s->mmio, obj, &cprh845_ops, s,
                          TYPE_QCOM845_CPRH, CPRH_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription cprh845_vmstate = {
    .name = TYPE_QCOM845_CPRH,
    .unmigratable = true,
};

static void cprh845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, cprh845_reset);
    dc->desc = "SDM845 CPRh v4.5 firmware handoff frontend";
    dc->vmsd = &cprh845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo cprh845_type = {
    .name = TYPE_QCOM845_CPRH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Cprh),
    .instance_init = cprh845_init,
    .class_init = cprh845_class_init,
};

static void cprh845_register_types(void)
{
    type_register_static(&cprh845_type);
}
type_init(cprh845_register_types)

void qcom845_cprh_create(void)
{
    sysbus_create_simple(TYPE_QCOM845_CPRH, 0x17dc0000, NULL);
    sysbus_create_simple(TYPE_QCOM845_CPRH, 0x17db0000, NULL);
}
