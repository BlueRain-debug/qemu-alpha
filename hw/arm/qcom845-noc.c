/*
 * SDM845 NoC QoS configuration frontend for local OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Only QoS generator configuration is modeled, not transaction arbitration,
 * bandwidth throttling, counters, or NoC error reporting. Each virtual fabric
 * has sixteen bounded ports. Unimplemented fields remain unassigned.
 * Layout/masks: Qualcomm msm_bus_noc_rpmh.c and sdm845-bus.dtsi in
 * android-msm-crosshatch-4.9-pie-qpr1 (drivers/soc/qcom/msm_bus/ and
 * arch/arm64/boot/dts/qcom/).
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"

#define TYPE_QCOM845_NOC "qcom845-noc"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Noc, QCOM845_NOC)
#define NOC_FABRICS 5
#define NOC_PORTS 16
#define NOC_FIELDS 4

typedef struct Noc845Field {
    MemoryRegion mmio;
    uint32_t reg;
    uint32_t mask;
} Noc845Field;

struct Qcom845Noc {
    SysBusDevice parent_obj;
    MemoryRegion fabrics[NOC_FABRICS];
    Noc845Field fields[NOC_FABRICS][NOC_PORTS][NOC_FIELDS];
};

static const hwaddr noc845_qos_bases[NOC_FABRICS] = {
    0x016e4000, 0x01704000, 0x01390000, 0x01749000, 0x01629000,
};
static const unsigned noc845_offsets[NOC_FIELDS] = { 0x8, 0x18, 0x40, 0x48 };
static const uint32_t noc845_masks[NOC_FIELDS] = {
    0x79, 0x03ff07ff, 0x7703, 0x03ff07ff,
};

static uint64_t noc845_read(void *opaque, hwaddr offset, unsigned size)
{
    Noc845Field *field = opaque;

    return field->reg;
}

static void noc845_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    Noc845Field *field = opaque;

    field->reg = value & field->mask;
}

static const MemoryRegionOps noc845_ops = {
    .read = noc845_read,
    .write = noc845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void noc845_reset(DeviceState *dev)
{
    Qcom845Noc *s = QCOM845_NOC(dev);

    for (unsigned i = 0; i < NOC_FABRICS; i++) {
        for (unsigned p = 0; p < NOC_PORTS; p++) {
            for (unsigned f = 0; f < NOC_FIELDS; f++) {
                s->fields[i][p][f].reg = 0;
            }
        }
    }
}

static void noc845_init(Object *obj)
{
    Qcom845Noc *s = QCOM845_NOC(obj);

    for (unsigned i = 0; i < NOC_FABRICS; i++) {
        memory_region_init(&s->fabrics[i], obj, TYPE_QCOM845_NOC, 0x10000);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->fabrics[i]);
        for (unsigned p = 0; p < NOC_PORTS; p++) {
            for (unsigned f = 0; f < NOC_FIELDS; f++) {
                Noc845Field *field = &s->fields[i][p][f];

                field->mask = noc845_masks[f];
                memory_region_init_io(&field->mmio, obj, &noc845_ops,
                                      field, TYPE_QCOM845_NOC, 4);
                memory_region_add_subregion(&s->fabrics[i],
                                            p * 0x1000 + noc845_offsets[f],
                                            &field->mmio);
            }
        }
    }
}

static const VMStateDescription noc845_vmstate = {
    .name = TYPE_QCOM845_NOC,
    .unmigratable = true,
};

static void noc845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, noc845_reset);
    dc->desc = "Experimental SDM845 NoC QoS configuration registers";
    dc->vmsd = &noc845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo noc845_type = {
    .name = TYPE_QCOM845_NOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Noc),
    .instance_init = noc845_init,
    .class_init = noc845_class_init,
};

static void noc845_register_types(void)
{
    type_register_static(&noc845_type);
}
type_init(noc845_register_types)

void qcom845_noc_create(void)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_NOC));

    sysbus_realize_and_unref(dev, &error_fatal);
    for (unsigned i = 0; i < NOC_FABRICS; i++) {
        sysbus_mmio_map(dev, i, noc845_qos_bases[i]);
    }
}
