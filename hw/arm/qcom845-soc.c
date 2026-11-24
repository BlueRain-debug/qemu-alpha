/*
 * SDM845 identity and virtual unprogrammed fuse windows for OEMSK850.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * QCpep845 reads HW_VERSION at 0x01fc8000 (major in [15:8]), and maps
 * JTAG part 0x8b at 0x03998010 to chip ID 321. Original board LogFS reports
 * SDM845 2.1. Physical per-device CPRh calibration fuses are unavailable;
 * the virtual QFPROM bank is zero, not a copy of a real device's fuses.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "trace.h"

#define TYPE_QCOM845_SOC "qcom845-soc"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Soc, QCOM845_SOC)

typedef struct Soc845Region {
    unsigned index;
    MemoryRegion mmio;
} Soc845Region;

struct Qcom845Soc {
    SysBusDevice parent_obj;
    Soc845Region regions[3];
    MemoryRegion imem;
};

static const hwaddr soc845_bases[] = { 0x01fc8000, 0x03998000, 0x00780000 };
static const hwaddr soc845_sizes[] = { 0x1000, 0x1000, 0x7000 };
#define QCOM845_IMEM_BASE 0x14680000
#define QCOM845_IMEM_SIZE 0x40000

static uint64_t soc845_read(void *opaque, hwaddr offset, unsigned size)
{
    Soc845Region *region = opaque;
    uint32_t value = 0;

    if (region->index == 0 && offset == 0) {
        value = 0x0201;
    } else if (region->index == 1 && offset == 0x10) {
        value = (0x8b << 12) | 0x2e1; /* SDM845 part, Qualcomm manufacturer. */
    }
    trace_qcom845_soc_read(region->index, offset, value);
    return value;
}

static void soc845_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    /* Read-only identity and unprogrammed virtual fuses. */
}

static const MemoryRegionOps soc845_ops = {
    .read = soc845_read,
    .write = soc845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void soc845_init(Object *obj)
{
    Qcom845Soc *s = QCOM845_SOC(obj);

    for (unsigned i = 0; i < ARRAY_SIZE(s->regions); i++) {
        s->regions[i].index = i;
        memory_region_init_io(&s->regions[i].mmio, obj, &soc845_ops,
                              &s->regions[i], TYPE_QCOM845_SOC, soc845_sizes[i]);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->regions[i].mmio);
    }
    memory_region_init_ram(&s->imem, obj, "qcom845.imem", QCOM845_IMEM_SIZE,
                           &error_fatal);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->imem);
}

static void soc845_reset(DeviceState *dev)
{
    Qcom845Soc *s = QCOM845_SOC(dev);
    void *ptr = memory_region_get_ram_ptr(&s->imem);

    if (ptr) {
        memset(ptr, 0, QCOM845_IMEM_SIZE);
    }
}

static const VMStateDescription soc845_vmstate = {
    .name = TYPE_QCOM845_SOC,
    .unmigratable = true,
};

static void soc845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, soc845_reset);
    dc->desc = "SDM845 2.1 identity, IMEM and virtual unprogrammed QFPROM";
    dc->vmsd = &soc845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo soc845_type = {
    .name = TYPE_QCOM845_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Soc),
    .instance_init = soc845_init,
    .class_init = soc845_class_init,
};

static void soc845_register_types(void)
{
    type_register_static(&soc845_type);
}
type_init(soc845_register_types)

void qcom845_soc_create(void)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_SOC));

    sysbus_realize_and_unref(dev, &error_fatal);
    for (unsigned i = 0; i < ARRAY_SIZE(soc845_bases); i++) {
        sysbus_mmio_map(dev, i, soc845_bases[i]);
    }
    sysbus_mmio_map(dev, ARRAY_SIZE(soc845_bases), QCOM845_IMEM_BASE);
}
