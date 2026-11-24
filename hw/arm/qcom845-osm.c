/*
 * SDM845 OSM register/firmware-LUT frontend for OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Eighteen explicitly virtual operating points per L3/Silver/Gold domain,
 * repeated final LUT row, bounded per-core performance selection. There is
 * no physical DVFS, LMh throttling, voltage feedback or TCG clock scaling.
 * Register layout: Linux qcom-cpufreq-hw.c and sdm845.dtsi.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_QCOM845_OSM "qcom845-osm"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Osm, QCOM845_OSM)
#define OSM_SIZE 0x1400
#define OSM_LUT_ENTRIES 40
#define OSM_OPPS 18
#define OSM_CORES 4

typedef struct Osm845Bank {
    MemoryRegion mmio;
    unsigned index;
    uint32_t enable;
    uint32_t per_core;
    uint32_t performance[OSM_CORES];
} Osm845Bank;

struct Qcom845Osm {
    SysBusDevice parent_obj;
    Osm845Bank banks[3];
};

/* Virtual firmware choices, not values read back from physical OSM SRAM. */
static const uint8_t osm845_lval_ranges[3][2] = {
    { 16, 50 }, { 30, 94 }, { 30, 146 },
};

static uint32_t osm845_lval(Osm845Bank *bank, unsigned row)
{
    unsigned low = osm845_lval_ranges[bank->index][0];
    unsigned high = osm845_lval_ranges[bank->index][1];

    return low + (high - low) * MIN(row, OSM_OPPS - 1) / (OSM_OPPS - 1);
}

static uint64_t osm845_read(void *opaque, hwaddr offset, unsigned size)
{
    Osm845Bank *bank = opaque;

    if (offset == 0) {
        return bank->enable;
    }
    if (offset == 0xbc) {
        return bank->per_core;
    }
    if (offset >= 0x110 && offset < 0x110 + OSM_LUT_ENTRIES * 32) {
        unsigned row = MIN((offset - 0x110) / 32, OSM_OPPS - 1);
        unsigned field = (offset - 0x110) % 32;

        if (field == 0) {
            return BIT(30) | (OSM_CORES << 16) | osm845_lval(bank, row);
        }
        if (field == 4) {
            return 800; /* Virtual voltage metadata, millivolts. */
        }
        return 0;
    }
    if (offset >= 0x920 && offset < 0x920 + OSM_CORES * 4) {
        return bank->performance[(offset - 0x920) / 4];
    }
    if (offset == 0x704 || offset == 0x20) {
        return osm845_lval(bank, bank->performance[0]);
    }
    return 0;
}

static void osm845_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    Osm845Bank *bank = opaque;

    if (offset == 0) {
        bank->enable = value & 1;
    } else if (offset == 0xbc) {
        bank->per_core = value & 1;
    } else if (offset >= 0x920 && offset < 0x920 + OSM_CORES * 4) {
        if (value >= OSM_LUT_ENTRIES) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "qcom845-osm: invalid performance index %"PRIu64"\n",
                          value);
            return;
        }
        bank->performance[(offset - 0x920) / 4] = value;
    } else {
        qemu_log_mask(LOG_UNIMP, "qcom845-osm: unimplemented write 0x%"
                      HWADDR_PRIx "\n", offset);
    }
}

static const MemoryRegionOps osm845_ops = {
    .read = osm845_read,
    .write = osm845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void osm845_reset(DeviceState *dev)
{
    Qcom845Osm *s = QCOM845_OSM(dev);

    for (unsigned i = 0; i < ARRAY_SIZE(s->banks); i++) {
        s->banks[i].enable = 1; /* Virtual firmware handoff is initialized. */
        s->banks[i].per_core = 0;
        memset(s->banks[i].performance, 0, sizeof(s->banks[i].performance));
    }
}

static void osm845_init(Object *obj)
{
    Qcom845Osm *s = QCOM845_OSM(obj);

    for (unsigned i = 0; i < ARRAY_SIZE(s->banks); i++) {
        s->banks[i].index = i;
        memory_region_init_io(&s->banks[i].mmio, obj, &osm845_ops, &s->banks[i],
                              TYPE_QCOM845_OSM, OSM_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->banks[i].mmio);
    }
}

static const VMStateDescription osm845_vmstate = {
    .name = TYPE_QCOM845_OSM,
    .unmigratable = true,
};

static void osm845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, osm845_reset);
    dc->desc = "Experimental SDM845 OSM with virtual firmware LUTs";
    dc->vmsd = &osm845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo osm845_type = {
    .name = TYPE_QCOM845_OSM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Osm),
    .instance_init = osm845_init,
    .class_init = osm845_class_init,
};

static void osm845_register_types(void)
{
    type_register_static(&osm845_type);
}
type_init(osm845_register_types)

void qcom845_osm_create(void)
{
    static const hwaddr bases[] = { 0x17d41000, 0x17d43000, 0x17d45800 };
    SysBusDevice *dev = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_OSM));

    sysbus_realize_and_unref(dev, &error_fatal);
    for (unsigned i = 0; i < ARRAY_SIZE(bases); i++) {
        sysbus_mmio_map(dev, i, bases[i]);
    }
}
