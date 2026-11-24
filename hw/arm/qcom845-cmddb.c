/*
 * Read-only firmware Command DB and AOP directory frontend for OEMSK850.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Vendor tables are supplied externally, never fabricated by this device.
 * The directory relocates the DB into emulated AOP SRAM, outside guest RAM.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/loader.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/module.h"

#define TYPE_QCOM845_CMDDB "qcom845-cmddb"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845CmdDb, QCOM845_CMDDB)
#define CMDDB_BASE 0x0c3d0000
#define CMDDB_SIZE 0x20000
#define CMDDB_HEADER 0x90

typedef struct CmdDb845Region {
    Qcom845CmdDb *owner;
    bool directory;
    MemoryRegion mmio;
} CmdDb845Region;

struct Qcom845CmdDb {
    SysBusDevice parent_obj;
    CmdDb845Region regions[2];
    char *file;
    unsigned length;
    uint8_t data[CMDDB_SIZE];
};

static bool cmddb845_validate(Qcom845CmdDb *s)
{
    if (s->length < CMDDB_HEADER || ldl_le_p(s->data) != 1 ||
        ldl_le_p(s->data + 4) != 0x0c0330db) {
        return false;
    }
    for (unsigned group = 0; group < 8; group++) {
        uint8_t *h = s->data + 8 + group * 16;
        unsigned slave = lduw_le_p(h);
        unsigned headers = CMDDB_HEADER + lduw_le_p(h + 2);
        unsigned aux = CMDDB_HEADER + lduw_le_p(h + 4);
        unsigned count = lduw_le_p(h + 6);

        if (!slave) {
            break;
        }
        if (slave > 7 || headers > s->length ||
            count > (s->length - headers) / 24 || aux > s->length) {
            return false;
        }
        for (unsigned i = 0; i < count; i++) {
            uint8_t *e = s->data + headers + i * 24;
            unsigned offset = lduw_le_p(e + 22);
            unsigned length = lduw_le_p(e + 20);

            if (((ldl_le_p(e + 16) >> 16) & 7) != slave ||
                offset > s->length - aux || length > s->length - aux - offset) {
                return false;
            }
        }
    }
    return true;
}

static uint64_t cmddb845_load(const uint8_t *data, unsigned size)
{
    uint64_t value = 0;

    for (unsigned i = 0; i < size; i++) {
        value |= (uint64_t)data[i] << (i * 8);
    }
    return value;
}

static uint64_t cmddb845_read(void *opaque, hwaddr offset, unsigned size)
{
    CmdDb845Region *region = opaque;
    Qcom845CmdDb *s = region->owner;

    if (region->directory) {
        uint8_t directory[0x18] = { 0 };

        if (s->length) {
            stl_le_p(directory + 0xc, CMDDB_BASE);
            stl_le_p(directory + 0x10, s->length);
        }
        return offset + size <= sizeof(directory) ?
               cmddb845_load(directory + offset, size) : 0;
    }
    return cmddb845_load(s->data + offset, size);
}

static void cmddb845_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    /* Firmware-owned read-only table and directory. */
}

static const MemoryRegionOps cmddb845_ops = {
    .read = cmddb845_read,
    .write = cmddb845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void cmddb845_realize(DeviceState *dev, Error **errp)
{
    Qcom845CmdDb *s = QCOM845_CMDDB(dev);
    int64_t length;

    if (!s->file) {
        return; /* Absent DB is reported by a zero directory entry. */
    }
    length = get_image_size(s->file, errp);
    if (length < 0) {
        return;
    }
    if (length < CMDDB_HEADER || length > CMDDB_SIZE ||
        load_image_size(s->file, s->data, sizeof(s->data)) != length) {
        error_setg(errp, "oemsk850: cannot load bounded CmdDB '%s'", s->file);
        return;
    }
    s->length = length;
    if (!cmddb845_validate(s)) {
        error_setg(errp, "oemsk850: malformed Command DB '%s'", s->file);
    }
}

static void cmddb845_init(Object *obj)
{
    Qcom845CmdDb *s = QCOM845_CMDDB(obj);

    for (unsigned i = 0; i < 2; i++) {
        s->regions[i].owner = s;
        s->regions[i].directory = i == 1;
        memory_region_init_io(&s->regions[i].mmio, obj, &cmddb845_ops,
                              &s->regions[i], TYPE_QCOM845_CMDDB,
                              i == 0 ? CMDDB_SIZE : 0x1000);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->regions[i].mmio);
    }
}

static const Property cmddb845_properties[] = {
    DEFINE_PROP_STRING("file", Qcom845CmdDb, file),
};

static const VMStateDescription cmddb845_vmstate = {
    .name = TYPE_QCOM845_CMDDB,
    .unmigratable = true,
};

static void cmddb845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = cmddb845_realize;
    dc->desc = "SDM845 read-only firmware Command DB and AOP directory";
    dc->vmsd = &cmddb845_vmstate;
    dc->user_creatable = false;
    device_class_set_props(dc, cmddb845_properties);
}

static const TypeInfo cmddb845_type = {
    .name = TYPE_QCOM845_CMDDB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845CmdDb),
    .instance_init = cmddb845_init,
    .class_init = cmddb845_class_init,
};

static void cmddb845_register_types(void)
{
    type_register_static(&cmddb845_type);
}
type_init(cmddb845_register_types)

void qcom845_cmddb_create(const char *file)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_CMDDB));

    if (file && file[0]) {
        qdev_prop_set_string(DEVICE(dev), "file", file);
    }
    sysbus_realize_and_unref(dev, &error_fatal);
    sysbus_mmio_map(dev, 0, CMDDB_BASE);
    sysbus_mmio_map(dev, 1, 0x0c3f0000);
}
