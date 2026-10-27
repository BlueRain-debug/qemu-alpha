/*
 * QEMU USB EHCI Emulation
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/usb/hcd-ehci.h"
#include "migration/vmstate.h"

#define TEGRA2_PHY_BASE          0x400
#define TEGRA2_PHY_REGS          (0xc00 / sizeof(uint32_t))
#define TEGRA2_PHY_CLK_VALID     BIT(7)
#define TEGRA2_UTMIP_RESET       BIT(11)
#define TEGRA2_UTMIP_ENABLE      BIT(12)

OBJECT_DECLARE_SIMPLE_TYPE(Tegra2EHCIState, TEGRA2_EHCI)

struct Tegra2EHCIState {
    EHCISysBusState parent_obj;
    MemoryRegion phy;
    uint32_t phy_regs[TEGRA2_PHY_REGS];
};

static bool tegra2_phy_needed(void *opaque)
{
    return object_dynamic_cast(OBJECT(opaque), TYPE_TEGRA2_EHCI) != NULL;
}

static const VMStateDescription vmstate_tegra2_phy = {
    .name = "ehci-sysbus/tegra2-phy",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = tegra2_phy_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(phy_regs, Tegra2EHCIState, TEGRA2_PHY_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ehci_sysbus = {
    .name        = "ehci-sysbus",
    .version_id  = 2,
    .minimum_version_id  = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(ehci, EHCISysBusState, 2, vmstate_ehci, EHCIState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_tegra2_phy,
        NULL
    },
};

static const Property ehci_sysbus_properties[] = {
    DEFINE_EHCI_COMMON_PROPERTIES(EHCISysBusState),
    DEFINE_PROP_BOOL("companion-enable", EHCISysBusState, ehci.companion_enable,
                     false),
};

static void usb_ehci_sysbus_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *d = SYS_BUS_DEVICE(dev);
    EHCISysBusState *i = SYS_BUS_EHCI(dev);
    EHCIState *s = &i->ehci;

    usb_ehci_realize(s, dev, errp);
    sysbus_init_irq(d, &s->irq);
}

static void usb_ehci_sysbus_reset(DeviceState *dev)
{
    SysBusDevice *d = SYS_BUS_DEVICE(dev);
    EHCISysBusState *i = SYS_BUS_EHCI(d);
    EHCIState *s = &i->ehci;

    ehci_reset(s);
}

static void ehci_sysbus_init(Object *obj)
{
    SysBusDevice *d = SYS_BUS_DEVICE(obj);
    EHCISysBusState *i = SYS_BUS_EHCI(obj);
    SysBusEHCIClass *sec = SYS_BUS_EHCI_GET_CLASS(obj);
    EHCIState *s = &i->ehci;

    s->capsbase = sec->capsbase;
    s->opregbase = sec->opregbase;
    s->portscbase = sec->portscbase;
    s->portnr = sec->portnr;
    s->as = &address_space_memory;

    usb_ehci_init(s, DEVICE(obj));
    sysbus_init_mmio(d, &s->mem);
}

static void ehci_sysbus_finalize(Object *obj)
{
    EHCISysBusState *i = SYS_BUS_EHCI(obj);
    EHCIState *s = &i->ehci;

    usb_ehci_finalize(s);
}

static void ehci_sysbus_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(klass);

    sec->portscbase = 0x44;
    sec->portnr = EHCI_PORTS;

    dc->realize = usb_ehci_sysbus_realize;
    dc->vmsd = &vmstate_ehci_sysbus;
    device_class_set_props(dc, ehci_sysbus_properties);
    device_class_set_legacy_reset(dc, usb_ehci_sysbus_reset);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static void ehci_platform_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    sec->capsbase = 0x0;
    sec->opregbase = 0x20;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static void ehci_exynos4210_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    sec->capsbase = 0x0;
    sec->opregbase = 0x10;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static void ehci_aw_h3_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    sec->capsbase = 0x0;
    sec->opregbase = 0x10;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static void ehci_npcm7xx_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    sec->capsbase = 0x0;
    sec->opregbase = 0x10;
    sec->portscbase = 0x44;
    sec->portnr = 1;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static uint64_t tegra2_phy_read(void *opaque, hwaddr addr, unsigned size)
{
    Tegra2EHCIState *s = opaque;
    uint32_t value = s->phy_regs[addr / sizeof(uint32_t)];

    if (addr == 0 && (value & TEGRA2_UTMIP_ENABLE) &&
        !(value & TEGRA2_UTMIP_RESET)) {
        /* The virtual PHY's fixed clock settles immediately after reset. */
        value |= TEGRA2_PHY_CLK_VALID;
    }
    return value;
}

static void tegra2_phy_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    Tegra2EHCIState *s = opaque;

    if (addr == 0) {
        /* USB_PHY_CLK_VALID is a read-only status bit in USB_SUSP_CTRL. */
        value &= ~TEGRA2_PHY_CLK_VALID;
    }
    s->phy_regs[addr / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps tegra2_phy_ops = {
    .read = tegra2_phy_read,
    .write = tegra2_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ehci_tegra2_reset(DeviceState *dev)
{
    Tegra2EHCIState *s = TEGRA2_EHCI(dev);

    usb_ehci_sysbus_reset(dev);
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    s->phy_regs[0] = TEGRA2_UTMIP_ENABLE | TEGRA2_UTMIP_RESET;
}

static void ehci_tegra2_init(Object *obj)
{
    Tegra2EHCIState *s = TEGRA2_EHCI(obj);

    /* UTMI control/tuning registers share the EHCI register window. */
    memory_region_init_io(&s->phy, obj, &tegra2_phy_ops, s,
                          "tegra2-usb-phy", 0xc00);
    memory_region_add_subregion(&s->parent_obj.ehci.mem, TEGRA2_PHY_BASE,
                                &s->phy);
}

static void ehci_tegra2_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    sec->capsbase = 0x100;
    sec->opregbase = 0x140;
    device_class_set_legacy_reset(dc, ehci_tegra2_reset);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static void ehci_ppc4xx_init(Object *o)
{
    EHCISysBusState *s = SYS_BUS_EHCI(o);

    s->ehci.companion_enable = true;
}

static void ehci_ppc4xx_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    sec->capsbase = 0x0;
    sec->opregbase = 0x10;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

/*
 * Faraday FUSBH200 USB 2.0 EHCI
 */

/**
 * FUSBH200EHCIRegs:
 * @FUSBH200_REG_EOF_ASTR: EOF/Async. Sleep Timer Register
 * @FUSBH200_REG_BMCSR: Bus Monitor Control/Status Register
 */
enum FUSBH200EHCIRegs {
    FUSBH200_REG_EOF_ASTR = 0x34,
    FUSBH200_REG_BMCSR    = 0x40,
};

static uint64_t fusbh200_ehci_read(void *opaque, hwaddr addr, unsigned size)
{
    EHCIState *s = opaque;
    hwaddr off = s->opregbase + s->portscbase + 4 * s->portnr + addr;

    switch (off) {
    case FUSBH200_REG_EOF_ASTR:
        return 0x00000041;
    case FUSBH200_REG_BMCSR:
        /* High-Speed, VBUS valid, interrupt level-high active */
        return (2 << 9) | (1 << 8) | (1 << 3);
    }

    return 0;
}

static void fusbh200_ehci_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
}

static const MemoryRegionOps fusbh200_ehci_mmio_ops = {
    .read = fusbh200_ehci_read,
    .write = fusbh200_ehci_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void fusbh200_ehci_init(Object *obj)
{
    EHCISysBusState *i = SYS_BUS_EHCI(obj);
    FUSBH200EHCIState *f = FUSBH200_EHCI(obj);
    EHCIState *s = &i->ehci;

    memory_region_init_io(&f->mem_vendor, OBJECT(f), &fusbh200_ehci_mmio_ops, s,
                          "fusbh200", 0x4c);
    memory_region_add_subregion(&s->mem,
                                s->opregbase + s->portscbase + 4 * s->portnr,
                                &f->mem_vendor);
}

static void fusbh200_ehci_class_init(ObjectClass *oc, const void *data)
{
    SysBusEHCIClass *sec = SYS_BUS_EHCI_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    sec->capsbase = 0x0;
    sec->opregbase = 0x10;
    sec->portscbase = 0x20;
    sec->portnr = 1;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo ehci_sysbus_types[] = {
    {
        .name          = TYPE_SYS_BUS_EHCI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(EHCISysBusState),
        .instance_init = ehci_sysbus_init,
        .instance_finalize = ehci_sysbus_finalize,
        .abstract      = true,
        .class_init    = ehci_sysbus_class_init,
        .class_size    = sizeof(SysBusEHCIClass),
    },
    {
        .name          = TYPE_PLATFORM_EHCI,
        .parent        = TYPE_SYS_BUS_EHCI,
        .class_init    = ehci_platform_class_init,
    },
    {
        .name          = TYPE_EXYNOS4210_EHCI,
        .parent        = TYPE_SYS_BUS_EHCI,
        .class_init    = ehci_exynos4210_class_init,
    },
    {
        .name          = TYPE_AW_H3_EHCI,
        .parent        = TYPE_SYS_BUS_EHCI,
        .class_init    = ehci_aw_h3_class_init,
    },
    {
        .name          = TYPE_NPCM7XX_EHCI,
        .parent        = TYPE_SYS_BUS_EHCI,
        .class_init    = ehci_npcm7xx_class_init,
    },
    {
        .name          = TYPE_TEGRA2_EHCI,
        .parent        = TYPE_SYS_BUS_EHCI,
        .instance_size = sizeof(Tegra2EHCIState),
        .instance_init = ehci_tegra2_init,
        .class_init    = ehci_tegra2_class_init,
    },
    {
        .name          = TYPE_PPC4xx_EHCI,
        .parent        = TYPE_SYS_BUS_EHCI,
        .class_init    = ehci_ppc4xx_class_init,
        .instance_init = ehci_ppc4xx_init,
    },
    {
        .name          = TYPE_FUSBH200_EHCI,
        .parent        = TYPE_SYS_BUS_EHCI,
        .instance_size = sizeof(FUSBH200EHCIState),
        .instance_init = fusbh200_ehci_init,
        .class_init    = fusbh200_ehci_class_init,
    },
};

DEFINE_TYPES(ehci_sysbus_types)
