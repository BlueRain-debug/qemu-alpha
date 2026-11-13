/*
 * MSM8998 display bring-up for the Windows QCDX miniport.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/acpi/aml-build.h"
#include "hw/core/cpu.h"
#include "hw/core/sysbus.h"
#include "hw/display/qcom-8998-display.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "trace.h"

#define TYPE_QCOM8998_DISPLAY "qcom-8998-display"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom8998Display, QCOM8998_DISPLAY)

enum {
    BANK_MDSS,
    BANK_ADRENO,
    BANK_VIDEO,
    BANK_VIDEO_VMEM,
    BANK_MDSS_SE,
    BANK_MDSS_SMMU,
    BANK_ADRENO_SMMU,
    BANK_SOC_JTAG_ID,
    BANK_SOC_VERSION,
    BANK_COUNT,
};

enum {
    MDSS_GSIV = 115,
    ADRENO_GSIV = 332,
    VIDEO_GSIV = 319,
    GIC_INTERNAL_IRQS = 32,
};

/* Display resource lengths and ordering are from the OEMSK PnP BootConfig. */
static const struct {
    const char *name;
    uint32_t base;
    uint32_t size;
} banks[BANK_COUNT] = {
    [BANK_MDSS] = { "mdss", 0x0c900000, 0xba008 },
    [BANK_ADRENO] = { "adreno", 0x05000000, 0x3f000 },
    [BANK_VIDEO] = { "venus", 0x0cc00000, 0x100000 },
    [BANK_VIDEO_VMEM] = { "venus-vmem", 0x0c880000, 0x800 },
    [BANK_MDSS_SE] = { "mdss-se", 0x0c9e0000, 0x2c },
    [BANK_MDSS_SMMU] = { "mdss-smmu", 0x0cd00000, 0x40000 },
    [BANK_ADRENO_SMMU] = { "adreno-smmu", 0x05040000, 0xe000 },
    /*
     * The SMMU driver reads these physical chip-info registers before it
     * maps its ACPI resources. Map just the words within the virt code flash.
     */
    [BANK_SOC_JTAG_ID] = { "soc-jtag-id", 0x03998010, 4 },
    [BANK_SOC_VERSION] = { "soc-version", 0x01fc8000, 4 },
};

#define MDSS_MDP_HW_VERSION 0x1000
#define MSM8998_MDP_REV_301 0x30000001
#define APQ8098_JTAG_ID 0x000620e1
#define APQ8098_SOC_HW_VERSION_2_0 0x30020200
#define SMMU_IDR0 0x20
#define SMMU_IDR1 0x24
#define SMMU_IDR2 0x28
#define SMMU_IDR0_STAGE1 (1U << 30)
#define SMMU_IDR0_STREAM_MATCH (1U << 27)
#define SMMU_IDR0_CONTEXT_IRQS(n) ((n) << 16)
#define SMMU_IDR0_STREAM_ID_BITS(n) ((n) << 9)
#define SMMU_IDR1_PAGE_INDEX_BITS(n) ((n) << 28)
#define SMMU_IDR2_4K_PAGES (1U << 12)
#define SMMU_IDR2_VA_48BIT (5U << 8)
#define SMMU_IDR2_PA_40BIT (2U << 4)
#define SMMU_IDR2_IPA_40BIT 2U

typedef struct Qcom8998Bank {
    MemoryRegion region;
    unsigned index;
    uint32_t *regs;
    hwaddr last_read_offset;
    uint32_t last_read_value;
} Qcom8998Bank;

struct Qcom8998Display {
    SysBusDevice parent_obj;
    Qcom8998Bank bank[BANK_COUNT];
    qemu_irq irq[3];
};

static uint64_t qcom8998_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom8998Bank *bank = opaque;
    uint32_t value = bank->regs[offset / sizeof(value)];

    /* Repeated polls should not dominate the bring-up trace. */
    if (offset != bank->last_read_offset || value != bank->last_read_value) {
        trace_qcom8998_mmio_read(banks[bank->index].name, offset, value,
                                current_cpu ?
                                current_cpu->cc->get_pc(current_cpu) : 0);
        bank->last_read_offset = offset;
        bank->last_read_value = value;
    }
    return value;
}

static void qcom8998_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    Qcom8998Bank *bank = opaque;
    bool smmu = bank->index == BANK_MDSS_SMMU ||
                bank->index == BANK_ADRENO_SMMU;

    trace_qcom8998_mmio_write(banks[bank->index].name, offset, value,
                             current_cpu ?
                             current_cpu->cc->get_pc(current_cpu) : 0);
    if (bank->index == BANK_SOC_JTAG_ID || bank->index == BANK_SOC_VERSION ||
        (smmu && offset >= SMMU_IDR0 && offset <= SMMU_IDR2) ||
        (bank->index == BANK_MDSS && offset == MDSS_MDP_HW_VERSION)) {
        return;
    }
    bank->regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps qcom8998_mmio_ops = {
    .read = qcom8998_read,
    .write = qcom8998_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void qcom8998_smmu_reset(Qcom8998Bank *bank, unsigned stream_entries,
                               unsigned page_index_bits, unsigned contexts)
{
    /*
     * Advertise the layout used by QCSMMU's stream and context-bank setup.
     * The probe models the registers only. It has no DMA translation yet.
     */
    bank->regs[SMMU_IDR0 / 4] = SMMU_IDR0_STAGE1 | SMMU_IDR0_STREAM_MATCH |
        SMMU_IDR0_CONTEXT_IRQS(1U) | SMMU_IDR0_STREAM_ID_BITS(15U) |
        stream_entries;
    bank->regs[SMMU_IDR1 / 4] = SMMU_IDR1_PAGE_INDEX_BITS(page_index_bits) |
        contexts;
    bank->regs[SMMU_IDR2 / 4] = SMMU_IDR2_4K_PAGES | SMMU_IDR2_VA_48BIT |
        SMMU_IDR2_PA_40BIT | SMMU_IDR2_IPA_40BIT;
}

static void qcom8998_reset(DeviceState *dev)
{
    Qcom8998Display *s = QCOM8998_DISPLAY(dev);

    for (unsigned i = 0; i < BANK_COUNT; i++) {
        memset(s->bank[i].regs, 0, banks[i].size);
        s->bank[i].last_read_offset = HWADDR_MAX;
    }
    s->bank[BANK_MDSS].regs[MDSS_MDP_HW_VERSION / 4] = MSM8998_MDP_REV_301;
    s->bank[BANK_SOC_JTAG_ID].regs[0] = APQ8098_JTAG_ID;
    s->bank[BANK_SOC_VERSION].regs[0] = APQ8098_SOC_HW_VERSION_2_0;
    /* Context banks start at 0x20000 for MDSS and 0x8000 for Adreno. */
    qcom8998_smmu_reset(&s->bank[BANK_MDSS_SMMU], 60, 4, 16);
    qcom8998_smmu_reset(&s->bank[BANK_ADRENO_SMMU], 16, 2, 2);
}

static void qcom8998_init(Object *obj)
{
    Qcom8998Display *s = QCOM8998_DISPLAY(obj);
    SysBusDevice *bus = SYS_BUS_DEVICE(obj);

    for (unsigned i = 0; i < BANK_COUNT; i++) {
        Qcom8998Bank *bank = &s->bank[i];
        g_autofree char *name = g_strdup_printf("qcom8998.%s", banks[i].name);

        bank->index = i;
        bank->regs = g_malloc0(banks[i].size);
        memory_region_init_io(&bank->region, obj, &qcom8998_mmio_ops, bank,
                              name, banks[i].size);
        sysbus_init_mmio(bus, &bank->region);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->irq); i++) {
        sysbus_init_irq(bus, &s->irq[i]);
    }
}

static void qcom8998_finalize(Object *obj)
{
    Qcom8998Display *s = QCOM8998_DISPLAY(obj);

    for (unsigned i = 0; i < BANK_COUNT; i++) {
        g_free(s->bank[i].regs);
    }
}

static const VMStateDescription qcom8998_vmstate = {
    .name = TYPE_QCOM8998_DISPLAY,
    .unmigratable = 1,
};

static void qcom8998_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, qcom8998_reset);
    dc->desc = "Experimental MSM8998 QCDX MMIO bring-up";
    dc->vmsd = &qcom8998_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo qcom8998_type = {
    .name = TYPE_QCOM8998_DISPLAY,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom8998Display),
    .instance_init = qcom8998_init,
    .instance_finalize = qcom8998_finalize,
    .class_init = qcom8998_class_init,
};

static void qcom8998_register_types(void)
{
    type_register_static(&qcom8998_type);
}
type_init(qcom8998_register_types)

void qcom8998_display_create(DeviceState *gic)
{
    static const uint32_t gsiv[] = { MDSS_GSIV, ADRENO_GSIV, VIDEO_GSIV };
    SysBusDevice *bus = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM8998_DISPLAY));

    DEVICE(bus)->id = g_strdup("qcdx8998");
    sysbus_realize_and_unref(bus, &error_fatal);
    for (unsigned i = 0; i < BANK_COUNT; i++) {
        sysbus_mmio_map_overlap(bus, i, banks[i].base, 1);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(gsiv); i++) {
        sysbus_connect_irq(bus, i, qdev_get_gpio_in(gic,
                           gsiv[i] - GIC_INTERNAL_IRQS));
    }
}

static void qcom8998_acpi_mem(Aml *crs, unsigned bank)
{
    aml_append(crs, aml_memory32_fixed(banks[bank].base, banks[bank].size,
                                       AML_READ_WRITE));
}

static void qcom8998_acpi_irq(Aml *crs, uint32_t gsiv, AmlLevelAndEdge sense)
{
    aml_append(crs, aml_interrupt(AML_CONSUMER, sense, AML_ACTIVE_HIGH,
                                  AML_EXCLUSIVE, &gsiv, 1));
}

static Aml *qcom8998_acpi_device(const char *name, const char *hid, unsigned uid)
{
    Aml *dev = aml_device("%s", name);

    aml_append(dev, aml_name_decl("_HID", aml_string("%s", hid)));
    aml_append(dev, aml_name_decl("_SUB", aml_string("MTP08998")));
    aml_append(dev, aml_name_decl("_UID", aml_int(uid)));
    return dev;
}

static void qcom8998_acpi_smmu(Aml *scope, const char *name, unsigned uid,
                               unsigned bank, const uint32_t *irqs,
                               size_t count)
{
    Aml *dev = qcom8998_acpi_device(name, "QCOM0011", uid);
    Aml *crs = aml_resource_template();

    qcom8998_acpi_mem(crs, bank);
    for (size_t i = 0; i < count; i++) {
        qcom8998_acpi_irq(crs, irqs[i], AML_EDGE);
    }
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(scope, dev);
}

void qcom8998_display_acpi(Aml *scope)
{
    /*
     * RESI labels the descriptors in _CRS by position. The labels follow the
     * MSM8998 Windows ACPI ABI used by the original miniport.
     */
    static const struct {
        const char *name;
        const char *engine;
        unsigned bank;
        uint32_t gsiv;
    } resources[] = {
        { "MDP_REGS", "DISPLAY", BANK_MDSS },
        { "VSYNC_INTERRUPT", "DISPLAY", 0, MDSS_GSIV },
        { "GFX_REGS", "GRAPHICS", BANK_ADRENO },
        { "GFX_INTERRUPT", "GRAPHICS", 0, ADRENO_GSIV },
        { "VIDEO_REGS", "VIDEO", BANK_VIDEO },
        { "VIDEO_VMEM_REGS", "VIDEO", BANK_VIDEO_VMEM },
        { "VIDC_INTERRUPT", "VIDEO", 0, VIDEO_GSIV },
        { "SE_CTRL_REGS", "DISPLAY", BANK_MDSS_SE },
    };
    static const uint32_t mdss_smmu_irqs[] = {
        296, 295, 298, 299, 300, 276, 277, 279, 280,
        281, 282, 283, 284, 285, 286, 287, 288,
    };
    static const uint32_t adreno_smmu_irqs[] = { 366, 361, 362 };
    /* Scalar memory and scheduling capabilities from the OEMSK board REGR. */
    static const struct {
        const char *name;
        uint32_t value;
    } adapter_settings[] = {
        { "DisableTzMDSSRestore", 1 },
        { "UseLowPTForGfxPerProcess", 1 },
        { "GPU64bAddrEnabled", 1 },
        { "MaxPreemptionOffsets", 0x80 },
        { "MaxRequiredDmaQueueEntry", 8 },
        { "SupportsSecureInAperture", 1 },
        { "ZeroFlagSupportInPTE", 1 },
        { "SupportsCacheCoherency", 1 },
    };
    Aml *dev = qcom8998_acpi_device("QGFX", "QCOM007C", 0);
    Aml *cls = aml_package(3);
    Aml *crs = aml_resource_template();
    Aml *resi = aml_method("RESI", 0, AML_NOTSERIALIZED);
    Aml *resource_info = aml_package(2 + ARRAY_SIZE(resources));
    Aml *regr = aml_method("REGR", 0, AML_NOTSERIALIZED);
    Aml *settings = aml_package(ARRAY_SIZE(adapter_settings) + 1);
    Aml *graphics_settings = aml_package(2);
    Aml *dcvs_settings = aml_package(2);
    Aml *dcvs_enable = aml_package(2);

    aml_append(cls, aml_int(3));
    aml_append(cls, aml_int(0));
    aml_append(cls, aml_int(0));
    aml_append(dev, aml_name_decl("_CLS", cls));
    aml_append(dev, aml_name_decl("_CID", aml_string("ACPIQCOM007C")));
    aml_append(dev, aml_name_decl("_HRV", aml_int(0x7c)));
    aml_append(resource_info, aml_int(3));
    aml_append(resource_info, aml_int(0));
    for (unsigned i = 0; i < ARRAY_SIZE(resources); i++) {
        Aml *entry = aml_package(3);

        if (resources[i].gsiv) {
            qcom8998_acpi_irq(crs, resources[i].gsiv, AML_LEVEL);
        } else {
            qcom8998_acpi_mem(crs, resources[i].bank);
        }
        aml_append(entry, aml_string("RESOURCE"));
        aml_append(entry, aml_string("%s", resources[i].name));
        aml_append(entry, aml_string("%s", resources[i].engine));
        aml_append(resource_info, entry);
    }
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(resi, aml_return(resource_info));
    aml_append(dev, resi);

    for (unsigned i = 0; i < ARRAY_SIZE(adapter_settings); i++) {
        Aml *entry = aml_package(2);

        aml_append(entry, aml_string("%s", adapter_settings[i].name));
        aml_append(entry, aml_int(adapter_settings[i].value));
        aml_append(settings, entry);
    }
    /*
     * Retain the board's GRAPHICS/DCVS section. QCDX's cached REGR lookup
     * requires a subkey list before it will read even the root values.
     */
    aml_append(dcvs_enable, aml_string("Enable"));
    aml_append(dcvs_enable, aml_int(1));
    aml_append(dcvs_settings, aml_string("DCVS"));
    aml_append(dcvs_settings, dcvs_enable);
    aml_append(graphics_settings, aml_string("GRAPHICS"));
    aml_append(graphics_settings, dcvs_settings);
    aml_append(settings, graphics_settings);
    aml_append(regr, aml_return(settings));
    aml_append(dev, regr);

    qcom8998_acpi_smmu(scope, "QSM0", 0, BANK_MDSS_SMMU, mdss_smmu_irqs,
                     ARRAY_SIZE(mdss_smmu_irqs));
    qcom8998_acpi_smmu(scope, "QSM3", 3, BANK_ADRENO_SMMU, adreno_smmu_irqs,
                     ARRAY_SIZE(adreno_smmu_irqs));
    aml_append(scope, dev);
}
