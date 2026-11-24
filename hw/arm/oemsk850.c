/*
 * Experimental SDM845 OEMSK board for local FactoryOS bring-up.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Virtual firmware and boot I/O use the common ArmVirt implementation.
 * Native SDM845 devices belong to this board, never to MSM8998's model.
 */
#include "qemu/osdep.h"
#include "hw/acpi/aml-build.h"
#include "hw/arm/virt.h"
#include "hw/arm/oemsk850.h"
#include "hw/intc/arm_gic.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/qtest.h"
#include "system/tcg.h"
#include "system/address-spaces.h"
#include "target/arm/cpu.h"

#define TYPE_OEMSK850_MACHINE MACHINE_TYPE_NAME("oemsk850")
OBJECT_DECLARE_SIMPLE_TYPE(Oemsk850MachineState, OEMSK850_MACHINE)

struct Oemsk850MachineState {
    VirtMachineState parent_obj;
    char *cmd_db;
    MemoryRegion native_gicd;
    MemoryRegion native_gicd_id;
};

#define SPI_BASE (OEMSK850_SYNTHETIC_INTID - GIC_INTERNAL)

static const int oemsk850_irqmap[] = {
    [VIRT_UART0] = SPI_BASE + 1,
    [VIRT_RTC] = SPI_BASE + 2,
    [VIRT_PCIE] = SPI_BASE + 3,
    [VIRT_GPIO] = SPI_BASE + 7,
    [VIRT_UART1] = SPI_BASE + 8,
    [VIRT_ACPI_GED] = SPI_BASE + 9,
    [VIRT_GWDT_WS0] = SPI_BASE + 10,
    [VIRT_EHCI] = SPI_BASE + 11,
    [VIRT_SDHCI] = SPI_BASE + 12,
    [VIRT_WINRT_TIMER] = SPI_BASE + 13,
    [VIRT_MMIO] = SPI_BASE + 16,
    [VIRT_GIC_V2M] = SPI_BASE + 48,
    [VIRT_SMMU] = SPI_BASE + 112,
    [VIRT_PLATFORM_BUS] = SPI_BASE + 128,
};

static void oemsk850_validate(VirtMachineState *vms)
{
    if ((!tcg_enabled() && !qtest_enabled()) ||
        vms->gic_version != VIRT_GIC_VERSION_2 ||
        !virt_is_acpi_enabled(vms) || vms->secure || vms->virt ||
        vms->winrt || vms->winrt_legacy_ram || vms->winrt_tegra2 ||
        vms->winrt_sp804 || vms->winrt_tegra_timer || vms->winrt_qcom_timer ||
        vms->winrt_omap_timer || vms->winrt_omap_dma ||
        vms->winrt_qfprom_probe || vms->winrt_qcdx_probe ||
        vms->winrt_qcom_cypress_touch || vms->winrt_qcom_synaptics_touch ||
        vms->winrt_pci || vms->iommu != VIRT_IOMMU_NONE) {
        error_report("oemsk850 requires TCG, GICv2, ACPI, secure=off, "
                     "virtualization=off, iommu=none and legacy WinRT "
                     "board options disabled");
        exit(EXIT_FAILURE);
    }
}

static void oemsk850_set_memmap(VirtMachineState *vms)
{
    /*
     * The boot firmware maps device memory at 0x08000000..0x0fffffff.
     * Fixed synthetic peripherals use gaps at 0x0b00/0x0b10/0x0b20 below
     * the native PDC at 0x0b280000. Allocatable platform/PCI windows must
     * remain outside the native 0x05000000..0x17ffffff address range.
     */
    for (unsigned i = 0; i < VIRT_LOWMEMMAP_LAST; i++) {
        hwaddr address = vms->memmap[i].base;

        if (address >= 0x08000000 && address < 0x09000000) {
            vms->memmap[i].base = address + 0x03000000;
        } else if (address >= 0x09000000 && address < 0x0a000000) {
            vms->memmap[i].base = address + 0x02100000;
        }
    }
    vms->memmap[VIRT_MMIO].base = 0x0b200000;
    /* Keep boot code out of native GCC/QFPROM/chip-info MMIO. */
    vms->memmap[VIRT_FLASH].base = OEMSK850_FLASH_BASE;
    vms->memmap[VIRT_SECURE_MEM].base = 0x1c000000;
    vms->memmap[VIRT_GWDT_REFRESH].base = 0x1d000000;
    vms->memmap[VIRT_GWDT_CONTROL].base = 0x1d001000;
    vms->memmap[VIRT_PLATFORM_BUS] = (MemMapEntry) {
        0x1b000000, 0x01000000,
    };
    vms->memmap[VIRT_PCIE_MMIO] = (MemMapEntry) {
        0x20000000, 0x0f000000,
    };
}

static void oemsk850_create_devices(VirtMachineState *vms)
{
    Oemsk850MachineState *s = OEMSK850_MACHINE(vms);
    CPUState *cs;
    static const unsigned apps_irqs[] = {
        128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
        140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150,
        213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223,
    };
    static const unsigned gpu_irqs[] = {
        396, 397, 398, 399, 400, 401, 402, 403,
    };

    if (!object_property_get_bool(OBJECT(first_cpu), "aarch64", NULL)) {
        error_report("oemsk850 requires an AArch64 CPU");
        exit(EXIT_FAILURE);
    }
    CPU_FOREACH(cs) {
        object_property_set_uint(OBJECT(cs), "rvbar", OEMSK850_FLASH_BASE,
                                 &error_fatal);
    }
    qcom845_scm_create();
    qcom845_soc_create();
    qcom845_noc_create();
    qcom845_dispcc_create();
    qcom845_tlmm_create();
    qcom845_mmcc_create();
    qcom845_cprh_create();
    qcom845_gcc_create();
    qcom845_osm_create();
    qcom845_cmddb_create(OEMSK850_MACHINE(vms)->cmd_db);
    /*
     * PEP accesses legacy distributor enable/pending registers through its
     * compiled-in native address. These offsets have the same meaning on
     * our GICv2, so alias that actual state rather than dropping IRQ writes.
     * This does not provide GICv3 redistributors or system-register access.
     */
    memory_region_init_alias(&s->native_gicd, OBJECT(s),
                              "oemsk850.native-gicd-legacy",
                              sysbus_mmio_get_region(SYS_BUS_DEVICE(vms->gic),
                                                     0), 0, 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x17a00000,
                                &s->native_gicd);
    /* PEP probes the native 64 KiB frame's ID words. Report our real GICv2. */
    memory_region_init_alias(&s->native_gicd_id, OBJECT(s),
                              "oemsk850.native-gicd-id",
                              sysbus_mmio_get_region(SYS_BUS_DEVICE(vms->gic),
                                                     0), 0xfd0, 0x30);
    memory_region_add_subregion(get_system_memory(), 0x17a0ffd0,
                                &s->native_gicd_id);
    qcom845_spmi_create();
    qcom845_rpmh_create(vms->gic);
    qcom845_tsens_create(vms->gic);
    qcom845_aoss_create(vms->gic);
    qcom845_smmu_create(vms->gic, 0x15000000, 0x80000,
                        ARRAY_SIZE(apps_irqs), 128, 97, apps_irqs);
    qcom845_smmu_create(vms->gic, 0x05040000, 0x10000,
                        ARRAY_SIZE(gpu_irqs), 16, 261, gpu_irqs);
}

static Aml *oemsk850_acpi_device(const char *name, const char *hid,
                               unsigned uid)
{
    Aml *dev = aml_device("%s", name);

    aml_append(dev, aml_name_decl("_HID", aml_string("%s", hid)));
    aml_append(dev, aml_name_decl("_SUB", aml_string("MTP00845")));
    aml_append(dev, aml_name_decl("_UID", aml_int(uid)));
    return dev;
}

static void oemsk850_build_dsdt(Aml *scope, VirtMachineState *vms)
{
    Aml *dev, *crs, *conf;
    /* Native CONF from 17047's SPMI controller descriptor. */
    static uint8_t spmi_conf[] = {
        0x00, 0x01, 0x01, 0x01, 0xff, 0x00, 0x02, 0x00,
        0x0a, 0x07, 0x04, 0x07, 0x01, 0xff, 0x10, 0x01,
        0x00, 0x01, 0x0c, 0x40, 0x00, 0x00, 0x02, 0x80,
        0x00, 0x00,
    };

    /* Chip Ver 2.1 is repeated in this image's original LogFS UEFI logs. */
    aml_append(scope, aml_name_decl("PSUB", aml_string("MTP00845")));
    aml_append(scope, aml_name_decl("SIDV", aml_int(0x00020001)));
    aml_append(scope, oemsk850_acpi_device("SCM0", "QCOM0214", 0));
    aml_append(scope, oemsk850_acpi_device("TREE", "QCOM02BB", 0));
    dev = oemsk850_acpi_device("SPMI", "QCOM0216", 1);
    aml_append(dev, aml_name_decl("_CCA", aml_int(0)));
    crs = aml_resource_template();
    aml_append(crs, aml_memory32_fixed(QCOM845_SPMI_BASE,
                                      QCOM845_SPMI_SIZE, AML_READ_WRITE));
    aml_append(dev, aml_name_decl("_CRS", crs));
    conf = aml_method("CONF", 0, AML_NOTSERIALIZED);
    aml_append(conf, aml_return(aml_buffer(sizeof(spmi_conf), spmi_conf)));
    aml_append(dev, conf);
    aml_append(scope, dev);

    dev = oemsk850_acpi_device("PMIC", "QCOM0266", 0);
    crs = aml_package(1);
    aml_append(crs, aml_name("\\_SB.SPMI"));
    aml_append(dev, aml_name_decl("_DEP", crs));
    conf = aml_method("PMCF", 0, AML_NOTSERIALIZED);
    crs = aml_package(4);
    aml_append(crs, aml_int(3));
    for (unsigned i = 0; i < 3; i++) {
        Aml *pair = aml_package(2);

        aml_append(pair, aml_int(i * 2));
        aml_append(pair, aml_int(i * 2 + 1));
        aml_append(crs, pair);
    }
    aml_append(conf, aml_return(crs));
    aml_append(dev, conf);
    aml_append(scope, dev);
}

static char *oemsk850_get_cmd_db(Object *obj, Error **errp)
{
    const char *path = OEMSK850_MACHINE(obj)->cmd_db;

    return g_strdup(path ? path : "");
}

static void oemsk850_set_cmd_db(Object *obj, const char *value, Error **errp)
{
    Oemsk850MachineState *s = OEMSK850_MACHINE(obj);

    g_free(s->cmd_db);
    s->cmd_db = g_strdup(value);
}

static void oemsk850_finalize(Object *obj)
{
    g_free(OEMSK850_MACHINE(obj)->cmd_db);
}

static void oemsk850_instance_init(Object *obj)
{
    VirtMachineState *vms = VIRT_MACHINE(obj);

    vms->gic_version = VIRT_GIC_VERSION_2;
    vms->acpi = ON_OFF_AUTO_ON;
    vms->irqmap = oemsk850_irqmap;
    object_property_add_alias(obj, "board-ssdt", obj, "winrt-qcdx-ssdt");
    object_property_add_str(obj, "cmd-db", oemsk850_get_cmd_db,
                            oemsk850_set_cmd_db);
    object_property_set_description(obj, "cmd-db",
                                   "External read-only firmware Command DB");
}

static void oemsk850_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    VirtMachineClass *vmc = VIRT_MACHINE_CLASS(oc);

    mc->desc = "Experimental OEMSK SDM845 board (virtual firmware boot)";
    mc->get_default_cpu_type = NULL;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("max");
    mc->max_cpus = 8;
    mc->default_ram_size = 4 * GiB;
    mc->default_ram_id = "oemsk850.ram";
    vmc->validate_board = oemsk850_validate;
    vmc->set_board_memmap = oemsk850_set_memmap;
    vmc->create_board_devices = oemsk850_create_devices;
    vmc->build_board_dsdt = oemsk850_build_dsdt;
    vmc->firmware_smc = qcom845_smc;
    vmc->gic_spi_count = OEMSK850_GIC_IRQS - GIC_INTERNAL;
    vmc->variable_flash_size = 16 * MiB;
}

static const TypeInfo oemsk850_machine_info = {
    .name = TYPE_OEMSK850_MACHINE,
    .parent = TYPE_VIRT_MACHINE,
    .instance_size = sizeof(Oemsk850MachineState),
    .class_init = oemsk850_class_init,
    .instance_init = oemsk850_instance_init,
    .instance_finalize = oemsk850_finalize,
    .is_available = target_aarch64,
};

static void oemsk850_register_types(void)
{
    type_register_static(&oemsk850_machine_info);
}
type_init(oemsk850_register_types)
