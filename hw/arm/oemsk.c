/*
 * Experimental OEMSK / MSM8998 board for local Windows driver bring-up.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/arm/virt.h"
#include "hw/arm/qcom8998-scm.h"
#include "hw/arm/qcom8998-rpm.h"
#include "hw/arm/qcom8998-spmi.h"
#include "hw/display/qcom-8998-display.h"
#include "hw/intc/arm_gic.h"
#include "hw/misc/unimp.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/qtest.h"
#include "system/address-spaces.h"
#include "system/tcg.h"
#include "target/arm/cpu.h"

#define TYPE_OEMSK_MACHINE MACHINE_TYPE_NAME("oemsk")
OBJECT_DECLARE_SIMPLE_TYPE(OemskMachineState, OEMSK_MACHINE)

struct OemskMachineState {
    VirtMachineState parent_obj;
    MemoryRegion shared_imem;
};

#define OEMSK_SYNTHETIC_SPI_BASE (512 - GIC_INTERNAL)
#define OEMSK_GIC_IRQS 704

/* Native PEP owns INTIDs through 490. Keep all synthetic IRQs above 511. */
static const int oemsk_irqmap[] = {
    [VIRT_UART0] = OEMSK_SYNTHETIC_SPI_BASE + 1,
    [VIRT_RTC] = OEMSK_SYNTHETIC_SPI_BASE + 2,
    [VIRT_PCIE] = OEMSK_SYNTHETIC_SPI_BASE + 3,
    [VIRT_GPIO] = OEMSK_SYNTHETIC_SPI_BASE + 7,
    [VIRT_UART1] = OEMSK_SYNTHETIC_SPI_BASE + 8,
    [VIRT_ACPI_GED] = OEMSK_SYNTHETIC_SPI_BASE + 9,
    [VIRT_GWDT_WS0] = OEMSK_SYNTHETIC_SPI_BASE + 10,
    [VIRT_EHCI] = OEMSK_SYNTHETIC_SPI_BASE + 11,
    [VIRT_SDHCI] = OEMSK_SYNTHETIC_SPI_BASE + 12,
    [VIRT_WINRT_TIMER] = OEMSK_SYNTHETIC_SPI_BASE + 13,
    [VIRT_MMIO] = OEMSK_SYNTHETIC_SPI_BASE + 16,
    [VIRT_GIC_V2M] = OEMSK_SYNTHETIC_SPI_BASE + 48,
    [VIRT_SMMU] = OEMSK_SYNTHETIC_SPI_BASE + 112,
    [VIRT_PLATFORM_BUS] = OEMSK_SYNTHETIC_SPI_BASE + 128,
};

static void oemsk_validate(VirtMachineState *vms)
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
        error_report("oemsk requires TCG, GICv2, ACPI, secure=off, "
                     "virtualization=off, iommu=none and legacy WinRT "
                     "board options disabled");
        exit(EXIT_FAILURE);
    }
}

static void oemsk_set_memmap(VirtMachineState *vms)
{
    /* SPMI occupies 0x08000000..0x0a7fffff. */
    for (unsigned i = VIRT_CPUPERIPHS; i <= VIRT_GIC_VCPU; i++) {
        vms->memmap[i].base += 0x03000000;
    }
    for (unsigned i = 0; i < VIRT_LOWMEMMAP_LAST; i++) {
        hwaddr address = vms->memmap[i].base;

        if (address >= 0x09000000 && address < 0x0a000000) {
            vms->memmap[i].base = 0x0b100000 + address - 0x09000000;
        }
    }
    vms->memmap[VIRT_MMIO].base = 0x0b300000;
    /* Never offer native MMCC, MDSS, Venus or SMMU space for allocation. */
    vms->memmap[VIRT_PLATFORM_BUS] = (MemMapEntry) {
        0x0b400000, 0x00c00000,
    };
    /* Native RPM IPC and CPU CPRh occupy the old low PCI aperture. */
    vms->memmap[VIRT_PCIE_MMIO] = (MemMapEntry) {
        0x20000000, 0x1eff0000,
    };
}

static void oemsk_create_devices(VirtMachineState *vms)
{
    OemskMachineState *s = OEMSK_MACHINE(vms);

    if (!object_property_get_bool(OBJECT(first_cpu), "aarch64", NULL)) {
        error_report("oemsk requires an AArch64 CPU");
        exit(EXIT_FAILURE);
    }
    /*
     * Native CPU-subsystem space: PEP's SPM_PHYS_CONFIG, ACC, CPU PLLs and
     * OSM. The old unassigned PCI aperture silently supplied RAZ/WI for
     * these unmodeled registers. Preserve that compatibility explicitly,
     * without advertising the space for PCI allocation. Implemented CPRh
     * and RPM IPC regions below override the placeholder at priority 1.
     * This does not simulate power sequencing, PLL lock or OSM operation.
     */
    create_unimplemented_device("oemsk.cpuss-unimplemented", 0x17800000,
                                2 * MiB);
    /* Native MADT GICv3/MSI windows; interrupt delivery still uses GICv2. */
    create_unimplemented_device("oemsk.gicd-unimplemented", 0x17a00000,
                                64 * KiB);
    create_unimplemented_device("oemsk.msi-unimplemented", 0x17a10000,
                                4 * KiB);
    create_unimplemented_device("oemsk.gicr-unimplemented", 0x17b00000, MiB);
    /* SharedIMEM and overlapping TZ/HYP log ranges from the original ACPI. */
    memory_region_init_ram_flags_nomigrate(&s->shared_imem, OBJECT(vms),
                                           "oemsk.shared-imem", 12 * KiB, 0,
                                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x146bf000,
                                &s->shared_imem);
    qcom8998_display_create(vms->gic);
    qcom8998_rpm_create(vms->gic);
    qcom8998_spmi_create();
    qcom8998_scm_create();
}

static void oemsk_build_dsdt(Aml *scope, VirtMachineState *vms)
{
    qcom8998_display_acpi(scope);
}

static void oemsk_instance_init(Object *obj)
{
    VirtMachineState *vms = VIRT_MACHINE(obj);

    vms->gic_version = VIRT_GIC_VERSION_2;
    vms->acpi = ON_OFF_AUTO_ON;
    vms->irqmap = oemsk_irqmap;
    object_property_add_alias(obj, "board-ssdt", obj, "winrt-qcdx-ssdt");
}

static void oemsk_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    VirtMachineClass *vmc = VIRT_MACHINE_CLASS(oc);

    mc->desc = "Experimental OEMSK MSM8998 board (virtual firmware boot)";
    mc->get_default_cpu_type = NULL;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("max");
    mc->max_cpus = 8;
    mc->default_ram_size = 4 * GiB;
    mc->default_ram_id = "oemsk.ram";
    vmc->validate_board = oemsk_validate;
    vmc->set_board_memmap = oemsk_set_memmap;
    vmc->create_board_devices = oemsk_create_devices;
    vmc->build_board_dsdt = oemsk_build_dsdt;
    vmc->firmware_smc = qcom8998_smc;
    vmc->gic_spi_count = OEMSK_GIC_IRQS - GIC_INTERNAL;
    vmc->variable_flash_size = 16 * MiB;
}

static const TypeInfo oemsk_machine_info = {
    .name = TYPE_OEMSK_MACHINE,
    .parent = TYPE_VIRT_MACHINE,
    .instance_size = sizeof(OemskMachineState),
    .class_init = oemsk_class_init,
    .instance_init = oemsk_instance_init,
    .is_available = target_aarch64,
};

static void oemsk_register_types(void)
{
    type_register_static(&oemsk_machine_info);
}
type_init(oemsk_register_types)
