/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_DISPLAY_QCOM_8998_DISPLAY_H
#define HW_DISPLAY_QCOM_8998_DISPLAY_H

/* Keep native PEP/GPU INTIDs below 512, including the PEP's INTID 490. */
#define QCOM8998_VIRT_IRQ_BASE 512
/* Synthetic virt devices use the following 192 INTIDs, through INTID 703. */
#define QCOM8998_GIC_IRQS (QCOM8998_VIRT_IRQ_BASE + 192)
/* At 0x04000000 this ends at the native Adreno window, 0x05000000. */
#define QCOM8998_VAR_FLASH_SIZE 0x01000000

void qcom8998_display_create(DeviceState *gic);
void qcom8998_display_acpi(Aml *scope);

#endif
