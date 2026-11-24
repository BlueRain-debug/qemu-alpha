/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_DISPLAY_QCOM_845_DISPLAY_H
#define HW_DISPLAY_QCOM_845_DISPLAY_H

#include "hw/arm/oemsk850.h"

#define TYPE_QCOM845_DISPLAY "qcom845-display"

void qcom845_display_create(DeviceState *gic, Qcom845Smmu *apps_smmu,
                            Qcom845Smmu *gpu_smmu);

#endif
