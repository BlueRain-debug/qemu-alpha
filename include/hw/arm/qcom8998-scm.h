/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_QCOM8998_SCM_H
#define HW_ARM_QCOM8998_SCM_H

#include "target/arm/cpu-qom.h"

bool qcom8998_smc(ARMCPU *cpu);
void qcom8998_scm_create(void);

#endif
