/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_ARM_OEMSK850_H
#define HW_ARM_OEMSK850_H

#include "target/arm/cpu-qom.h"
#include "hw/core/qdev.h"
#include "exec/hwaddr.h"

/* Native interrupt IDs are kept below the synthetic board's INTID 576. */
#define OEMSK850_SYNTHETIC_INTID 576
#define OEMSK850_GIC_IRQS 768
#define OEMSK850_FLASH_BASE 0x30000000
#define QCOM845_SPMI_BASE 0x0c400000
#define QCOM845_SPMI_SIZE 0x02800000

bool qcom845_smc(ARMCPU *cpu);
void qcom845_scm_create(void);
void qcom845_spmi_create(void);
void qcom845_rpmh_create(DeviceState *gic);
void qcom845_tsens_create(DeviceState *gic);
void qcom845_aoss_create(DeviceState *gic);
void qcom845_soc_create(void);
void qcom845_cprh_create(void);
void qcom845_cmddb_create(const char *file);
void qcom845_gcc_create(void);
void qcom845_osm_create(void);
void qcom845_noc_create(void);
void qcom845_dispcc_create(void);
void qcom845_tlmm_create(void);
void qcom845_mmcc_create(void);
typedef struct Qcom845Smmu Qcom845Smmu;
Qcom845Smmu *qcom845_smmu_create(DeviceState *gic, hwaddr base,
                                hwaddr size, unsigned contexts,
                                unsigned streams, unsigned global_intid,
                                unsigned num_irqs,
                                const unsigned *context_intids);
bool qcom845_smmu_translate(Qcom845Smmu *s, uint16_t sid, uint64_t iova,
                            bool write, hwaddr *physical);

#endif
