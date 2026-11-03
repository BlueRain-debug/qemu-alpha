/* Local MSM8x60 graphics bring-up instrumentation, not a complete GPU model. */
/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_QCOM_MDP4_PROBE_H
#define HW_QCOM_MDP4_PROBE_H

#define TYPE_QCOM_MDP4_PROBE "qcom-mdp4-probe"

/* Native resource order is significant to the vendor miniport. */
#define QCOM_MDP4_PROBE_BANKS 18
#define QCOM_MDP4_PROBE_IRQS 15
#define QCOM_MDP4_VIDEO_BASE 0x40300000ULL
#define QCOM_MDP4_VIDEO_SIZE 0x01800000ULL
#define QCOM_MDP4_VRAM_BASE 0x38000000ULL
#define QCOM_MDP4_VRAM_SIZE 0x03938700ULL
/* Native MMCC at 0x04000000 conflicts with virt's usual variable flash bank. */
#define QCOM_MDP4_NVRAM_BASE 0xf8000000ULL

static const struct {
    uint64_t base;
    uint64_t size;
} qcom_mdp4_probe_banks[QCOM_MDP4_PROBE_BANKS] = {
    { 0x05100000, 0x100000 }, /* MDP4 */
    { 0x04700000, 0x001000 },
    { 0x04a00000, 0x001000 },
    { 0x04f00000, 0x002000 },
    { 0x04300000, 0x020000 }, /* Adreno register window */
    { 0x04e00000, 0x010000 },
    { 0x04400000, 0x100000 },
    { 0x07300000, 0x100000 }, /* Ten native SMMU resource windows. */
    { 0x07400000, 0x100000 },
    { 0x07500000, 0x100000 },
    { 0x07600000, 0x100000 },
    { 0x07700000, 0x100000 },
    { 0x07800000, 0x100000 },
    { 0x07900000, 0x100000 },
    { 0x07a00000, 0x100000 },
    { 0x07b00000, 0x100000 },
    { 0x07c00000, 0x100000 },
    { 0x04000000, 0x040000 }, /* Hard-coded MMCC mapping in the miniport. */
};

/* ACPI GSIs, not SPI indices. Order: five GPU resources, ten SMMU resources. */
static const unsigned qcom_mdp4_probe_gsis[QCOM_MDP4_PROBE_IRQS] = {
    107, 111, 112, 105, 81, 98, 84, 96, 94, 92, 100, 86, 90, 88, 102,
};

#endif
