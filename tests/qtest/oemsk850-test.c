/* SDM845 board separation and SPMI v5 regression tests. */
/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"
#include "qobject/qdict.h"
#include "qobject/qlist.h"

#define SPMI 0x0c400000
#define OBS (SPMI + 0x02200000)
#define CHANNEL(n) (OBS + (n) * 0x80)

static bool has_device(QTestState *qts, const char *type)
{
    g_autoptr(QDict) reply = qtest_qmp(qts,
        "{'execute':'qom-list','arguments':{'path':'/machine/unattached'}}");
    QList *list = qdict_get_qlist(reply, "return");
    QListEntry *entry;

    QLIST_FOREACH_ENTRY(list, entry) {
        QDict *item = qobject_to(QDict, qlist_entry_obj(entry));

        if (!strcmp(qdict_get_str(item, "type"), type)) {
            return true;
        }
    }
    return false;
}

static QTestState *start_board(void)
{
    return qtest_init("-M oemsk850 -m 256M -nodefaults");
}

static void intercept_gic(QTestState *qts)
{
    g_autoptr(QDict) reply = qtest_qmp(qts,
        "{'execute':'qom-list','arguments':{'path':'/machine/unattached'}}");
    QList *list = qdict_get_qlist(reply, "return");
    QListEntry *entry;

    QLIST_FOREACH_ENTRY(list, entry) {
        QDict *item = qobject_to(QDict, qlist_entry_obj(entry));

        if (!strcmp(qdict_get_str(item, "type"), "child<arm_gic>")) {
            g_autofree char *path = g_strdup_printf("/machine/unattached/%s",
                                                   qdict_get_str(item, "name"));
            qtest_irq_intercept_in(qts, path);
            return;
        }
    }
    g_assert_not_reached();
}

static void test_isolation(void)
{
    QTestState *qts = start_board();

    g_assert_true(has_device(qts, "child<qcom845-scm>"));
    g_assert_true(has_device(qts, "child<qcom845-spmi>"));
    g_assert_true(has_device(qts, "child<qcom845-rpmh>"));
    g_assert_true(has_device(qts, "child<qcom845-smmu>"));
    g_assert_true(has_device(qts, "child<qcom845-tsens>"));
    g_assert_true(has_device(qts, "child<qcom845-aoss>"));
    g_assert_true(has_device(qts, "child<qcom845-soc>"));
    g_assert_true(has_device(qts, "child<qcom845-cprh>"));
    g_assert_true(has_device(qts, "child<qcom845-cmddb>"));
    g_assert_true(has_device(qts, "child<qcom845-gcc>"));
    g_assert_true(has_device(qts, "child<qcom845-osm>"));
    g_assert_true(has_device(qts, "child<qcom845-display>"));
    g_assert_false(has_device(qts, "child<qcom8998-scm>"));
    g_assert_false(has_device(qts, "child<qcom8998-rpm>"));
    g_assert_false(has_device(qts, "child<qcom8998-spmi>"));
    g_assert_false(has_device(qts, "child<qcom-8998-display>"));
    /* 768 interrupt IDs, sufficient for native 541 plus synthetic 576..767. */
    g_assert_cmphex(qtest_readl(qts, 0x0b000004) & 31, ==, 23);
#ifdef CONFIG_HMP
    {
        g_autofree char *tree = qtest_hmp(qts, "info mtree");
        g_autofree char *registers = qtest_hmp(qts, "info registers");

        g_assert_nonnull(strstr(registers, "PC=0000000030000000"));
        g_assert_nonnull(strstr(tree, "0000000020000000-000000002effffff"));
        g_assert_nonnull(strstr(tree, "0000000030000000-0000000033ffffff"));
        g_assert_nonnull(strstr(tree, "0000000034000000-0000000034ffffff"));
        g_assert_nonnull(strstr(tree, "000000001b000000-000000001bffffff"));
        g_assert_nonnull(strstr(tree, "000000000c400000-000000000ebfffff"));
    }
#endif
    qtest_quit(qts);
    qts = qtest_init("-M oemsk -m 256M -nodefaults");
    g_assert_false(has_device(qts, "child<qcom845-spmi>"));
    g_assert_false(has_device(qts, "child<qcom845-scm>"));
    g_assert_false(has_device(qts, "child<qcom845-tsens>"));
    g_assert_false(has_device(qts, "child<qcom845-aoss>"));
    g_assert_false(has_device(qts, "child<qcom845-soc>"));
    g_assert_false(has_device(qts, "child<qcom845-cprh>"));
    g_assert_false(has_device(qts, "child<qcom845-cmddb>"));
    g_assert_false(has_device(qts, "child<qcom845-gcc>"));
    g_assert_false(has_device(qts, "child<qcom845-osm>"));
    g_assert_cmphex(qtest_readl(qts, 0x0b000004) & 31, ==, 21);
    qtest_quit(qts);
    qts = qtest_init("-M virt,gic-version=2 -cpu max -m 256M -nodefaults");
    g_assert_false(has_device(qts, "child<qcom845-spmi>"));
    g_assert_false(has_device(qts, "child<qcom845-scm>"));
    g_assert_false(has_device(qts, "child<qcom845-tsens>"));
    g_assert_false(has_device(qts, "child<qcom845-aoss>"));
    g_assert_false(has_device(qts, "child<qcom845-soc>"));
    g_assert_false(has_device(qts, "child<qcom845-cprh>"));
    g_assert_false(has_device(qts, "child<qcom845-cmddb>"));
    g_assert_false(has_device(qts, "child<qcom845-gcc>"));
    g_assert_false(has_device(qts, "child<qcom845-osm>"));
    qtest_quit(qts);
}

static void test_spmi_revision(void)
{
    QTestState *qts = start_board();

    g_assert_cmphex(qtest_readl(qts, SPMI + 0x40000), ==, 0x50000000);
    g_assert_cmphex(qtest_readl(qts, SPMI + 0x40900), ==, 0x100);
    g_assert_cmphex(qtest_readl(qts, SPMI + 0x40904), ==, 0x20100);
    g_assert_cmphex(qtest_readl(qts, SPMI + 0x40908), ==, 0x40100);
    g_assert_cmphex(qtest_readl(qts, SPMI + 0x4090c), ==, 0);
    for (unsigned apid = 0; apid < 3; apid++) {
        static const uint8_t subtype[] = { 0x14, 0x15, 0x18 };

        qtest_writel(qts, CHANNEL(apid), (1U << 27) | (3U << 4) | 2);
        g_assert_cmphex(qtest_readl(qts, CHANNEL(apid) + 8), ==, 1);
        g_assert_cmphex(qtest_readl(qts, CHANNEL(apid) + 0x18), ==,
                        (subtype[apid] << 16) | 0x5100 | (apid == 2 ? 1 : 2));
    }
    /* The next execution environment has a separate 64-KiB observer view. */
    qtest_writel(qts, OBS + 0x10000, (1U << 27) | (5U << 4));
    g_assert_cmphex(qtest_readl(qts, OBS + 0x10018), ==, 0x14);
    qtest_quit(qts);
}

static void test_spmi_failure_reset(void)
{
    QTestState *qts = start_board();

    /* Revision-only model must fail writes, rather than claim rail changes. */
    qtest_writel(qts, SPMI + 0x200000, 0);
    g_assert_cmphex(qtest_readl(qts, SPMI + 0x200008), ==, 3);
    qtest_writel(qts, OBS, (1U << 27) | (255U << 4) | 7);
    g_assert_cmphex(qtest_readl(qts, OBS + 8), ==, 3);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, SPMI + 0x200008), ==, 1);
    g_assert_cmphex(qtest_readl(qts, OBS + 8), ==, 1);
    g_assert_cmphex(qtest_readl(qts, OBS + 0x18), ==, 0);
    qtest_quit(qts);
}

#define RSC 0x179e0000
#define DISPLAY_RSC 0x0af20000
#define TCS(b) ((b) + ((b) == DISPLAY_RSC ? 0x1c00 : 0xd00))

static void rsc_command(QTestState *qts, uint64_t base, uint32_t message,
                        uint32_t address, uint32_t data)
{
    qtest_writel(qts, TCS(base) + 0x14, 0);
    qtest_writel(qts, TCS(base) + 0x30, message);
    qtest_writel(qts, TCS(base) + 0x34, address);
    qtest_writel(qts, TCS(base) + 0x38, data);
    qtest_writel(qts, TCS(base) + 0x1c, 1);
    qtest_writel(qts, TCS(base) + 0x14, (1U << 16) | (1U << 24));
}

static void test_rsc_votes_irq(void)
{
    QTestState *qts = start_board();

    intercept_gic(qts);
    g_assert_cmphex(qtest_readl(qts, RSC), ==, 0x20700);
    g_assert_cmphex(qtest_readl(qts, RSC + 0xc), ==,
                    (16U << 27) | (9 << 12) | (9 << 6) | 9);
    g_assert_cmphex(qtest_readl(qts, DISPLAY_RSC + 0xc) & 63, ==, 9);
    g_assert_cmphex(qtest_readl(qts, TCS(DISPLAY_RSC) + 0x18), ==, 1);
    g_assert_cmphex(qtest_readl(qts, DISPLAY_RSC + 0xd18), ==, 0);
    qtest_writel(qts, TCS(RSC), 1);
    rsc_command(qts, RSC, 0x10108, 0x30000, 0x1234);
    g_assert_cmphex(qtest_readl(qts, TCS(RSC) + 0x3c), ==, 0x10100);
    g_assert_cmphex(qtest_readl(qts, TCS(RSC) + 0x40), ==, 0x1234);
    g_assert_cmphex(qtest_readl(qts, TCS(RSC) + 4), ==, 1);
    g_assert_true(qtest_get_irq(qts, 37 - 32));
    qtest_writel(qts, TCS(RSC) + 8, 1);
    g_assert_false(qtest_get_irq(qts, 37 - 32));
    rsc_command(qts, DISPLAY_RSC, 0x108, 0x30000, 0);
    g_assert_cmphex(qtest_readl(qts, TCS(DISPLAY_RSC) + 0x40), ==, 0x1234);
    g_assert_false(qtest_get_irq(qts, 161 - 32));
    qtest_writel(qts, TCS(DISPLAY_RSC), 1);
    g_assert_true(qtest_get_irq(qts, 161 - 32));
    qtest_writel(qts, TCS(DISPLAY_RSC) + 8, 1);
    g_assert_false(qtest_get_irq(qts, 161 - 32));
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    rsc_command(qts, RSC, 0x108, 0x30000, 0);
    g_assert_cmphex(qtest_readl(qts, TCS(RSC) + 0x40), ==, 0);
    g_assert_false(qtest_get_irq(qts, 37 - 32));
    qtest_quit(qts);
}

static void test_rsc_reject_invalid(void)
{
    QTestState *qts = start_board();

    qtest_writel(qts, TCS(RSC), 1);
    rsc_command(qts, RSC, 0x10108, 0x30001, 42);
    g_assert_cmphex(qtest_readl(qts, TCS(RSC) + 4), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TCS(RSC) + 0x3c), ==, 0);
    rsc_command(qts, RSC, 0x10108, 0x1000000, 42);
    g_assert_cmphex(qtest_readl(qts, TCS(RSC) + 4), ==, 0);
    qtest_quit(qts);
}

#define GPU_SMMU 0x05040000
#define GPU_CB(n) (GPU_SMMU + 0x8000 + (n) * 0x1000)

static void test_smmu_fault_irq(void)
{
    QTestState *qts = start_board();

    intercept_gic(qts);
    g_assert_cmphex((qtest_readl(qts, GPU_SMMU + 0x20) >> 16) & 255, ==, 1);
    g_assert_cmphex((qtest_readl(qts, 0x15000020) >> 16) & 255, ==, 1);
    g_assert_cmphex(qtest_readl(qts, GPU_SMMU + 0x24) & 255, ==, 8);
    g_assert_cmphex(qtest_readl(qts, 0x15000024) & 255, ==, 64);
    qtest_writel(qts, GPU_CB(0), 1U << 6);
    qtest_writel(qts, GPU_CB(1), 1U << 6);
    qtest_writel(qts, GPU_CB(0) + 0x5c, 2);
    g_assert_true(qtest_get_irq(qts, 396 - 32));
    g_assert_false(qtest_get_irq(qts, 397 - 32));
    g_assert_cmphex(qtest_readl(qts, GPU_CB(1) + 0x58), ==, 0);
    /* Share context 0's line; clearing one fault must not lower the other. */
    qtest_writel(qts, GPU_SMMU + 0x1004, 1U << 16);
    qtest_writel(qts, GPU_CB(1) + 0x5c, 8);
    qtest_writel(qts, GPU_CB(0) + 0x58, 2);
    g_assert_true(qtest_get_irq(qts, 396 - 32));
    qtest_writel(qts, GPU_CB(1) + 0x58, 8);
    g_assert_false(qtest_get_irq(qts, 396 - 32));
    qtest_writel(qts, GPU_SMMU, 1U << 2);
    qtest_writel(qts, GPU_SMMU + 0x4c, 2);
    g_assert_true(qtest_get_irq(qts, 261 - 32));
    qtest_writel(qts, GPU_SMMU + 0x48, 2);
    g_assert_false(qtest_get_irq(qts, 261 - 32));
    qtest_quit(qts);
}

static void test_smmu_atos(void)
{
    QTestState *qts = start_board();
    const uint64_t root = 0x41000000, l2 = root + 0x1000, l3 = root + 0x2000;
    const uint64_t iova = 0x12345000, target = 0x42000000;
    const uint64_t pte = l3 + ((iova >> 12) & 511) * 8;

    qtest_writeq(qts, root, l2 | 3);
    qtest_writeq(qts, l2 + ((iova >> 21) & 511) * 8, l3 | 3);
    qtest_writeq(qts, pte, target | (1U << 10) | 3);
    qtest_writel(qts, GPU_SMMU + 0x1800, 1);
    qtest_writel(qts, GPU_CB(0) + 0x30, 25);
    qtest_writeq(qts, GPU_CB(0) + 0x20, root);
    qtest_writel(qts, GPU_CB(0), 1 | (1U << 6));
    qtest_writeq(qts, GPU_CB(0) + 0x800, iova);
    g_assert_cmphex(qtest_readq(qts, GPU_CB(0) + 0x50), ==, target);
    qtest_writeq(qts, pte, target | (1U << 10) | (1U << 7) | 3);
    qtest_writeq(qts, GPU_CB(0) + 0x808, iova);
    g_assert_cmphex(qtest_readq(qts, GPU_CB(0) + 0x50) & 1, ==, 1);
    qtest_writeq(qts, GPU_CB(0) + 0x800, iova);
    g_assert_cmphex(qtest_readq(qts, GPU_CB(0) + 0x50), ==, target);
    qtest_writeq(qts, pte, target | 3);
    qtest_writeq(qts, GPU_CB(0) + 0x800, iova);
    g_assert_cmphex(qtest_readq(qts, GPU_CB(0) + 0x50) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, GPU_CB(0) + 0x58), ==, 0);
    qtest_writeq(qts, GPU_CB(0) + 0x20, root | (UINT64_C(1) << 40));
    qtest_writeq(qts, GPU_CB(0) + 0x800, iova);
    g_assert_cmphex(qtest_readq(qts, GPU_CB(0) + 0x50) & 1, ==, 1);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, GPU_SMMU), ==, 1);
    g_assert_cmphex(qtest_readq(qts, GPU_CB(0) + 0x20), ==, 0);
    qtest_quit(qts);
}

static void test_tsens_temperature(void)
{
    QTestState *qts = start_board();
    const uint64_t srot[] = { 0x0c222000, 0x0c223000 };
    const uint64_t tm[] = { 0x0c263000, 0x0c265000 };

    for (unsigned i = 0; i < 2; i++) {
        unsigned mask = i == 0 ? 0x1fff : 0xff;

        g_assert_cmphex(qtest_readl(qts, srot[i]), ==, 0x20000000);
        g_assert_cmphex((qtest_readl(qts, srot[i] + 4) >> 3) & 0xffff,
                        ==, mask);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 0xe4), ==, 1);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 0xa0), ==, (1U << 21) | 250);
        qtest_writel(qts, srot[i] + 4, 0);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 0xe4), ==, 0);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 0xa0), ==, 0);
        qtest_writel(qts, srot[i] + 4, (1U << 21) | 1 | (2U << 3));
        g_assert_cmphex(qtest_readl(qts, tm[i] + 0xa0), ==, 0);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 0xa4), ==, (1U << 21) | 250);
        qtest_writel(qts, srot[i] + 4, 3);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 0xe4), ==, 0);
    }
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, tm[0] + 0xe4), ==, 1);
    g_assert_cmphex(qtest_readl(qts, tm[1] + 0xa0), ==, (1U << 21) | 250);
    qtest_quit(qts);
}

static void test_tsens_alarm_irq(void)
{
    QTestState *qts = start_board();
    const uint64_t tm[] = { 0x0c263000, 0x0c265000 };
    const unsigned irqs[][2] = { { 538 - 32, 540 - 32 },
                                { 539 - 32, 541 - 32 } };

    intercept_gic(qts);
    for (unsigned i = 0; i < 2; i++) {
        /* -10C lower, +20C upper, +20C critical: only high alarms. */
        qtest_writel(qts, tm[i] + 0x20, (200 << 12) | (4096 - 100));
        qtest_writel(qts, tm[i] + 0x60, 200);
        qtest_writel(qts, tm[i] + 4, 7);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 8), ==, 1U << 16);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 0x14), ==, 1);
        g_assert_false(qtest_get_irq(qts, irqs[i][0]));
        g_assert_false(qtest_get_irq(qts, irqs[i][1]));
        qtest_writel(qts, tm[i] + 0x10, ~(1U << 16));
        qtest_writel(qts, tm[i] + 0x1c, ~1U);
        g_assert_true(qtest_get_irq(qts, irqs[i][0]));
        g_assert_true(qtest_get_irq(qts, irqs[i][1]));
        qtest_writel(qts, tm[i] + 0xc, 1U << 16);
        qtest_writel(qts, tm[i] + 0x18, 1);
        g_assert_false(qtest_get_irq(qts, irqs[i][0]));
        g_assert_false(qtest_get_irq(qts, irqs[i][1]));
        qtest_clock_step(qts, 4000000);
        g_assert_cmphex(qtest_readl(qts, tm[i] + 8), ==, 0);
        qtest_writel(qts, tm[i] + 0xc, 0);
        qtest_writel(qts, tm[i] + 0x18, 0);
        g_assert_true(qtest_get_irq(qts, irqs[i][0]));
        g_assert_true(qtest_get_irq(qts, irqs[i][1]));
        qtest_writel(qts, tm[i] + 4, 0);
        g_assert_false(qtest_get_irq(qts, irqs[i][0]));
        g_assert_false(qtest_get_irq(qts, irqs[i][1]));
    }
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, tm[0] + 8), ==, 0);
    g_assert_cmphex(qtest_readl(qts, tm[1] + 0x14), ==, 0);
    qtest_quit(qts);
}

#define AOSS 0x0c300000
#define AOSS_DOORBELL 0x1799000c
#define AOSS_UP 0x0000ffff
#define AOSS_DOWN 0xffff0000

static void aoss_open(QTestState *qts)
{
    qtest_writel(qts, AOSS + 0x10, AOSS_UP);
    qtest_writel(qts, AOSS + 0x24, AOSS_UP);
    qtest_writel(qts, AOSS_DOORBELL, 1);
    qtest_writel(qts, AOSS + 0x2c, AOSS_UP);
    qtest_writel(qts, AOSS_DOORBELL, 1);
    qtest_writel(qts, AOSS + 0x18, AOSS_UP);
    qtest_writel(qts, AOSS_DOORBELL, 1);
}

static void test_aoss_handshake(void)
{
    QTestState *qts = start_board();
    const uint64_t gic = 0x0b000000;
    const uint64_t pending = gic + 0x200 + (421 / 32) * 4;
    const uint32_t bit = 1U << (421 % 32);

    g_assert_cmphex(qtest_readl(qts, AOSS), ==, 0x4d41494c);
    g_assert_cmphex(qtest_readl(qts, AOSS + 4), ==, 1);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x38), ==, 0x100);
    qtest_writel(qts, gic + 0xc00 + (421 / 16) * 4,
                  2U << ((421 % 16) * 2));
    qtest_writel(qts, AOSS + 0x24, AOSS_UP);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x28), ==, AOSS_DOWN);
    qtest_writel(qts, AOSS_DOORBELL, 1);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x28), ==, AOSS_UP);
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, bit);
    qtest_writel(qts, gic + 0x280 + (421 / 32) * 4, bit);
    aoss_open(qts);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x14), ==, AOSS_UP);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x30), ==, AOSS_UP);
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, bit);
    qtest_writel(qts, AOSS + 0x24, AOSS_DOWN);
    qtest_writel(qts, AOSS_DOORBELL, 1);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x28), ==, AOSS_DOWN);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x30), ==, AOSS_DOWN);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x24), ==, AOSS_DOWN);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x100), ==, 0);
    qtest_quit(qts);
}

static void test_aoss_messages(void)
{
    QTestState *qts = start_board();
    char message[64] = "{class: clock, res: qdss, val: 1}";
    const unsigned bytes = strlen(message) + 1;
    const uint64_t gic = 0x0b000000;
    const uint64_t pending = gic + 0x200 + (421 / 32) * 4;
    const uint32_t bit = 1U << (421 % 32);

    qtest_writel(qts, gic + 0xc00 + (421 / 16) * 4,
                  2U << ((421 % 16) * 2));
    aoss_open(qts);
    qtest_writel(qts, gic + 0x280 + (421 / 32) * 4, bit);
    for (unsigned i = 0; i < sizeof(message); i += 4) {
        uint32_t word = (uint8_t)message[i] | ((uint8_t)message[i + 1] << 8) |
                        ((uint8_t)message[i + 2] << 16) |
                        ((uint32_t)(uint8_t)message[i + 3] << 24);

        qtest_writel(qts, AOSS + 0x104 + i, word);
    }
    /* Original Windows requests have a byte count, not a word count. */
    g_assert_cmpuint(bytes & 3, !=, 0);
    qtest_writel(qts, AOSS + 0x100, bytes);
    qtest_writel(qts, AOSS_DOORBELL, 1);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x100), ==, 0);
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, bit);
    qtest_writel(qts, gic + 0x280 + (421 / 32) * 4, bit);
    /* Unknown or oversized messages must not be acknowledged. */
    qtest_writel(qts, AOSS + 0x104, 0xdeadbeef);
    qtest_writel(qts, AOSS + 0x100, 64);
    qtest_writel(qts, AOSS_DOORBELL, 1);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x100), ==, 64);
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, 0);
    qtest_writel(qts, AOSS + 0x100, 0x1000);
    qtest_writel(qts, AOSS_DOORBELL, 1);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x100), ==, 0x1000);
    qtest_writel(qts, AOSS + 0x24, 0x12345678);
    qtest_writel(qts, AOSS_DOORBELL, 1);
    g_assert_cmphex(qtest_readl(qts, AOSS + 0x28), ==, AOSS_UP);
    qtest_quit(qts);
}

static void test_soc_identity(void)
{
    QTestState *qts = start_board();
    uint32_t version = qtest_readl(qts, 0x01fc8000);

    /* Match the driver's major/minor decoding and the board's ACPI SIDV. */
    g_assert_cmphex((version & 255) | ((version & 0xff00) << 8), ==, 0x20001);
    g_assert_cmphex((qtest_readl(qts, 0x03998010) >> 12) & 0xffff, ==, 0x8b);
    qtest_writel(qts, 0x01fc8000, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, 0x01fc8000), ==, version);
    /* No physical per-device calibration data is fabricated. */
    g_assert_cmphex(qtest_readl(qts, 0x00784248), ==, 0);
    qtest_writel(qts, 0x00784248, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, 0x00784248), ==, 0);
    qtest_quit(qts);
}

static void test_bcm_revision(void)
{
    QTestState *qts = start_board();

    g_assert_cmphex(qtest_readl(qts, 0x0bbf1708), ==, 0x10500);
    qtest_writel(qts, 0x0bbf1708, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, 0x0bbf1708), ==, 0x10500);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, 0x0bbf1708), ==, 0x10500);
    qtest_quit(qts);
}

static void test_cprh_handoff(void)
{
    QTestState *qts = start_board();
    const uint64_t bases[] = { 0x17dc0000, 0x17db0000 };

    for (unsigned i = 0; i < 2; i++) {
        uint32_t control = qtest_readl(qts, bases[i] + 0x3a80);

        g_assert_cmphex(qtest_readl(qts, bases[i]), ==, 0x40050000);
        g_assert_cmphex(control & 1, ==, 1);
        g_assert_cmpuint(((control >> 1) & 0x3ff) * ((control >> 25) & 15),
                         ==, 400);
        qtest_writel(qts, bases[i] + 0x3a80, 0);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x3a80), ==, 0);
        qtest_writel(qts, bases[i] + 0x3a84, UINT32_MAX);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x3a84), ==, 0);
    }
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, bases[0] + 0x3a80) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, bases[1] + 0x3a80) & 1, ==, 1);
    qtest_quit(qts);
}

static void test_cmddb_directory(void)
{
    g_autofree char *path = NULL;
    g_autofree char *machine = NULL;
    g_autofree char *quoted = NULL;
    uint8_t data[0x90 + 24 + 2] = { 0 };
    QTestState *qts = start_board();
    int fd;

    g_assert_cmphex(qtest_readl(qts, 0x0c3f000c), ==, 0);
    qtest_quit(qts);
    fd = g_file_open_tmp("oemsk850-cmddb-XXXXXX", &path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    stl_le_p(data, 1);
    stl_le_p(data + 4, 0x0c0330db);
    stw_le_p(data + 8, 3);
    stw_le_p(data + 12, 24);
    stw_le_p(data + 14, 1);
    stw_le_p(data + 16, 0x1001);
    memcpy(data + 0x90, "cx.lvl", 6);
    stl_le_p(data + 0x90 + 16, 0x30000);
    stw_le_p(data + 0x90 + 20, 2);
    stw_le_p(data + 0x90 + 24, 128);
    g_assert_true(g_file_set_contents(path, (const char *)data, sizeof(data),
                                      NULL));
    machine = g_strdup_printf("oemsk850,cmd-db=%s", path);
    quoted = g_strdup_printf("\"%s\"", machine);
    qts = qtest_initf("-M %s -m 256M -nodefaults", quoted);
    g_assert_cmphex(qtest_readl(qts, 0x0c3f000c), ==, 0x0c3d0000);
    g_assert_cmphex(qtest_readl(qts, 0x0c3f0010), ==, sizeof(data));
    g_assert_cmphex(qtest_readq(qts, 0x0c3d0000), ==,
                    UINT64_C(0x0c0330db00000001));
    g_assert_cmphex(qtest_readw(qts, 0x0c3d0000 + 0x90 + 24), ==, 128);
    qtest_writel(qts, 0x0c3d0000, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, 0x0c3d0000), ==, 1);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, 0x0c3f000c), ==, 0x0c3d0000);
    qtest_quit(qts);
    unlink(path);
}

static void test_native_gicd_alias(void)
{
    QTestState *qts = start_board();

    g_assert_cmphex(qtest_readl(qts, 0x17a00004), ==,
                    qtest_readl(qts, 0x0b000004));
    for (unsigned offset = 0xfd0; offset < 0x1000; offset += 4) {
        g_assert_cmphex(qtest_readl(qts, 0x17a0f000 + offset), ==,
                        qtest_readl(qts, 0x0b000000 + offset));
    }
    g_assert_cmphex(qtest_readl(qts, 0x17a0ffe8) & 0xf0, ==, 0x20);
    /* Native INTID 37 is an RSC line, not a synthetic boot interrupt. */
    qtest_writel(qts, 0x17a00104, 1U << 5);
    g_assert_cmphex(qtest_readl(qts, 0x0b000104) & (1U << 5), ==, 1U << 5);
    qtest_writel(qts, 0x0b000184, 1U << 5);
    g_assert_cmphex(qtest_readl(qts, 0x17a00104) & (1U << 5), ==, 0);
    qtest_quit(qts);
}

static void test_gcc_clock_control(void)
{
    QTestState *qts = start_board();
    const uint64_t gcc = 0x00100000;

    /* GCC QUP DFS must not read a bit from the boot firmware FD. */
    g_assert_cmphex(qtest_readl(qts, gcc + 0x17048), ==, 0);
    qtest_writel(qts, gcc + 0x17048, 1);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x17048) & 1, ==, 0);
    qtest_writel(qts, gcc + 0x17034, 3);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x17034), ==, 2);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x502c) & (1U << 31), ==, 1U << 31);
    qtest_writel(qts, gcc + 0x502c, 1);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x502c) & (1U << 31), ==, 0);
    qtest_writel(qts, gcc + 0x52004, 1);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x414c) & (1U << 31), ==, 0);
    qtest_writel(qts, gcc + 0x52004, 0);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x414c) & (1U << 31), ==, 1U << 31);
    g_assert_cmphex(qtest_readl(qts, gcc + 0xf004) & (1U << 31), ==, 0);
    qtest_writel(qts, gcc + 0xf004, 0);
    g_assert_cmphex(qtest_readl(qts, gcc + 0xf004) & (1U << 31), ==, 1U << 31);
    g_assert_cmphex(qtest_readl(qts, gcc) & (1U << 31), ==, 1U << 31);
    g_assert_cmphex(qtest_readl(qts, gcc) & (1U << 30), ==, 0);
    qtest_writel(qts, gcc + 0x52000, 1);
    g_assert_cmphex(qtest_readl(qts, gcc) & (1U << 30), ==, 1U << 30);
    qtest_writel(qts, gcc + 0x52000, 0);
    g_assert_cmphex(qtest_readl(qts, gcc) & (1U << 30), ==, 0);
    qtest_writel(qts, gcc + 0x2c, 0);
    g_assert_cmphex(qtest_readl(qts, gcc) & (1U << 31), ==, 0);
    qtest_writel(qts, gcc + 0x2c, 1);
    g_assert_cmphex(qtest_readl(qts, gcc) & (1U << 31), ==, 1U << 31);
    /* GPLL6 uses vote bit 6; writing status flags must not fake a lock. */
    qtest_writel(qts, gcc + 0x13000, (1U << 20) | 6);
    qtest_writel(qts, gcc + 0x1302c, 1);
    qtest_writel(qts, gcc + 0x52000, 1U << 4);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x13000) & (1U << 30), ==, 0);
    qtest_writel(qts, gcc + 0x52000, 1U << 6);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x13000) >> 30, ==, 3);
    qtest_writel(qts, gcc + 0x76000, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x76000) >> 30, ==, 0);
    /* HLOS1 GPU SMMU vote GDSC (+0x7d02c) and clock (+0x7d000). */
    g_assert_cmphex(qtest_readl(qts, gcc + 0x7d02c), ==, 1);
    qtest_writel(qts, gcc + 0x7d02c, 0);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x7d02c) >> 31, ==, 1);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x7d000) >> 31, ==, 1);
    qtest_writel(qts, gcc + 0x7d000, 1);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x7d000) >> 31, ==, 0);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, gcc + 0x17034), ==, 0);
    g_assert_cmphex(qtest_readl(qts, gcc + 0xf004) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, gcc + 0x7d02c), ==, 1);
    qtest_quit(qts);
}

static void test_osm_lut(void)
{
    QTestState *qts = start_board();
    const uint64_t bases[] = { 0x17d41000, 0x17d43000, 0x17d45800 };

    for (unsigned i = 0; i < ARRAY_SIZE(bases); i++) {
        uint32_t last = qtest_readl(qts, bases[i] + 0x110 + 17 * 32);

        g_assert_cmphex(qtest_readl(qts, bases[i]), ==, 1);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x110 + 39 * 32), ==, last);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x110 + 14 * 32), <, last);
        qtest_writel(qts, bases[i] + 0x110 + 17 * 32, 0);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x110 + 17 * 32), ==, last);
        qtest_writel(qts, bases[i] + 0x920, 17);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x704), ==, last & 255);
        qtest_writel(qts, bases[i] + 0x920, 39);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x704), ==, last & 255);
        qtest_writel(qts, bases[i] + 0x920, 40);
        qtest_writel(qts, bases[i] + 0x920, UINT32_MAX);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x920), ==, 39);
        qtest_writel(qts, bases[i] + 0x924, 1);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x920), ==, 39);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 0x924), ==, 1);
    }
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, bases[1] + 0x920), ==, 0);
    qtest_quit(qts);
}

static void test_noc_qos(void)
{
    QTestState *qts = start_board();
    const uint64_t bases[] = {
        0x016e4000, 0x01704000, 0x01390000, 0x01749000, 0x01629000,
    };
    const unsigned offsets[] = { 8, 0x18, 0x40, 0x48 };
    const uint32_t masks[] = { 0x79, 0x03ff07ff, 0x7703, 0x03ff07ff };

    for (unsigned i = 0; i < ARRAY_SIZE(bases); i++) {
        for (unsigned f = 0; f < ARRAY_SIZE(offsets); f++) {
            uint64_t first = bases[i] + offsets[f];
            uint64_t last = first + 15 * 0x1000;

            g_assert_cmphex(qtest_readl(qts, first), ==, 0);
            qtest_writel(qts, first, UINT32_MAX);
            g_assert_cmphex(qtest_readl(qts, first), ==, masks[f]);
            g_assert_cmphex(qtest_readl(qts, last), ==, 0);
            qtest_writel(qts, last, 0x12345678);
            g_assert_cmphex(qtest_readl(qts, last), ==, 0x12345678 & masks[f]);
        }
    }
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    for (unsigned i = 0; i < ARRAY_SIZE(bases); i++) {
        g_assert_cmphex(qtest_readl(qts, bases[i] + 8), ==, 0);
        g_assert_cmphex(qtest_readl(qts, bases[i] + 15 * 0x1000 + 0x48), ==, 0);
    }
    qtest_quit(qts);
}

static void test_dispcc(void)
{
    QTestState *qts = start_board();
    const uint64_t base = 0x0af00000;

    g_assert_cmphex(qtest_readl(qts, base + 0x3000) >> 31, ==, 0);
    qtest_writel(qts, base + 0x3000, 0);
    g_assert_cmphex(qtest_readl(qts, base + 0x3000) >> 31, ==, 1);
    qtest_writel(qts, base + 0x4004, 1);
    g_assert_cmphex(qtest_readl(qts, base + 0x4004) >> 31, ==, 0);
    qtest_writel(qts, base + 0x4004, 0);
    g_assert_cmphex(qtest_readl(qts, base + 0x4004) >> 31, ==, 1);
    qtest_writel(qts, base + 0x208c, 0x1234);
    qtest_writel(qts, base + 0x2088, 3);
    g_assert_cmphex(qtest_readl(qts, base + 0x2088), ==, 2);
    g_assert_cmphex(qtest_readl(qts, base + 0x208c), ==, 0x1234);
    qtest_writel(qts, base, 7);
    g_assert_cmphex(qtest_readl(qts, base) >> 31, ==, 0);
    qtest_writel(qts, base + 0x2c, 1);
    g_assert_cmphex(qtest_readl(qts, base) >> 31, ==, 1);
    qtest_writel(qts, base + 0x2c, 0);
    g_assert_cmphex(qtest_readl(qts, base) >> 31, ==, 0);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, base + 0x3000), ==, 1);
    g_assert_cmphex(qtest_readl(qts, base + 0x208c), ==, 0);
    qtest_quit(qts);
}

static void test_tlmm(void)
{
    QTestState *qts = start_board();
    const uint64_t east = 0x0350a000; /* GPIO 10, display VSYNC mux. */

    g_assert_cmphex(qtest_readl(qts, east + 0x10), ==, 1);
    g_assert_cmphex(qtest_readl(qts, 0x0390a010), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x03d0a010), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x03904010), ==, 1); /* GPIO 4, north. */
    g_assert_cmphex(qtest_readl(qts, 0x03d0c010), ==, 1); /* GPIO 12, south. */
    g_assert_cmphex(qtest_readl(qts, 0x03995010), ==, 1); /* Last GPIO. */
    qtest_writel(qts, 0x0390a000, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, east), ==, 0);
    qtest_writel(qts, east + 4, 3);
    g_assert_cmphex(qtest_readl(qts, east + 4), ==, 2);
    qtest_writel(qts, east, UINT32_MAX);
    g_assert_cmphex(qtest_readq(qts, east), ==, UINT64_C(0x3000003ff));
    qtest_writel(qts, east + 8, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, east + 8), ==, 0xff);
    qtest_writel(qts, east + 0x10, 0);
    g_assert_cmphex(qtest_readl(qts, east + 0x10), ==, 1);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, east), ==, 0);
    g_assert_cmphex(qtest_readl(qts, east + 4), ==, 0);
    g_assert_cmphex(qtest_readl(qts, east + 8), ==, 0);
    qtest_quit(qts);
}

static void test_mmcc(void)
{
    QTestState *qts = start_board();
    const struct {
        uint64_t base;
        unsigned gdsc, branch, root, pll;
    } domains[] = {
        { 0x0ab00000, 0x8b4, 0x850, 0x7f0, 0x42c },
        { 0x0ad00000, 0x6004, 0x606c, 0x600c, 0 },
        { 0x05090000, 0x106c, 0x1098, 0x1120, 0x100 },
    };

    for (unsigned i = 0; i < ARRAY_SIZE(domains); i++) {
        uint64_t base = domains[i].base;
        uint64_t pll = base + domains[i].pll;
        uint64_t gdsc = base + domains[i].gdsc;
        uint64_t branch = base + domains[i].branch;
        uint64_t root = base + domains[i].root;

        g_assert_cmphex(qtest_readl(qts, gdsc), ==, 1);
        qtest_writel(qts, gdsc, 0);
        g_assert_cmphex(qtest_readl(qts, gdsc) >> 31, ==, 1);
        qtest_writel(qts, branch, 1);
        g_assert_cmphex(qtest_readl(qts, branch) >> 31, ==, 0);
        qtest_writel(qts, branch, 0);
        g_assert_cmphex(qtest_readl(qts, branch) >> 31, ==, 1);
        qtest_writel(qts, root, 3);
        g_assert_cmphex(qtest_readl(qts, root), ==, 2);
        qtest_writel(qts, pll, UINT32_MAX);
        g_assert_cmphex(qtest_readl(qts, pll) >> 30, ==, 0);
        qtest_writel(qts, pll, 6);
        qtest_writel(qts, pll + 0x2c, 1);
        g_assert_cmphex(qtest_readl(qts, pll) >> 31, ==, 1);
        qtest_writel(qts, pll, 0);
        g_assert_cmphex(qtest_readl(qts, pll) >> 31, ==, 0);
    }
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    for (unsigned i = 0; i < ARRAY_SIZE(domains); i++) {
        g_assert_cmphex(qtest_readl(qts, domains[i].base + domains[i].gdsc),
                        ==, 1);
    }
    qtest_quit(qts);
}

static void test_display_frontend(void)
{
    QTestState *qts = start_board();

    intercept_gic(qts);

    /* IMEM SRAM (0x14680000, 256 KiB) supports byte/dword/qword & resets. */
    qtest_writeb(qts, 0x146bf94c, 0x5a);
    g_assert_cmphex(qtest_readb(qts, 0x146bf94c), ==, 0x5a);
    qtest_writeq(qts, 0x146bf000, UINT64_C(0x1122334455667788));
    g_assert_cmphex(qtest_readq(qts, 0x146bf000), ==,
                    UINT64_C(0x1122334455667788));

    /* DPU 4.0 MDSS/MDP & Venus 4.4 version defaults + read-only protection. */
    g_assert_cmphex(qtest_readl(qts, 0x0ae00000), ==, 0x40000001);
    g_assert_cmphex(qtest_readl(qts, 0x0ae01000), ==, 0x40000001);
    g_assert_cmphex(qtest_readl(qts, 0x0aae0000), ==, 0x40440000);
    qtest_writel(qts, 0x0ae00000, 0xdeadbeef);
    qtest_writel(qts, 0x0aae0000, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, 0x0ae00000), ==, 0x40000001);
    g_assert_cmphex(qtest_readl(qts, 0x0aae0000), ==, 0x40440000);

    /* DPU 4.0 CTL0 SW_RESET self-clears and CTL0_START raises VSYNC IRQ. */
    qtest_writel(qts, 0x0ae02030, 1);
    g_assert_cmphex(qtest_readl(qts, 0x0ae02030), ==, 0);
    qtest_writel(qts, 0x0ae01010, 1U << 8);
    qtest_writel(qts, 0x0ae0201c, 1);
    g_assert_cmphex(qtest_readl(qts, 0x0ae00010) & 1, ==, 1);
    g_assert_true(qtest_get_irq(qts, 0x73 - 32));
    qtest_writel(qts, 0x0ae01018, 1U << 8);
    g_assert_false(qtest_get_irq(qts, 0x73 - 32));

    /* DSI0 DMA SW trigger raises DSI0 IRQ bit and clears via W1C. */
    qtest_writel(qts, 0x0ae94110, 1U << 1);
    qtest_writel(qts, 0x0ae94090, 1);
    g_assert_cmphex(qtest_readl(qts, 0x0ae00010) & (1U << 4), ==, 1U << 4);
    g_assert_true(qtest_get_irq(qts, 0x73 - 32));
    qtest_writel(qts, 0x0ae94110, 1U | (1U << 1));
    g_assert_false(qtest_get_irq(qts, 0x73 - 32));

    /* GPU RSC TCS0 idle status, 64-bit MMIO reads, and Adreno/PDC backing. */
    g_assert_cmphex(qtest_readl(qts, 0x0508dd18), ==, 1);
    qtest_writel(qts, 0x05002000, 0x08005000);
    qtest_writel(qts, 0x05002004, 0x00000008);
    g_assert_cmphex(qtest_readq(qts, 0x05002000), ==, UINT64_C(0x808005000));
    qtest_writel(qts, 0x0506a000, 0x55aa1122);
    g_assert_cmphex(qtest_readl(qts, 0x0506a000), ==, 0x55aa1122);
    qtest_writel(qts, 0x0b280000, 0x11223344);
    g_assert_cmphex(qtest_readl(qts, 0x0b280000), ==, 0x11223344);
    qtest_writel(qts, 0x0b480000, 0x87654321);
    g_assert_cmphex(qtest_readl(qts, 0x0b480000), ==, 0x87654321);

    /* Adreno 630 CP ring execution & GFX IRQ (mask +0xe0, AO +0x2ec54, clr +0xdc). */
    const uint64_t rb = 0x41200000, fence_pa = 0x41201000, rptr_pa = 0x41201018;
    qtest_writel(qts, rb + 0, 0x70468004); /* CP_EVENT_WRITE (4 dwords) */
    qtest_writel(qts, rb + 4, 0x80000004); /* CACHE_FLUSH_TS | IRQ */
    qtest_writel(qts, rb + 8, fence_pa);
    qtest_writel(qts, rb + 12, 0);
    qtest_writel(qts, rb + 16, 0xdeadcafe);
    qtest_writel(qts, rb + 20, 0x70408001); /* CP_INTERRUPT (1 dword) */
    qtest_writel(qts, rb + 24, 0x20000000); /* -> BIT(13) */
    qtest_writel(qts, 0x050000e0, (1U << 13) | (1U << 14) | (1U << 15));
    qtest_writel(qts, 0x05002008, 0x0d);
    qtest_writel(qts, 0x05002000, rb);
    qtest_writel(qts, 0x05002004, 0);
    qtest_writel(qts, 0x05002010, rptr_pa);
    qtest_writel(qts, 0x05002014, 0);
    qtest_writel(qts, 0x0500201c, 7);
    g_assert_cmphex(qtest_readl(qts, 0x05002018), ==, 7);
    g_assert_cmphex(qtest_readl(qts, rptr_pa), ==, 7);
    g_assert_cmphex(qtest_readl(qts, fence_pa), ==, 0xdeadcafe);
    g_assert_cmphex(qtest_readl(qts, 0x0508ec54), ==, (1U << 13) | (1U << 15));
    g_assert_cmphex(qtest_readl(qts, 0x05000804), ==, (1U << 13) | (1U << 15));
    g_assert_true(qtest_get_irq(qts, 0x14c - 32));
    qtest_writel(qts, 0x050000dc, (1U << 13) | (1U << 15));
    g_assert_cmphex(qtest_readl(qts, 0x0508ec54), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0x14c - 32));

    /* SMMUv2 TCR2 SEP=36bit + TTBR1 upper-range ATOS + CP_SMMU_TABLE_UPDATE. */
    const uint64_t l1 = 0x41300000, l2 = l1 + 0x1000, l3 = l1 + 0x2000;
    const uint64_t iova = UINT64_C(0x808005000), target = 0x42100000;
    qtest_writeq(qts, l1 + (((iova >> 30) & 31) * 8), l2 | 3);
    qtest_writeq(qts, l2 + (((iova >> 21) & 511) * 8), l3 | 3);
    qtest_writeq(qts, l3 + (((iova >> 12) & 511) * 8), target | (1U << 10) | 3);
    qtest_writel(qts, GPU_SMMU + 0x1800, 1);
    qtest_writel(qts, GPU_CB(0) + 0x10, 1U << 15); /* SEP = 36-bit */
    qtest_writel(qts, GPU_CB(0) + 0x30, (2U << 30) | (29U << 16) | 29U);
    qtest_writeq(qts, GPU_CB(0) + 0x28, l1);
    qtest_writel(qts, GPU_CB(0), 1);
    qtest_writeq(qts, GPU_CB(0) + 0x800, iova);
    g_assert_cmphex(qtest_readq(qts, GPU_CB(0) + 0x50), ==, target);

    /* Verify CP_SMMU_TABLE_UPDATE (0x53) switches CB0 TTBR0 and CP_COND_EXEC skips. */
    const uint64_t l1_low = 0x41310000;
    const uint64_t rb2 = 0x41202000, cond_bool = 0x41203000, out_pa = 0x41203010;
    qtest_writel(qts, GPU_SMMU, 1); /* Bypass SMMU for physical ring test */
    qtest_writel(qts, cond_bool, 0); /* Disabled -> CP_COND_EXEC must skip 4 dwords */
    qtest_writel(qts, out_pa, 0x11111111);
    qtest_writel(qts, rb2 + 0, 0x70538003); /* CP_SMMU_TABLE_UPDATE (3 dwords) */
    qtest_writel(qts, rb2 + 4, (uint32_t)l1_low);
    qtest_writel(qts, rb2 + 8, (uint32_t)(l1_low >> 32));
    qtest_writel(qts, rb2 + 12, 0);
    qtest_writel(qts, rb2 + 16, 0x70448006); /* CP_COND_EXEC (6 dwords) */
    qtest_writel(qts, rb2 + 20, cond_bool);
    qtest_writel(qts, rb2 + 24, 0);
    qtest_writel(qts, rb2 + 28, cond_bool);
    qtest_writel(qts, rb2 + 32, 0);
    qtest_writel(qts, rb2 + 36, 1);
    qtest_writel(qts, rb2 + 40, 4); /* Skip 4 dwords (CP_MEM_WRITE below) */
    qtest_writel(qts, rb2 + 44, 0x703d8003); /* Skipped CP_MEM_WRITE */
    qtest_writel(qts, rb2 + 48, out_pa);
    qtest_writel(qts, rb2 + 52, 0);
    qtest_writel(qts, rb2 + 56, 0xbad0bad0);
    qtest_writel(qts, 0x05002008, 0x0d);
    qtest_writel(qts, 0x05002000, rb2);
    qtest_writel(qts, 0x05002004, 0);
    qtest_writel(qts, 0x0500201c, 15);
    g_assert_cmphex(qtest_readq(qts, GPU_CB(0) + 0x20), ==, l1_low);
    g_assert_cmphex(qtest_readl(qts, out_pa), ==, 0x11111111);

    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    g_assert_cmphex(qtest_readl(qts, 0x0ae00000), ==, 0x40000001);
    g_assert_cmphex(qtest_readl(qts, 0x0aae0000), ==, 0x40440000);
    g_assert_cmphex(qtest_readb(qts, 0x146bf94c), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x050000e0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x0506a000), ==, 0);
    qtest_quit(qts);
}

static void assert_scanout_pixel(QTestState *qts, const char *path,
                                  unsigned x, unsigned y, uint32_t rgb)
{
    g_autofree char *data = NULL;
    gsize size;
    /* The merged width also avoids padding in the legacy PPM encoder. */
    const char header[] = "P6\n2700 1800\n255\n";
    size_t offset = sizeof(header) - 1 + (y * 2700 + x) * 3;

    qtest_qmp_assert_success(qts,
        "{'execute':'screendump','arguments':{'filename':%s,"
        "'device':'qcom845-display','head':2}}", path);
    g_assert_true(g_file_get_contents(path, &data, &size, NULL));
    g_assert_cmpuint(size, >, offset + 2);
    g_assert_cmpmem(data, sizeof(header) - 1, header, sizeof(header) - 1);
    g_assert_cmphex((uint8_t)data[offset], ==, rgb >> 16);
    g_assert_cmphex((uint8_t)data[offset + 1], ==, (rgb >> 8) & 255);
    g_assert_cmphex((uint8_t)data[offset + 2], ==, rgb & 255);
}

static void test_scanout_cpu_write(void)
{
    QTestState *qts = start_board();
    g_autofree char *path = NULL;
    int fd = g_file_open_tmp("oemsk850-scanout-XXXXXX", &path, NULL);
    const uint64_t source = 0x41500000;
    const uint64_t pipe = 0x0ae05000;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qtest_writel(qts, 0x15000000, 1); /* SMMU bypass. */
    qtest_writel(qts, pipe, (2U << 16) | 16); /* Two rows. */
    qtest_writel(qts, pipe + 0xc, (2U << 16) | 16);
    qtest_writel(qts, pipe + 0x24, 64);
    qtest_writel(qts, pipe + 0x14, source);
    qtest_writel(qts, 0x0ae02000, 1); /* VIG0 staged on CTL0. */
    qtest_writel(qts, 0x0ae02018, 1);
    assert_scanout_pixel(qts, path, 0, 0, 0);

    /* No further SSPP writes, GPU draws, CTL flushes or enabled IRQs. */
    qtest_writel(qts, source, 0x00112233);
    qtest_clock_step(qts, 100000000);
    assert_scanout_pixel(qts, path, 0, 0, 0x112233);
    assert_scanout_pixel(qts, path, 0, 1, 0);
    g_assert_cmphex(qtest_readl(qts, 0x0ae00010), ==, 0);
    qtest_writel(qts, source + 64, 0x00abcdef);
    qtest_clock_step(qts, 100000000);
    assert_scanout_pixel(qts, path, 0, 0, 0x112233);
    assert_scanout_pixel(qts, path, 0, 1, 0xabcdef);
    qtest_qmp_assert_success(qts, "{'execute':'system_reset'}");
    qtest_clock_step(qts, 20000000);
    assert_scanout_pixel(qts, path, 0, 0, 0);
    qtest_quit(qts);
    unlink(path);
}

static uint32_t pm4_type4(uint32_t reg, uint32_t cnt)
{
    return (4U << 28) | ((reg & 0x3ffffU) << 8) | (cnt & 0x7fU);
}

static uint32_t pm4_type7(uint32_t op, uint32_t cnt)
{
    return (7U << 28) | ((op & 0x7fU) << 16) | (cnt & 0x7fffU);
}

static void test_a630_direct_render(void)
{
    QTestState *qts = start_board();
    const uint64_t rb   = 0x41400000;
    const uint64_t dsg  = 0x41401000; /* Deferred CP_SET_DRAW_STATE IB */
    const uint64_t vs   = 0x41402000;
    const uint64_t fs   = 0x41403000;
    const uint64_t vb   = 0x41404000;
    const uint64_t btbl = 0x41405000; /* Bindless UBO descriptor table */
    const uint64_t ubo  = 0x41406000; /* UBO payload (RGBA float color) */
    const uint64_t rt   = 0x41410000; /* 16x16 linear BGRA8 render target */

    /* Bypass GPU SMMU so ring/shaders/buffers use direct physical addresses. */
    qtest_writel(qts, GPU_SMMU, 1);

    /* Clear 16x16 render target (pitch = 64 bytes = 16 pixels). */
    for (unsigned i = 0; i < 16 * 16; i++) {
        qtest_writel(qts, rt + i * 4, 0);
    }

    /* Vertex buffer: 3 vertices covering lower-left triangle of 16x16 target. */
    const uint32_t verts[6] = {
        0xbf800000, 0xbf800000, /* (-1, -1) -> (0, 16) */
        0x3f800000, 0xbf800000, /* ( 1, -1) -> (16, 16) */
        0xbf800000, 0x3f800000, /* (-1,  1) -> (0, 0) */
    };
    for (unsigned i = 0; i < 6; i++) {
        qtest_writel(qts, vb + i * 4, verts[i]);
    }

    /* UBO color = (R=0.8, G=0.4, B=0.2, A=1.0) -> BGRA8 0xffcc6633. */
    qtest_writel(qts, ubo + 0, 0x3f4ccccd);  /* 0.8f */
    qtest_writel(qts, ubo + 4, 0x3ecccccd);  /* 0.4f */
    qtest_writel(qts, ubo + 8, 0x3e4ccccd);  /* 0.2f */
    qtest_writel(qts, ubo + 12, 0x3f800000); /* 1.0f */

    /* Bindless UBO descriptor 0 at btbl (16-byte stride). */
    qtest_writel(qts, btbl + 0, (uint32_t)ubo);
    qtest_writel(qts, btbl + 4, (uint32_t)(ubo >> 32) | (1U << 17));

    /*
     * VS: VFD loads (x,y,0,1) into r0.xyzw; ldc loads vec4 color from
     * bindless base 3 slot 0 offset 0 into r1.xyzw; end.
     */
    qtest_writel(qts, vs + 0, 0x00c7b106); /* ldc.u32.4.imm.base3 r1.x, 0, 0 */
    qtest_writel(qts, vs + 4, 0xc0260004);
    qtest_writel(qts, vs + 8, 0x00000000); /* end */
    qtest_writel(qts, vs + 12, 0x03000000);

    /* FS: bary.f (ei)(rpt3)r0.x, 4; end. */
    qtest_writel(qts, fs + 0, 0x00002004);
    qtest_writel(qts, fs + 4, 0x47308b00);
    qtest_writel(qts, fs + 8, 0x00000000);
    qtest_writel(qts, fs + 12, 0x03000000);

    /* Deferred draw state group: viewport, scissor, MRT0, VFD, VS/FS, bindless. */
    uint32_t p = 0;
#define EMIT(base, val) qtest_writel(qts, (base) + (p++) * 4, (val))
    EMIT(dsg, pm4_type4(0x8000, 1)); EMIT(dsg, 0x60);       /* GRAS_CL_CNTL */
    EMIT(dsg, pm4_type4(0x8010, 4));
    EMIT(dsg, 0x41000000); /* XOFFSET = 8.0 */
    EMIT(dsg, 0x41000000); /* XSCALE  = 8.0 */
    EMIT(dsg, 0x41000000); /* YOFFSET = 8.0 */
    EMIT(dsg, 0xc1000000); /* YSCALE  = -8.0 */
    EMIT(dsg, pm4_type4(0x80f0, 2));
    EMIT(dsg, 0x00000000); EMIT(dsg, 0x000f000f);           /* 16x16 scissor */
    EMIT(dsg, pm4_type4(0x8820, 7));
    EMIT(dsg, 0x00000780); /* MRT0_CONTROL: mask=0xf, no blend */
    EMIT(dsg, 0x00000000); /* BLEND_CONTROL */
    EMIT(dsg, 0x00002030); /* BUF_INFO: RGBA8 linear + WZYX (BGRA) swap */
    EMIT(dsg, 0x00000001); /* PITCH: 1 * 64 = 64 bytes */
    EMIT(dsg, 0x00000010); /* ARRAY_PITCH: 16 * 64 = 1024 bytes */
    EMIT(dsg, (uint32_t)rt);
    EMIT(dsg, (uint32_t)(rt >> 32));
    EMIT(dsg, pm4_type4(0xa000, 2));
    EMIT(dsg, 0x00000101); /* VFD_CONTROL_0: 1 fetch, 1 decode */
    EMIT(dsg, 0xfcfcfcfc); /* VFD_CONTROL_1 */
    EMIT(dsg, pm4_type4(0xa010, 4));
    EMIT(dsg, (uint32_t)vb); EMIT(dsg, (uint32_t)(vb >> 32));
    EMIT(dsg, 24); EMIT(dsg, 8);                            /* 3 * 8B */
    EMIT(dsg, pm4_type4(0xa090, 1)); EMIT(dsg, 0x46700000); /* R32G32_FLOAT */
    EMIT(dsg, pm4_type4(0xa0d0, 1)); EMIT(dsg, 0x0000000f); /* r0.xyzw */
    EMIT(dsg, pm4_type4(0xa802, 2));
    EMIT(dsg, 0x00000002); /* SP_VS_OUT_CNT = 2 */
    EMIT(dsg, 0x0f040f00); /* out0: r0.xyzw, out1: r1.xyzw */
    EMIT(dsg, pm4_type4(0xa813, 1)); EMIT(dsg, 0x00000400); /* loc0=0, loc1=4 */
    EMIT(dsg, pm4_type4(0xa81c, 2));
    EMIT(dsg, (uint32_t)vs); EMIT(dsg, (uint32_t)(vs >> 32));
    EMIT(dsg, pm4_type4(0xa983, 2));
    EMIT(dsg, (uint32_t)fs); EMIT(dsg, (uint32_t)(fs >> 32));
    EMIT(dsg, pm4_type4(0xa98e, 1)); EMIT(dsg, 0x00000000); /* FS out r0.xyzw */
    EMIT(dsg, pm4_type4(0xab16, 2));
    EMIT(dsg, (uint32_t)btbl | 1U); /* SP_GFX_BINDLESS_BASE[3], 16B */
    EMIT(dsg, (uint32_t)(btbl >> 32));
    uint32_t dsg_dw = p;

    /* Ring buffer: CP_SET_MARKER + CP_SET_DRAW_STATE + CP_DRAW_INDX_OFFSET. */
    p = 0;
    EMIT(rb, pm4_type7(0x65, 1)); EMIT(rb, 1); /* RM6_DIRECT_RENDER */
    EMIT(rb, pm4_type7(0x43, 3));
    EMIT(rb, (1U << 24) | dsg_dw);             /* group 1, deferred */
    EMIT(rb, (uint32_t)dsg);
    EMIT(rb, (uint32_t)(dsg >> 32));
    EMIT(rb, pm4_type7(0x38, 3));
    EMIT(rb, 0x00000084);                      /* DI_PT_TRILIST, auto-index */
    EMIT(rb, 1);                               /* 1 instance */
    EMIT(rb, 3);                               /* 3 vertices */
#undef EMIT

    qtest_writel(qts, 0x05002008, 0x0d);
    qtest_writel(qts, 0x05002000, rb);
    qtest_writel(qts, 0x05002004, 0);
    qtest_writel(qts, 0x0500201c, p);

    /* Inside lower-left triangle: (2, 12) must be BGRA8 0xffcc6633. */
    g_assert_cmphex(qtest_readl(qts, rt + (12 * 16 + 2) * 4), ==, 0xffcc6633);
    /* Outside triangle (upper-right): (12, 2) must remain 0x00000000. */
    g_assert_cmphex(qtest_readl(qts, rt + (2 * 16 + 12) * 4), ==, 0x00000000);

    /*
     * Exercise GMEM binning + DI_PT_RECTLIST path used by Direct2D console:
     * 1) RM6_BIN_VISIBILITY (marker 0x2) must skip rasterization.
     * 2) RM6_BIN_RENDER_START (marker 0x14) with TILE6_2 must route output to
     *    RB_RESOLVE_SYSTEM_BUFFER_* even when RB_MRT_0_BASE is stale.
     * 3) DI_PT_RECTLIST must ignore stale non-zero VFD_INDEX_OFFSET.
     */
    const uint64_t rb2  = 0x41407000;
    const uint64_t ubo2 = 0x41408000; /* Rect vertex UBO: 2 vec4s */
    const uint64_t vs_r = 0x41409000; /* RECTLIST VS indexed by raw vtx_id */
    const uint64_t rt2  = 0x41411000; /* New swapchain backbuffer (16x16) */
    for (unsigned i = 0; i < 16 * 16; i++) {
        qtest_writel(qts, rt2 + i * 4, 0);
    }
    /* UBO[1][0] = (8.0, 2.0, 0, 1), UBO[1][1] = (12.0, 6.0, 0, 1). */
    qtest_writel(qts, ubo2 + 0,  0x41000000); /* 8.0f */
    qtest_writel(qts, ubo2 + 4,  0x40000000); /* 2.0f */
    qtest_writel(qts, ubo2 + 8,  0x00000000);
    qtest_writel(qts, ubo2 + 12, 0x3f800000);
    qtest_writel(qts, ubo2 + 16, 0x41400000); /* 12.0f */
    qtest_writel(qts, ubo2 + 20, 0x40c00000); /* 6.0f */
    qtest_writel(qts, ubo2 + 24, 0x00000000);
    qtest_writel(qts, ubo2 + 28, 0x3f800000);
    /* Bindless UBO descriptor 1 at btbl + 16. */
    qtest_writel(qts, btbl + 16, (uint32_t)ubo2);
    qtest_writel(qts, btbl + 20, (uint32_t)(ubo2 >> 32) | (2U << 17));
    /*
     * vs_r: ldc.4 r1.xyzw, cb[0][0] (color);
     *       ldc.4 r0.xyzw, cb[1][r0.x] (position from raw vertex ID); end.
     */
    qtest_writel(qts, vs_r + 0,  0x00c7b106);
    qtest_writel(qts, vs_r + 4,  0xc0260004);
    qtest_writel(qts, vs_r + 8,  0x0047b106); /* s1_im=0, src1=0 (r0.x) */
    qtest_writel(qts, vs_r + 12, 0xc0260200); /* ubo=1, dst=0 (r0.xyzw) */
    qtest_writel(qts, vs_r + 16, 0x00000000); /* end */
    qtest_writel(qts, vs_r + 20, 0x03000000);

    /* Change UBO color to (R=0.2, G=0.8, B=0.4, A=1.0) -> BGRA8 0xff33cc66. */
    qtest_writel(qts, ubo + 0, 0x3e4ccccd);
    qtest_writel(qts, ubo + 4, 0x3f4ccccd);
    qtest_writel(qts, ubo + 8, 0x3ecccccd);
    qtest_writel(qts, ubo + 12, 0x3f800000);

    p = 0;
#define EMIT2(base, val) qtest_writel(qts, (base) + (p++) * 4, (val))
    /* Leave RB_MRT_0_BASE pointing at stale rt, switch BUF_INFO to TILE6_2. */
    EMIT2(rb2, pm4_type4(0x8822, 1)); EMIT2(rb2, 0x00002230); /* TILE6_2 */
    /* Program RB_RESOLVE_SYSTEM_BUFFER_* to rt2 (linear, BGRA8, pitch=64). */
    EMIT2(rb2, pm4_type4(0x88d7, 5));
    EMIT2(rb2, (0x30U << 7) | (1U << 5)); /* linear, swap=1 (WZYX), RGBA8 */
    EMIT2(rb2, (uint32_t)rt2);
    EMIT2(rb2, (uint32_t)(rt2 >> 32));
    EMIT2(rb2, 1);  /* pitch = 64 bytes */
    EMIT2(rb2, 16); /* array_pitch = 1024 bytes */
    /* Visibility pass (marker 0x2) with full scissor must not write rt or rt2. */
    EMIT2(rb2, pm4_type7(0x65, 1)); EMIT2(rb2, 0x2);
    EMIT2(rb2, pm4_type7(0x38, 3)); EMIT2(rb2, 0x84); EMIT2(rb2, 1); EMIT2(rb2, 3);
    /* GMEM bin render (marker 0x14) clipped to [0..7, 8..15] writes rt2 only. */
    EMIT2(rb2, pm4_type4(0x80f0, 2)); EMIT2(rb2, 0x00080000); EMIT2(rb2, 0x000f0007);
    EMIT2(rb2, pm4_type7(0x65, 1)); EMIT2(rb2, 0x14);
    EMIT2(rb2, pm4_type7(0x38, 3)); EMIT2(rb2, 0x84); EMIT2(rb2, 1); EMIT2(rb2, 3);
    /* Direct-render DI_PT_RECTLIST with stale VFD_INDEX_OFFSET=99 -> rt2 [8..12,2..6]. */
    EMIT2(rb2, pm4_type4(0x80f0, 2)); EMIT2(rb2, 0x00000000); EMIT2(rb2, 0x000f000f);
    EMIT2(rb2, pm4_type4(0x8822, 5));
    EMIT2(rb2, 0x00002030); EMIT2(rb2, 1); EMIT2(rb2, 16);
    EMIT2(rb2, (uint32_t)rt2); EMIT2(rb2, (uint32_t)(rt2 >> 32));
    EMIT2(rb2, pm4_type4(0xa001, 1)); EMIT2(rb2, 0xfcfcfc00); /* vtx_id -> r0.x */
    EMIT2(rb2, pm4_type4(0xa00e, 1)); EMIT2(rb2, 99); /* Stale VFD_INDEX_OFFSET */
    EMIT2(rb2, pm4_type4(0xa81c, 2));
    EMIT2(rb2, (uint32_t)vs_r); EMIT2(rb2, (uint32_t)(vs_r >> 32));
    EMIT2(rb2, pm4_type4(0x8000, 1)); EMIT2(rb2, 0x61); /* screen_space = 1 */
    EMIT2(rb2, pm4_type7(0x65, 1)); EMIT2(rb2, 0x1);
    EMIT2(rb2, pm4_type7(0x38, 3)); EMIT2(rb2, 0x88); EMIT2(rb2, 1); EMIT2(rb2, 2);
#undef EMIT2

    qtest_writel(qts, 0x05002008, 0x0d);
    qtest_writel(qts, 0x05002000, rb2);
    qtest_writel(qts, 0x05002004, 0);
    qtest_writel(qts, 0x0500201c, p);

    /* Stale rt must remain untouched (still original 0xffcc6633, not overwritten). */
    g_assert_cmphex(qtest_readl(qts, rt + (12 * 16 + 2) * 4), ==, 0xffcc6633);
    /* GMEM bin rendered (2, 12) in rt2; visibility pass did not draw (1, 2). */
    g_assert_cmphex(qtest_readl(qts, rt2 + (12 * 16 + 2) * 4), ==, 0xff33cc66);
    g_assert_cmphex(qtest_readl(qts, rt2 + (2 * 16 + 1) * 4), ==, 0x00000000);
    /* DI_PT_RECTLIST ignored stale VFD_INDEX_OFFSET and filled (10, 4) in rt2. */
    g_assert_cmphex(qtest_readl(qts, rt2 + (4 * 16 + 10) * 4), ==, 0xff33cc66);

    /* Solid RECTLIST across page boundaries, with clipping and row padding. */
    const uint64_t wide_rt = 0x41800ffc;
    const unsigned wide_pitch = 5440;

    qtest_writel(qts, ubo2, 0x40400000);      /* x0 = 3 */
    qtest_writel(qts, ubo2 + 4, 0x3f800000);  /* y0 = 1 */
    qtest_writel(qts, ubo2 + 16, 0x44a8a000); /* x1 = 1349 */
    qtest_writel(qts, ubo2 + 20, 0x40400000); /* y1 = 3 */
    qtest_writel(qts, 0x050203c0, 0x00010007); /* scissor TL (7, 1) */
    qtest_writel(qts, 0x050203c4, 0x00020542); /* scissor BR (1346, 2) */
    qtest_writel(qts, 0x0502208c, wide_pitch / 64);
    qtest_writel(qts, 0x05022090, wide_pitch * 4 / 64);
    qtest_writel(qts, 0x05022094, wide_rt);
    qtest_writel(qts, 0x05022098, 0);
    qtest_writel(qts, rb2, pm4_type7(0x38, 3));
    qtest_writel(qts, rb2 + 4, 0x88);
    qtest_writel(qts, rb2 + 8, 1);
    qtest_writel(qts, rb2 + 12, 2);
    qtest_writel(qts, 0x05002018, 0);
    qtest_writel(qts, 0x0500201c, 4);
    for (unsigned y = 1; y <= 2; y++) {
        uint64_t row = wide_rt + y * wide_pitch;

        g_assert_cmphex(qtest_readl(qts, row + 6 * 4), ==, 0);
        g_assert_cmphex(qtest_readl(qts, row + 7 * 4), ==, 0xff33cc66);
        g_assert_cmphex(qtest_readl(qts, row + 1024 * 4), ==, 0xff33cc66);
        g_assert_cmphex(qtest_readl(qts, row + 1346 * 4), ==, 0xff33cc66);
        g_assert_cmphex(qtest_readl(qts, row + 1347 * 4), ==, 0);
        g_assert_cmphex(qtest_readl(qts, row + 1350 * 4), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, wide_rt + 7 * 4), ==, 0);
    g_assert_cmphex(qtest_readl(qts, wide_rt + 3 * wide_pitch + 7 * 4), ==, 0);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/oemsk850/isolation", test_isolation);
    qtest_add_func("/oemsk850/spmi-revision", test_spmi_revision);
    qtest_add_func("/oemsk850/spmi-failure-reset", test_spmi_failure_reset);
    qtest_add_func("/oemsk850/rsc-votes-irq", test_rsc_votes_irq);
    qtest_add_func("/oemsk850/rsc-reject-invalid", test_rsc_reject_invalid);
    qtest_add_func("/oemsk850/smmu-fault-irq", test_smmu_fault_irq);
    qtest_add_func("/oemsk850/smmu-atos", test_smmu_atos);
    qtest_add_func("/oemsk850/tsens-temperature", test_tsens_temperature);
    qtest_add_func("/oemsk850/tsens-alarm-irq", test_tsens_alarm_irq);
    qtest_add_func("/oemsk850/aoss-handshake", test_aoss_handshake);
    qtest_add_func("/oemsk850/aoss-messages", test_aoss_messages);
    qtest_add_func("/oemsk850/soc-identity", test_soc_identity);
    qtest_add_func("/oemsk850/bcm-revision", test_bcm_revision);
    qtest_add_func("/oemsk850/cprh-handoff", test_cprh_handoff);
    qtest_add_func("/oemsk850/cmddb-directory", test_cmddb_directory);
    qtest_add_func("/oemsk850/native-gicd-alias", test_native_gicd_alias);
    qtest_add_func("/oemsk850/gcc-clock-control", test_gcc_clock_control);
    qtest_add_func("/oemsk850/osm-lut", test_osm_lut);
    qtest_add_func("/oemsk850/noc-qos", test_noc_qos);
    qtest_add_func("/oemsk850/display-clock-control", test_dispcc);
    qtest_add_func("/oemsk850/tlmm", test_tlmm);
    qtest_add_func("/oemsk850/multimedia-clock-control", test_mmcc);
    qtest_add_func("/oemsk850/display-frontend", test_display_frontend);
    qtest_add_func("/oemsk850/scanout-cpu-write", test_scanout_cpu_write);
    qtest_add_func("/oemsk850/a630-direct-render", test_a630_direct_render);
    return g_test_run();
}
