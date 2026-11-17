/* OEMSK board and Adreno SMMUv2 regression tests. */
/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"
#include "qobject/qlist.h"

#define GPU 0x05000000
#define SMMU 0x05040000
#define CB(n) (SMMU + 0x8000 + (n) * 0x1000)
#define ROOT 0x41000000
#define L2 0x41001000
#define L3 0x41002000
#define RING_RAM 0x42000000
#define RING_IOVA UINT64_C(0x808005000)
#define SHADOW_IOVA UINT64_C(0x808000018)
#define FSR 0x58
#define CB_FAR 0x60
#define FSYNR0 0x68
#define CFIE (1U << 6)
#define HUPCF (1U << 8)
#define MULTI (1U << 31)
#define FAULT_MASK (MULTI | 0x1fe)
#define AF (1U << 10)
#define CR0_FAULTS ((1U << 10) | (1U << 21))

static char *find_device(QTestState *qts, const char *type)
{
    g_autoptr(QDict) reply = qtest_qmp(qts,
        "{'execute':'qom-list','arguments':{'path':'/machine/unattached'}}");
    QList *list = qdict_get_qlist(reply, "return");
    QListEntry *entry;

    g_assert_nonnull(list);
    QLIST_FOREACH_ENTRY(list, entry) {
        QDict *item = qobject_to(QDict, qlist_entry_obj(entry));

        if (!strcmp(qdict_get_str(item, "type"), type)) {
            return g_strdup_printf("/machine/unattached/%s",
                                    qdict_get_str(item, "name"));
        }
    }
    return NULL;
}

static QTestState *start_oemsk(void)
{
    QTestState *qts = qtest_init("-M oemsk -m 256M -nodefaults");
    g_autofree char *path = find_device(qts, "child<arm_gic>");

    g_assert_nonnull(path);
    qtest_irq_intercept_in(qts, path);
    return qts;
}

static void set_context(QTestState *qts, unsigned index, uint32_t control)
{
    qtest_writel(qts, SMMU + 0x800, 1U << 31);
    qtest_writel(qts, SMMU + 0xc00, index);
    qtest_writel(qts, SMMU + 0x1000 + index * 4, 1U << 16);
    qtest_writel(qts, SMMU + 0x1800 + index * 4, 1);
    qtest_writel(qts, CB(index), control | 1);
    qtest_writel(qts, CB(index) + 0x10, (1U << 15) | 1);
    qtest_writel(qts, CB(index) + 0x30, (2U << 30) | (29 << 16) | 29);
    qtest_writeq(qts, CB(index) + 0x28, ROOT);
}

static void set_ring(QTestState *qts)
{
    /* Original QCDX ME_INIT, three contexts and error detection enabled. */
    static const uint32_t words[] = {
        0x70c80004, 0x0000000b, 3, 0x20000000, 0,
    };

    qtest_writeq(qts, ROOT, L2 | 3);
    qtest_writeq(qts, L2 + 0x40 * 8, L3 | 3);
    qtest_writeq(qts, GPU + 0x2000, RING_IOVA);
    qtest_writel(qts, GPU + 0x2008, 13 | (1U << 27));
    for (unsigned i = 0; i < G_N_ELEMENTS(words); i++) {
        qtest_writel(qts, RING_RAM + i * 4, words[i]);
    }
}

static void trigger(QTestState *qts)
{
    qtest_writel(qts, GPU + 0x201c, 5);
}

static uint32_t fault(QTestState *qts, unsigned index)
{
    return qtest_readl(qts, CB(index) + FSR) & FAULT_MASK;
}

static void clear_fault(QTestState *qts, unsigned index)
{
    qtest_writel(qts, CB(index) + FSR, UINT32_MAX);
    g_assert_cmphex(fault(qts, index), ==, 0);
}

static void test_context_fault(void)
{
    QTestState *qts = start_oemsk();

    set_context(qts, 0, HUPCF);
    set_ring(qts);
    trigger(qts);
    g_assert_cmphex(fault(qts, 0), ==, 2);
    g_assert_cmphex(qtest_readq(qts, CB(0) + CB_FAR), ==, RING_IOVA);
    g_assert_cmphex(qtest_readl(qts, CB(0) + FSR) & 0x600, ==, 0x400);
    g_assert_false(qtest_get_irq(qts, 361 - 32));
    /* CFRE is zero: recording and CFIE still work independently. */
    qtest_writel(qts, CB(0), 1 | HUPCF | CFIE);
    g_assert_true(qtest_get_irq(qts, 361 - 32));
    g_assert_cmphex(qtest_readl(qts, 0x0b000200 + (361 / 32) * 4) &
                   (1U << (361 % 32)), !=, 0);
    qtest_writel(qts, CB(0) + FSR, 0);
    g_assert_true(qtest_get_irq(qts, 361 - 32));
    trigger(qts);
    g_assert_cmphex(fault(qts, 0), ==, MULTI | 2);
    g_assert_cmphex(qtest_readq(qts, CB(0) + CB_FAR), ==, RING_IOVA);
    qtest_writel(qts, CB(0) + FSR, 2);
    g_assert_cmphex(fault(qts, 0), ==, MULTI);
    g_assert_true(qtest_get_irq(qts, 361 - 32));
    clear_fault(qts, 0);
    g_assert_false(qtest_get_irq(qts, 361 - 32));
    /* Terminated DMA is retried only after the device is submitted again. */
    qtest_writeq(qts, L3 + 5 * 8, RING_RAM | AF | 3);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 0);
    trigger(qts);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    g_assert_cmphex(fault(qts, 0), ==, 0);
    qtest_quit(qts);
}

static void test_fault_classes(void)
{
    QTestState *qts = start_oemsk();

    set_context(qts, 0, HUPCF | CFIE);
    set_ring(qts);
    qtest_writel(qts, CB(0), 1 | HUPCF | CFIE | (1U << 7));
    g_assert_cmphex(qtest_readl(qts, CB(0)) & (1U << 7), ==, 0);
    qtest_writel(qts, CB(0) + 0x30,
                 (2U << 30) | (29 << 16) | 29 | (1U << 23));
    trigger(qts);
    g_assert_cmphex(fault(qts, 0), ==, 2); /* EPD1 disables table walks. */
    clear_fault(qts, 0);
    qtest_writel(qts, CB(0) + 0x30, (3U << 30) | (29 << 16) | 29);
    trigger(qts);
    g_assert_cmphex(fault(qts, 0), ==, 0x100); /* Unsupported granule. */
    clear_fault(qts, 0);
    qtest_writel(qts, CB(0) + 0x30, (2U << 30) | (29 << 16) | 29);
    qtest_writeq(qts, L3 + 5 * 8, RING_RAM | 3);
    trigger(qts);
    g_assert_cmphex(fault(qts, 0), ==, 4); /* Access flag. */
    clear_fault(qts, 0);
    qtest_writeq(qts, L3 + 5 * 8, RING_RAM | AF | 3 | (UINT64_C(1) << 36));
    trigger(qts);
    g_assert_cmphex(fault(qts, 0), ==, 0x80); /* Address size. */
    clear_fault(qts, 0);
    qtest_writeq(qts, CB(0) + 0x28, 0x20000000);
    trigger(qts);
    g_assert_cmphex(fault(qts, 0), ==, 0x10); /* Table-walk external abort. */
    g_assert_cmphex(qtest_readl(qts, CB(0) + FSYNR0) & (1U << 10), !=, 0);
    clear_fault(qts, 0);
    qtest_writeq(qts, CB(0) + 0x28, ROOT);
    qtest_writeq(qts, L3 + 5 * 8, RING_RAM | AF | 3);
    qtest_writeq(qts, L3, 0x42001000 | AF | 3 | (1U << 7));
    qtest_writel(qts, 0x42001018, 0xfeedface);
    qtest_writeq(qts, GPU + 0x2010, SHADOW_IOVA);
    qtest_writel(qts, GPU + 0x2008, 13);
    trigger(qts);
    g_assert_cmphex(fault(qts, 0), ==, 8); /* Read-only pointer shadow. */
    g_assert_cmphex(qtest_readq(qts, CB(0) + CB_FAR), ==, SHADOW_IOVA);
    g_assert_cmphex(qtest_readl(qts, CB(0) + FSYNR0) & (1U << 4), !=, 0);
    g_assert_cmphex(qtest_readl(qts, 0x42001018), ==, 0xfeedface);
    qtest_quit(qts);
}

static void test_context_isolation(void)
{
    QTestState *qts = start_oemsk();

    set_context(qts, 0, CFIE);
    set_ring(qts);
    trigger(qts);
    /* HUPCF=0 prevents a repaired mapping from running until acknowledgment. */
    qtest_writeq(qts, L3 + 5 * 8, RING_RAM | AF | 3);
    trigger(qts);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 0);
    g_assert_cmphex(fault(qts, 0), ==, 2);
    qtest_writeq(qts, L3 + 5 * 8, 0);
    set_context(qts, 1, CFIE);
    trigger(qts);
    g_assert_true(qtest_get_irq(qts, 361 - 32));
    g_assert_true(qtest_get_irq(qts, 362 - 32));
    clear_fault(qts, 0);
    g_assert_false(qtest_get_irq(qts, 361 - 32));
    g_assert_true(qtest_get_irq(qts, 362 - 32));
    qtest_system_reset(qts);
    g_assert_false(qtest_get_irq(qts, 362 - 32));
    g_assert_cmphex(fault(qts, 0), ==, 0);
    g_assert_cmphex(fault(qts, 1), ==, 0);
    qtest_quit(qts);
}

static void test_global_fault(void)
{
    QTestState *qts = start_oemsk();

    set_ring(qts);
    trigger(qts);
    g_assert_cmphex(qtest_readl(qts, SMMU + 0x48), ==, 2);
    g_assert_false(qtest_get_irq(qts, 366 - 32));
    qtest_writel(qts, SMMU, CR0_FAULTS | 4);
    g_assert_true(qtest_get_irq(qts, 366 - 32));
    qtest_writel(qts, SMMU + 0x48, UINT32_MAX);
    g_assert_false(qtest_get_irq(qts, 366 - 32));
    set_context(qts, 0, CFIE);
    qtest_writel(qts, SMMU + 0x804, 1U << 31);
    trigger(qts);
    g_assert_cmphex(qtest_readl(qts, SMMU + 0x48), ==, 4);
    qtest_writel(qts, SMMU + 0x48, UINT32_MAX);
    qtest_writel(qts, SMMU + 0x804, 0);
    qtest_writel(qts, SMMU + 0xc00, 7);
    trigger(qts);
    g_assert_cmphex(qtest_readl(qts, SMMU + 0x48), ==, 8);
    g_assert_cmphex(fault(qts, 0), ==, 0);
    qtest_quit(qts);
}

static void test_machine_separation(void)
{
    QTestState *qts = start_oemsk();
    g_autofree char *bus = find_device(qts, "child<platform-bus-device>");
    g_autoptr(QDict) reply = qtest_qmp(qts,
        "{'execute':'qom-get','arguments':{'path':%s,'property':'mmio_size'}}",
        bus);

    g_assert_cmphex(qdict_get_int(reply, "return"), ==, 0xc00000);
    /* 704 INTIDs, with synthetic SPI space above the native device IRQs. */
    g_assert_cmphex(qtest_readl(qts, 0x0b000004) & 31, ==, 21);
    qtest_writel(qts, 0x146bf94c, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, 0x146bf94c), ==, 0x12345678);
#ifdef CONFIG_HMP
    {
        g_autofree char *tree = qtest_hmp(qts, "info mtree");

        g_assert_nonnull(strstr(tree, "0000000020000000-000000003efeffff"));
    }
#endif
    qtest_quit(qts);
    qts = qtest_init("-M virt,gic-version=2 -cpu max -m 256M -nodefaults");
    g_autofree char *display = find_device(qts, "child<qcom-8998-display>");

    g_assert_null(display);
    g_assert_cmphex(qtest_readl(qts, 0x08000004) & 31, ==, 8);
    qtest_quit(qts);
}

static void test_bypass(void)
{
    for (unsigned mode = 0; mode < 3; mode++) {
        QTestState *qts = start_oemsk();

        set_ring(qts);
        if (mode == 0) {
            qtest_writel(qts, SMMU, CR0_FAULTS | 1); /* CLIENTPD. */
        } else if (mode == 1) {
            qtest_writel(qts, SMMU + 0x800, 1U << 31);
            qtest_writel(qts, SMMU + 0xc00, 1U << 16); /* S2CR bypass. */
        } else {
            set_context(qts, 0, 0);
            qtest_writel(qts, CB(0), 0); /* Context translation disabled. */
        }
        qtest_writeq(qts, GPU + 0x2000, RING_RAM);
        trigger(qts);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
        g_assert_cmphex(qtest_readl(qts, SMMU + 0x48), ==, 0);
        /* Physical bypass still cannot turn command DMA into MMIO access. */
        qtest_writeq(qts, GPU + 0x2000, GPU);
        qtest_writel(qts, GPU + 0x2018, 0);
        trigger(qts);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 0);
        qtest_quit(qts);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/oemsk/context-fault", test_context_fault);
    qtest_add_func("/oemsk/fault-classes", test_fault_classes);
    qtest_add_func("/oemsk/context-isolation-reset", test_context_isolation);
    qtest_add_func("/oemsk/global-fault", test_global_fault);
    qtest_add_func("/oemsk/machine-separation", test_machine_separation);
    qtest_add_func("/oemsk/bypass", test_bypass);
    return g_test_run();
}
