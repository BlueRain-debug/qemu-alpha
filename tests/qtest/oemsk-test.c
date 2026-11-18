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

/* Physical command DMA isolates polling semantics from translation tests. */
static void wait_ring(QTestState *qts, uint32_t flags, uint64_t address,
                      uint32_t reference, uint32_t mask)
{
    uint32_t words[] = {
        0x70c80004, 0x0b, 3, 0x20000000, 0,
        0x70bc8006, flags, address, address >> 32, reference, mask, 16,
        0x48880001, 0x13572468,
    };

    qtest_writel(qts, GPU + 0x20c8, 1);
    qtest_writel(qts, SMMU, CR0_FAULTS | 1);
    qtest_writeq(qts, GPU + 0x2000, RING_RAM);
    qtest_writel(qts, GPU + 0x2008, 13 | (1U << 27));
    qtest_writel(qts, GPU + 0x2018, 0);
    qtest_writel(qts, GPU + 0x201c, 0);
    qtest_writel(qts, GPU + 0x22000, 0);
    for (unsigned i = 0; i < G_N_ELEMENTS(words); i++) {
        qtest_writel(qts, RING_RAM + i * 4, words[i]);
    }
    qtest_writel(qts, GPU + 0x201c, G_N_ELEMENTS(words));
    qtest_writel(qts, GPU + 0x20c8, 0);
}

static void test_wait_reg_mem(void)
{
    QTestState *qts = start_oemsk();
    const uint64_t operand = RING_RAM + 0x1000;
    static const struct {
        unsigned function;
        uint32_t initial, final, reference;
    } comparisons[] = {
        { 1, 8, 6, 7 }, { 2, 8, 7, 7 }, { 3, 6, 7, 7 },
        { 4, 7, 8, 7 }, { 5, 6, 7, 7 }, { 6, 6, 8, 7 },
    };

    for (unsigned i = 0; i < G_N_ELEMENTS(comparisons); i++) {
        qtest_writel(qts, operand, comparisons[i].initial);
        wait_ring(qts, 0x10 | comparisons[i].function, operand,
                   comparisons[i].reference, UINT32_MAX);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
        qtest_clock_step(qts, 2000000);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x22000), ==, 0);
        qtest_writel(qts, operand, comparisons[i].final);
        qtest_clock_step(qts, 2000000);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 14);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x22000), ==, 0x13572468);
    }
    /* Mask applies to the observed operand; signed comparison is explicit. */
    qtest_writel(qts, operand, 0x80000007);
    wait_ring(qts, 0x13, operand, 7, 0xff);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 14);
    wait_ring(qts, 0x19, operand, 0, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 14);
    wait_ring(qts, 0x11, operand, 0, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    /* Stored registers use indices, and a CPU write can satisfy the poll. */
    qtest_writel(qts, GPU + 0xe000 * 4, 0);
    wait_ring(qts, 3, 0xe000, 0xab, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    qtest_writel(qts, GPU + 0xe000 * 4, 0xab);
    qtest_clock_step(qts, 2000000);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 14);
    /* Bad modes, address overflow, MMIO and unaligned memory stay pending. */
    for (unsigned flags = 0x23; flags <= 0x33; flags += 0x10) {
        wait_ring(qts, flags, operand, 7, UINT32_MAX);
        qtest_clock_step(qts, 2000000);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x22000), ==, 0);
    }
    wait_ring(qts, 0x17, operand, 7, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    wait_ring(qts, 0x10, operand, 0, 0);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 14);
    wait_ring(qts, 0x113, operand, 7, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    wait_ring(qts, 0x13, operand + 1, 7, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    wait_ring(qts, 0x13, GPU, 7, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    wait_ring(qts, 3, UINT64_C(1) << 32 | 0xe000, 0xab, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    /* Coherent whole-cache invalidation orders earlier writes and waits. */
    qtest_writel(qts, GPU + 0x20c8, 1);
    qtest_writel(qts, GPU + 0x2018, 0);
    qtest_writel(qts, GPU + 0x201c, 0);
    qtest_writel(qts, RING_RAM, 0x400e9501);
    qtest_writel(qts, RING_RAM + 4, 0x12);
    qtest_writel(qts, RING_RAM + 8, 0x48880001);
    qtest_writel(qts, RING_RAM + 12, 0x24681357);
    qtest_writel(qts, GPU + 0x201c, 4);
    qtest_writel(qts, GPU + 0x20c8, 0);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 4);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x22000), ==, 0x24681357);
    qtest_writel(qts, GPU + 0x20c8, 1);
    qtest_writel(qts, GPU + 0x2018, 0);
    qtest_writel(qts, RING_RAM + 4, 0x13);
    qtest_writel(qts, GPU + 0x20c8, 0);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 0);
    /* Halt cancels a scheduled wait; no subsequent marker is completed. */
    qtest_writel(qts, operand, 0);
    wait_ring(qts, 0x13, operand, 7, UINT32_MAX);
    qtest_writel(qts, GPU + 0x20c8, 1);
    qtest_writel(qts, operand, 7);
    qtest_clock_step(qts, 2000000);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x22000), ==, 0);
    qtest_quit(qts);
}

static void test_cp_smmu_sync(void)
{
    QTestState *qts = start_oemsk();
    static const uint32_t words[] = {
        0x70c80004, 0x0b, 3, 0x20000000, 0,
        0x49218601, 1, /* Original CP write to CB0.TLBIALL. */
        0x4121fc01, 1, /* Original CP write to CB0.TLBSYNC. */
        0x70bc8006, 3, 0x121fd, 0, 0, 1, 1,
        0x70138000, /* WAIT_FOR_ME */
        0x48880001, 0x13572468,
    };

    qtest_writel(qts, SMMU, CR0_FAULTS | 1);
    qtest_writeq(qts, CB(0) + 0x20, ROOT);
    qtest_writel(qts, CB(0) + 0x5c, 2);
    qtest_writel(qts, CB(0) + 0x7f4, 1);
    g_assert_cmphex(qtest_readl(qts, CB(0) + 0x7f4), ==, 0);
    qtest_writeq(qts, GPU + 0x2000, RING_RAM);
    qtest_writel(qts, GPU + 0x2008, 13 | (1U << 27));
    for (unsigned i = 0; i < G_N_ELEMENTS(words); i++) {
        qtest_writel(qts, RING_RAM + i * 4, words[i]);
    }
    qtest_writel(qts, GPU + 0x201c, G_N_ELEMENTS(words));
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==,
                   G_N_ELEMENTS(words));
    g_assert_cmphex(qtest_readl(qts, GPU + 0x22000), ==, 0x13572468);
    g_assert_cmphex(fault(qts, 0), ==, 2);
    g_assert_cmphex(qtest_readq(qts, CB(0) + 0x20), ==, ROOT);
    g_assert_cmphex(qtest_readl(qts, CB(0) + 0x618), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CB(0) + 0x7f0), ==, 0);
    /* A burst containing an unsupported SMMU register remains pending. */
    qtest_writel(qts, GPU + 0x20c8, 1);
    qtest_writel(qts, GPU + 0x2018, 5);
    qtest_writel(qts, RING_RAM + 20, 0x49218602);
    qtest_writel(qts, RING_RAM + 28, 0xdeadbeef);
    qtest_writel(qts, GPU + 0x20c8, 0);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, 5);
    g_assert_cmphex(qtest_readq(qts, CB(0) + 0x20), ==, ROOT);
    qtest_quit(qts);
}

static void test_adreno_edge_irq(void)
{
    QTestState *qts = start_oemsk();
    const uint64_t gic = 0x0b000000;
    const uint64_t config = gic + 0xc00 + (332 / 16) * 4;
    const uint64_t pending = gic + 0x200 + (332 / 32) * 4;
    const uint32_t bit = 1U << (332 % 32);
    static const uint32_t words[] = {
        0x70c80004, 0x0b, 3, 0x20000000, 0,
        0x70400001, 0x80000000,
    };

    /* Exercise a guest-configured edge input as well as native level use. */
    qtest_writeb(qts, gic + 0x800 + 332, 1);
    qtest_writel(qts, config, 2U << ((332 % 16) * 2));
    qtest_writel(qts, SMMU, CR0_FAULTS | 1);
    qtest_writeq(qts, GPU + 0x2000, RING_RAM);
    qtest_writel(qts, GPU + 0x2008, 13 | (1U << 27));
    qtest_writel(qts, GPU + 0xe0, 1U << 15);
    for (unsigned i = 0; i < G_N_ELEMENTS(words); i++) {
        qtest_writel(qts, RING_RAM + i * 4, words[i]);
    }
    qtest_writel(qts, GPU + 0x201c, G_N_ELEMENTS(words));
    g_assert_true(qtest_get_irq(qts, 332 - 32));
    g_assert_cmphex(qtest_readl(qts, GPU + 0x1380), ==, 1U << 15);
    qtest_writel(qts, GPU + 0x1380, UINT32_MAX);
    qtest_writel(qts, GPU + 0x1384, 0);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x1380), ==, 1U << 15);
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, bit);
    qtest_writel(qts, pending + 0x80, bit);
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, 0);
    /* Rewriting an unchanged mask cannot synthesize another rising edge. */
    qtest_writel(qts, GPU + 0xe0, 1U << 15);
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, 0);
    qtest_writel(qts, GPU + 0xdc, 1U << 13);
    g_assert_true(qtest_get_irq(qts, 332 - 32));
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, 0);
    qtest_writel(qts, GPU + 0xe0, 0);
    g_assert_false(qtest_get_irq(qts, 332 - 32));
    g_assert_cmphex(qtest_readl(qts, GPU + 0x1380), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x1384), ==, 1U << 15);
    qtest_writel(qts, GPU + 0xe0, 1U << 15);
    g_assert_cmphex(qtest_readl(qts, pending) & bit, ==, bit);
    qtest_writel(qts, GPU + 0xdc, 1U << 15);
    g_assert_false(qtest_get_irq(qts, 332 - 32));
    g_assert_cmphex(qtest_readl(qts, GPU + 0x1380), ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x1384), ==, 0);
    qtest_quit(qts);
}

static unsigned rect_parity(unsigned value)
{
    unsigned parity = 1;

    while (value) {
        parity ^= value & 1;
        value >>= 1;
    }
    return parity;
}

static void rect_packet(uint32_t *words, unsigned *n, unsigned opcode,
                         const uint32_t *data, unsigned count)
{
    words[(*n)++] = 0x70000000 | (opcode << 16) | count |
                    (rect_parity(count) << 15) | (rect_parity(opcode) << 23);
    for (unsigned i = 0; i < count; i++) {
        words[(*n)++] = data[i];
    }
}

static void rect_register(uint32_t *words, unsigned *n,
                           unsigned reg, uint32_t value)
{
    words[(*n)++] = 0x40000001 | (reg << 8) | (rect_parity(reg) << 27);
    words[(*n)++] = value;
}

static void test_rectangle(void)
{
    QTestState *qts = start_oemsk();
    const uint64_t pixels = 0x43000000, fence = 0x43001000;
    uint32_t words[512] = { 0 };
    uint32_t vs[35] = { 0 }, fs[35] = { 0 };
    uint32_t constants[] = {
        (2U << 22) | (8U << 18), 1, 0,
        0x40000000, 0x3f800000, 0, 0x3f800000,
        0x40c00000, 0x40800000, 0, 0x3f800000,
    };
    uint32_t color[] = {
        (2U << 22) | (12U << 18), 1, 0,
        0x3f000000, 0x3f800000, 0, 0x3f800000,
        0x3f800001, 0xbf800002, 0x56800000, 0,
    };
    static const uint32_t init[] = { 0x0b, 3, 0x20000000, 0 };
    static const uint32_t mode[] = { 1 };
    static const uint32_t disabled[] = { 0 };
    static const uint32_t lrz_flush[] = { 0x26 };
    static const uint32_t draw[] = { 0x888, 1, 2 };
    static const uint32_t finish[] = { 0x43001000, 0, 0x12345678 };
    static const struct { unsigned reg; uint32_t value; } config[] = {
        { 0xe000, 0x181 }, { 0xe140, 0x20000 },
        { 0xe146, 1 }, { 0xe147, 0xf }, { 0xe150, 0x780 },
        { 0xe152, 0x30 }, { 0xe153, 1 }, { 0xe154, 8 },
        { 0xe155, 0x43000000 }, { 0xe1a9, 0xffff0000 },
        { 0xe401, 0xfcfc03 }, { 0xe584, 1 }, { 0xe585, 1 },
        { 0xe592, 1 }, { 0xe593, 0xf04 }, { 0xe5ac, 1 },
        { 0xe5c3, 1 }, { 0xe5ca, 1 }, { 0xe5d3, 0x30 },
        { 0xe5dc, 0x23c00004 }, /* Inactive prefetch command may be stale. */
        { 0xe0ab, 0x70007 }, { 0xe0cb, 0x70007 }, { 0xe0eb, 0x70007 },
    };
    unsigned n = 0, position_word, skip_word, local_word, override_word;
    unsigned constant_packet;
    uint8_t actual[512], expected[512];

    /* Synthetic VS: vertex ID << 2 selects a vec4 of position constants. */
    vs[0] = (1U << 22) | (8U << 18);
    vs[3] = 0x20020003; vs[4] = 0x56d81803;
    vs[5] = 3; vs[6] = 0x20150000;
    vs[7] = 0; vs[8] = 0x201100f4;
    vs[9] = 0xc00; vs[10] = 0x20154b04;
    vs[11] = 0; vs[12] = 0x03000000;
    /* Synthetic FS: repeat four constant moves, then END. */
    fs[0] = (1U << 22) | (12U << 18);
    fs[3] = 0; fs[4] = 0x30355b00;
    fs[5] = 4; fs[6] = 0x20354005;
    fs[7] = 0x10051004; fs[8] = 0x63828004;
    fs[9] = 0x10060004; fs[10] = 0x40700004;
    fs[11] = 0x00040000; fs[12] = 0x40100000;
    fs[13] = 0; fs[14] = 0x03000000;
    rect_packet(words, &n, 0x48, init, G_N_ELEMENTS(init));
    rect_packet(words, &n, 0x6c, mode, G_N_ELEMENTS(mode));
    skip_word = n + 1;
    rect_packet(words, &n, 0x1d, disabled, G_N_ELEMENTS(disabled));
    local_word = n + 1;
    rect_packet(words, &n, 0x23, disabled, G_N_ELEMENTS(disabled));
    override_word = n + 1;
    rect_packet(words, &n, 0x64, mode, G_N_ELEMENTS(mode));
    rect_packet(words, &n, 0x46, lrz_flush, G_N_ELEMENTS(lrz_flush));
    for (unsigned i = 0; i < G_N_ELEMENTS(config); i++) {
        rect_register(words, &n, config[i].reg, config[i].value);
    }
    rect_packet(words, &n, 0x30, vs, G_N_ELEMENTS(vs));
    rect_packet(words, &n, 0x30, fs, G_N_ELEMENTS(fs));
    constant_packet = n;
    position_word = n + 4 + 4;
    rect_packet(words, &n, 0x30, constants, G_N_ELEMENTS(constants));
    rect_packet(words, &n, 0x30, color, G_N_ELEMENTS(color));
    rect_packet(words, &n, 0x38, draw, G_N_ELEMENTS(draw));
    rect_packet(words, &n, 0x3d, finish, G_N_ELEMENTS(finish));
    qtest_writel(qts, SMMU, CR0_FAULTS | 1);
    qtest_writeq(qts, GPU + 0x2000, RING_RAM);
    qtest_writel(qts, GPU + 0x2008, 13 | (1U << 27));
    for (unsigned trial = 0; trial < 5; trial++) {
        qtest_writel(qts, GPU + 0x20c8, 1);
        qtest_writel(qts, GPU + 0x2018, 0);
        qtest_writel(qts, GPU + 0x201c, 0);
        memset(expected, 0xa5, sizeof(expected));
        qtest_memwrite(qts, pixels, expected, sizeof(expected));
        qtest_writel(qts, fence, 0);
        words[position_word] = 0x40c00000;
        words[skip_word] = trial >= 2;
        words[local_word] = trial == 2 || trial == 3;
        words[override_word] = trial != 2 && trial != 4;
        if (trial == 1) {
            /* A fractional screen coordinate is outside the supported path. */
            words[position_word] = 0x40d00000;
        }
        for (unsigned i = 0; i < n; i++) {
            qtest_writel(qts, RING_RAM + i * 4, words[i]);
        }
        qtest_writel(qts, GPU + 0x201c, n);
        qtest_writel(qts, GPU + 0x20c8, 0);
        if (trial == 0 || trial >= 3) {
            g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, n);
            g_assert_cmphex(qtest_readl(qts, fence), ==, 0x12345678);
            for (unsigned y = 1; y < 4; y++) {
                for (unsigned x = 2; x < 6; x++) {
                    memcpy(expected + y * 64 + x * 4, "\x80\xff\0\xff", 4);
                }
            }
        } else {
            g_assert_cmphex(qtest_readl(qts, fence), ==, 0);
        }
        qtest_memread(qts, pixels, actual, sizeof(actual));
        g_assert_cmpmem(actual, sizeof(actual), expected, sizeof(expected));
    }
    /* Fetch the same vertex constants through a shared UBO table. */
    for (unsigned trial = 0; trial < 3; trial++) {
        uint32_t table[] = {
            (1U << 22) | (6U << 18) | 1, 2, 0, 0x44003000, 0
        };
        uint32_t auxiliary[] = {
            (1U << 22) | (6U << 18), 1, 0, 2U << 16, 0
        };
        uint32_t indirect[] = {
            (2U << 22) | (8U << 18) | (3U << 16), 0x10011, 0
        };
        uint32_t previous = constants[7];

        n = constant_packet;
        words[skip_word] = words[local_word] = 0;
        words[override_word] = 1;
        if (trial == 1) {
            constants[7] = 0x40d00000; /* Read the changed buffer contents. */
        } else if (trial == 2) {
            /* Metadata stages, but the fetch cannot read MMIO. */
            table[3] = GPU;
        }
        for (unsigned i = 0; i < 8; i++) {
            qtest_writel(qts, 0x44003010 + i * 4, constants[3 + i]);
        }
        constants[7] = previous;
        rect_packet(words, &n, 0x30, table, G_N_ELEMENTS(table));
        rect_packet(words, &n, 0x30, auxiliary, G_N_ELEMENTS(auxiliary));
        rect_packet(words, &n, 0x30, indirect, G_N_ELEMENTS(indirect));
        rect_packet(words, &n, 0x30, color, G_N_ELEMENTS(color));
        rect_packet(words, &n, 0x38, draw, G_N_ELEMENTS(draw));
        rect_packet(words, &n, 0x3d, finish, G_N_ELEMENTS(finish));
        qtest_writel(qts, GPU + 0x20c8, 1);
        qtest_writel(qts, GPU + 0x2018, 0);
        qtest_writel(qts, GPU + 0x201c, 0);
        memset(expected, 0xa5, sizeof(expected));
        qtest_memwrite(qts, pixels, expected, sizeof(expected));
        qtest_writel(qts, fence, 0);
        for (unsigned i = 0; i < n; i++) {
            qtest_writel(qts, RING_RAM + i * 4, words[i]);
        }
        qtest_writel(qts, GPU + 0x201c, n);
        qtest_writel(qts, GPU + 0x20c8, 0);
        if (!trial) {
            g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, n);
            g_assert_cmphex(qtest_readl(qts, fence), ==, 0x12345678);
            for (unsigned y = 1; y < 4; y++) {
                for (unsigned x = 2; x < 6; x++) {
                    memcpy(expected + y * 64 + x * 4, "\x80\xff\0\xff", 4);
                }
            }
        } else {
            g_assert_cmphex(qtest_readl(qts, fence), ==, 0);
        }
        qtest_memread(qts, pixels, actual, sizeof(actual));
        g_assert_cmpmem(actual, sizeof(actual), expected, sizeof(expected));
    }
    qtest_quit(qts);
}

static void test_gmem_resolve(void)
{
    QTestState *qts = start_oemsk();
    const uint64_t pixels = 0x43000000, fence = 0x43003000;
    const uint64_t destination = RING_IOVA + 0x2000;
    const uint64_t fence_iova = SHADOW_IOVA & ~UINT64_C(0xfff);
    const uint32_t init[] = { 0x0b, 3, 0x20000000, 0 };
    uint32_t event[] = {
        0x1e, (uint32_t)fence_iova, fence_iova >> 32, 0x1111
    };
    uint32_t words[128];
    uint8_t actual[10240], expected[10240];
    unsigned n, clear_start, resolve_start;

    set_context(qts, 0, HUPCF | CFIE);
    set_ring(qts);
    qtest_writeq(qts, L3 + 5 * 8, RING_RAM | AF | 3);
    qtest_writeq(qts, L3, fence | AF | 3);
    for (unsigned trial = 0; trial < 8; trial++) {
        n = 0;
        event[3] = 0x1111;
        rect_packet(words, &n, 0x48, init, G_N_ELEMENTS(init));
        rect_register(words, &n, 0xe00, 0x80);
        rect_register(words, &n, 0xe01, 0);
        rect_register(words, &n, 0xc82, 0);
        rect_register(words, &n, 0xd08, 0);
        rect_register(words, &n, 0xd09, 0);
        rect_register(words, &n, 0xe62, 0);
        /* A 64x32 logical bin; fast clear ignores a stale sysmem pitch. */
        rect_register(words, &n, 0xe140, 0x202);
        rect_register(words, &n, 0xe141, trial == 4 ? 1U << 16 : 0);
        rect_register(words, &n, 0xe142, trial == 3 ? 1 : 0);
        rect_register(words, &n, 0xe143, 0);
        rect_register(words, &n, 0xe152, trial == 5 ? 0x2330 : 0x2230);
        rect_register(words, &n, 0xe153, 0x5a);
        rect_register(words, &n, 0xe155, trial == 2 ? 0xff000 : 0);
        rect_register(words, &n, 0xe156, 0);
        rect_register(words, &n, 0xe1d0, trial == 7 ? 16 : 0);
        rect_register(words, &n, 0xe210, 0);
        rect_register(words, &n, 0xe211, trial == 7 ? 16 : 0);
        rect_register(words, &n, 0xe212, trial == 7 ? 0x1f004f : 0x1f003f);
        rect_register(words, &n, 0xe213, trial == 7 ? 4 : 0);
        rect_register(words, &n, 0xe218, 0x11223344);
        rect_register(words, &n, 0xe21c, 0xf2);
        clear_start = n;
        rect_packet(words, &n, 0x46, event, G_N_ELEMENTS(event));
        /* Resolve a partial rectangle into rows with 64 bytes of padding. */
        rect_register(words, &n, 0xe211, trial == 7 ? 0x10012 : 0x10002);
        rect_register(words, &n, 0xe212, trial == 7 ? 0x1e004d : 0x1e003d);
        rect_register(words, &n, 0xe213, trial == 6 ? 5 : 4);
        rect_register(words, &n, 0xe214, (uint32_t)destination);
        rect_register(words, &n, 0xe215, destination >> 32);
        rect_register(words, &n, 0xe216, 5);
        rect_register(words, &n, 0xe217, 0);
        rect_register(words, &n, 0xe21c, 0);
        resolve_start = n;
        event[3] = 0x2222;
        rect_packet(words, &n, 0x46, event, G_N_ELEMENTS(event));
        qtest_writel(qts, GPU + 0x20c8, 1);
        qtest_writel(qts, GPU + 0x2018, 0);
        qtest_writel(qts, GPU + 0x201c, 0);
        qtest_writel(qts, fence, 0);
        memset(expected, 0xa5, sizeof(expected));
        qtest_memwrite(qts, pixels, expected, sizeof(expected));
        for (unsigned page = 0; page < 3; page++) {
            qtest_writeq(qts, L3 + (7 + page) * 8,
                         trial == 1 && page == 2 ? 0 :
                         (pixels + page * 4096) | AF | 3);
        }
        for (unsigned i = 0; i < n; i++) {
            qtest_writel(qts, RING_RAM + i * 4, words[i]);
        }
        qtest_writel(qts, GPU + 0x201c, n);
        qtest_writel(qts, GPU + 0x20c8, 0);
        g_assert_cmphex(qtest_readl(qts, GPU + 0xe00 * 4), ==, 0x80);
        if (trial && trial != 7) {
            g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==,
                           trial == 1 || trial == 6 ?
                           resolve_start : clear_start);
            g_assert_cmphex(qtest_readl(qts, fence), ==,
                           trial == 1 || trial == 6 ? 0x1111 : 0);
            qtest_memread(qts, pixels, actual, sizeof(actual));
            g_assert_cmpmem(actual, sizeof(actual), expected, sizeof(expected));
        }
        if (trial == 1) {
            /* A fault in a later row must leave even the first row intact. */
            g_assert_cmphex(fault(qts, 0), ==, 2);
            g_assert_cmphex(qtest_readq(qts, CB(0) + CB_FAR), ==,
                           destination + 8192);
            g_assert_cmphex(qtest_readl(qts, CB(0) + FSYNR0) & (1U << 4),
                           !=, 0);
            g_assert_true(qtest_get_irq(qts, 361 - 32));
            qtest_writeq(qts, L3 + 9 * 8, (pixels + 8192) | AF | 3);
            clear_fault(qts, 0);
            qtest_writel(qts, GPU + 0x201c, n);
        }
        if (trial <= 1 || trial == 7) {
            g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, n);
            g_assert_cmphex(qtest_readl(qts, fence), ==, 0x2222);
            for (unsigned y = 1; y <= 30; y++) {
                for (unsigned x = trial == 7 ? 18 : 2;
                     x <= (trial == 7 ? 77 : 61); x++) {
                    memcpy(expected + y * 320 + x * 4, "\x22\x33\x44\x11", 4);
                }
            }
            qtest_memread(qts, pixels, actual, sizeof(actual));
            g_assert_cmpmem(actual, sizeof(actual), expected, sizeof(expected));
        }
    }
    qtest_quit(qts);
}

static void test_texture_rectangle(void)
{
    QTestState *qts = start_oemsk();
    uint32_t words[256], vs[35] = { 1U << 22 | 8U << 18 };
    uint32_t fs[35] = { 1U << 22 | 12U << 18, 0, 0, 0, 0x03000000 };
    const uint32_t program[] = {
        0x20020003, 0x56d81803, 3, 0x20150000,
        0, 0x201100f4, 0xc21, 0x20154004, 0xc20, 0x20154003,
        0xc23, 0x20154008, 0xc22, 0x20156007,
        0, 0x20554005, 0x3f800000, 0x20554006, 0, 0x03000000,
    };
    const uint32_t constants[] = {
        2U << 22 | 8U << 18 | 8, 1, 0,
        0x40000000, 0x3f800000, 0xbf800000, 0,
        0x40c00000, 0x40800000, 0x40400000, 0x40400000,
    };
    const uint32_t sampler[] = {
        1U << 22 | 4U << 18, 0, 0, 0x60920, 0x67, 0, 0
    };
    uint32_t texture[] = {
        1U << 22 | 4U << 18, 1, 0,
        0x0c006880, 8 | 8 << 15, 0x20000000 | 64 << 7, 0,
        0x44000000, 0x20000, 0, 0, 0, 0, 0, 0,
    };
    const uint32_t init[] = { 0x0b, 3, 0x20000000, 0 };
    const uint32_t mode[] = { 1 }, draw[] = { 0x888, 1, 2 };
    const uint32_t finish[] = { 0x43001000, 0, 0x12345678 };
    const struct { unsigned reg; uint32_t value; } config[] = {
        { 0xe000, 0x181 }, { 0xe140, 0x20000 },
        { 0xe146, 1 }, { 0xe147, 0xf }, { 0xe150, 0x780 },
        { 0xe152, 0x30 }, { 0xe153, 1 }, { 0xe154, 8 },
        { 0xe155, 0x43000000 }, { 0xe1a9, 0xffff0000 },
        { 0xe401, 0xfcfc03 }, { 0xe584, 1 }, { 0xe585, 1 },
        { 0xe592, 2 }, { 0xe593, 0x0f030307 }, { 0xe5a3, 4 },
        { 0xe5ac, 1 }, { 0xe5c3, 1 }, { 0xe5ca, 1 },
        { 0xe5d3, 0x30 }, { 0xe5db, 0x19 }, { 0xe5dc, 0x23c00004 },
        { 0xe791, 4 }, { 0xe792, 2 }, { 0xe1d0, 0x7800480 },
        { 0xe0ab, 0x70007 }, { 0xe0cb, 0x70007 }, { 0xe0eb, 0x70007 },
    };
    const unsigned order[4][4] = {
        { 0, 1, 2, 3 }, { 2, 1, 0, 3 },
        { 3, 0, 1, 2 }, { 3, 2, 1, 0 },
    };
    uint8_t source[512], actual[4096], expected[4096];

    memcpy(vs + 3, program, sizeof(program));
    for (unsigned i = 0; i < sizeof(source); i++) {
        source[i] = i * 37 + i / 64;
    }
    qtest_memwrite(qts, 0x44000000, source, sizeof(source));
    qtest_writel(qts, SMMU, CR0_FAULTS | 1);
    qtest_writeq(qts, GPU + 0x2000, RING_RAM);
    qtest_writel(qts, GPU + 0x2008, 13 | (1U << 27));
    for (unsigned trial = 0; trial < 20; trial++) {
        unsigned n = 0, swap = trial & 3, tswap = (trial >> 2) & 3;
        bool r8 = trial == 17 || trial == 18;
        bool tiled = trial >= 18;
        unsigned pitch = tiled ? (r8 ? 128 : 256) : 64;
        unsigned length = tiled ? sizeof(expected) : 512;

        qtest_writel(qts, GPU + 0x20c8, 1);
        qtest_writel(qts, GPU + 0x2018, 0);
        qtest_writel(qts, GPU + 0x201c, 0);
        qtest_writel(qts, 0x43001000, 0);
        memset(expected, 0xa5, sizeof(expected));
        qtest_memwrite(qts, 0x43000000, expected, length);
        rect_packet(words, &n, 0x48, init, G_N_ELEMENTS(init));
        rect_packet(words, &n, 0x6c, mode, G_N_ELEMENTS(mode));
        rect_packet(words, &n, 0x64, mode, G_N_ELEMENTS(mode));
        for (unsigned i = 0; i < G_N_ELEMENTS(config); i++) {
            rect_register(words, &n, config[i].reg, config[i].value);
        }
        if (trial >= 17) {
            swap = tswap = 0;
        }
        rect_register(words, &n, 0xe152,
                      (r8 ? 3 : 0x30) | swap << 13 | (tiled ? 0x300 : 0));
        rect_register(words, &n, 0xe153, pitch / 64);
        rect_register(words, &n, 0xe154, length / 64);
        rect_register(words, &n, 0xe5d3, r8 ? 3 : 0x30);
        texture[3] = (r8 ? 0x00c0b200 : 0x0c006880) |
                     tswap << 30 | (trial == 16 ? 2 : 0);
        rect_packet(words, &n, 0x30, vs, G_N_ELEMENTS(vs));
        rect_packet(words, &n, 0x30, fs, G_N_ELEMENTS(fs));
        if (trial == 18) {
            const uint32_t origin[] = {
                2U << 22 | 8U << 18 | 8, 1, 0, 0, 0, 0, 0,
                0x3f800000, 0x3f800000, 0x3f800000, 0x3f800000,
            };

            rect_packet(words, &n, 0x30, origin, G_N_ELEMENTS(origin));
        } else {
            rect_packet(words, &n, 0x30, constants, G_N_ELEMENTS(constants));
        }
        rect_packet(words, &n, 0x30, sampler, G_N_ELEMENTS(sampler));
        rect_packet(words, &n, 0x30, texture, G_N_ELEMENTS(texture));
        rect_packet(words, &n, 0x38, draw, G_N_ELEMENTS(draw));
        rect_packet(words, &n, 0x3d, finish, G_N_ELEMENTS(finish));
        for (unsigned i = 0; i < n; i++) {
            qtest_writel(qts, RING_RAM + i * 4, words[i]);
        }
        qtest_writel(qts, GPU + 0x201c, n);
        qtest_writel(qts, GPU + 0x20c8, 0);
        g_assert_cmphex(qtest_readl(qts, 0x43001000), ==,
                       trial == 16 || trial == 19 ? 0 : 0x12345678);
        if (trial == 18) {
            expected[0] = source[0];
        } else if (trial != 16 && trial != 19) {
            g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, n);
            for (unsigned y = 1; y < 4; y++) {
                for (unsigned x = 2; x < 6; x++) {
                    unsigned tx = x == 2 ? 0 : x - 3;
                    uint8_t logical[4];
                    unsigned offset = y * pitch + x * (r8 ? 1 : 4);

                    if (r8) {
                        expected[offset] = source[(y - 1) * 64 + tx];
                        continue;
                    }
                    for (unsigned c = 0; c < 4; c++) {
                        logical[order[tswap][c]] =
                            source[(y - 1) * 64 + tx * 4 + c];
                    }
                    for (unsigned c = 0; c < 4; c++) {
                        expected[offset + c] = logical[order[swap][c]];
                    }
                }
            }
        }
        qtest_memread(qts, 0x43000000, actual, length);
        g_assert_cmpmem(actual, length, expected, length);
    }
    qtest_quit(qts);
}

static void test_draw_state_execution(void)
{
    QTestState *qts = start_oemsk();
    const uint64_t group_iova = (SHADOW_IOVA & ~UINT64_C(0xfff)) + 0x2000;
    const uint64_t group_ram = 0x43000000;
    const uint64_t next_ram = 0x43200000;
    const uint64_t fence = 0x43002000;
    const uint32_t init[] = { 0x0b, 3, 0x20000000, 0 };
    const uint32_t draw[] = { 0x888, 1, 2 };
    const uint32_t marker[] = { SHADOW_IOVA & ~0xfffU, 8, 0x1234 };
    uint32_t state[] = {
        0x01000002, (uint32_t)group_iova, group_iova >> 32
    };
    uint32_t words[64], n;
    unsigned draw_start;

    set_context(qts, 0, HUPCF | CFIE);
    set_ring(qts);
    qtest_writeq(qts, L3 + 5 * 8, RING_RAM | AF | 3);
    qtest_writeq(qts, L3 + 2 * 8, group_ram | AF | 3);
    qtest_writeq(qts, L3, fence | AF | 3);
    qtest_writel(qts, group_ram, 0x48e21801);
    for (unsigned trial = 0; trial < 4; trial++) {
        qtest_writel(qts, group_ram + 4, 0x10 + trial);
        state[0] = 0x01000002 | (trial == 2 ? 1U << 16 : 0);
        if (trial == 3) {
            state[0] = 0x01020000; /* Disable without fetching the buffer. */
            state[1] = state[2] = 0;
        }
        n = 0;
        rect_packet(words, &n, 0x48, init, G_N_ELEMENTS(init));
        rect_packet(words, &n, 0x43, state, G_N_ELEMENTS(state));
        draw_start = n;
        rect_packet(words, &n, 0x38, draw, G_N_ELEMENTS(draw));
        rect_packet(words, &n, 0x3d, marker, G_N_ELEMENTS(marker));
        qtest_writel(qts, GPU + 0x20c8, 1);
        qtest_writel(qts, GPU + 0x2018, 0);
        qtest_writel(qts, GPU + 0x201c, 0);
        for (unsigned i = 0; i < n; i++) {
            qtest_writel(qts, RING_RAM + i * 4, words[i]);
        }
        qtest_writel(qts, GPU + 0x201c, n);
        qtest_writel(qts, GPU + 0x20c8, 0);
        /* State executes, but the unconfigured rasterizer cannot finish. */
        g_assert_cmphex(qtest_readl(qts, GPU + 0xe218 * 4), ==,
                       trial < 2 ? 0x10 : 0x12);
        g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, draw_start);
        g_assert_cmphex(qtest_readl(qts, fence), ==, 0);
    }
    /* A Type4 payload crosses into a missing, noncontiguous source page. */
    qtest_writel(qts, group_ram + 0xff8, 0x48e21802);
    qtest_writel(qts, group_ram + 0xffc, 0x11223344);
    qtest_writel(qts, next_ram, 0x55667788);
    state[0] = 0x1f000003;
    state[1] = (uint32_t)(group_iova + 0xff8);
    state[2] = group_iova >> 32;
    n = 0;
    rect_packet(words, &n, 0x43, state, G_N_ELEMENTS(state));
    draw_start = n;
    rect_packet(words, &n, 0x38, draw, G_N_ELEMENTS(draw));
    rect_packet(words, &n, 0x3d, marker, G_N_ELEMENTS(marker));
    qtest_writel(qts, GPU + 0x20c8, 1);
    qtest_writel(qts, GPU + 0x2018, 0);
    qtest_writel(qts, GPU + 0x201c, 0);
    for (unsigned i = 0; i < n; i++) {
        qtest_writel(qts, RING_RAM + i * 4, words[i]);
    }
    qtest_writel(qts, GPU + 0x201c, n);
    qtest_writel(qts, GPU + 0x20c8, 0);
    g_assert_cmphex(fault(qts, 0), ==, 2);
    g_assert_cmphex(qtest_readq(qts, CB(0) + CB_FAR), ==, group_iova + 0x1000);
    g_assert_true(qtest_get_irq(qts, 361 - 32));
    g_assert_cmphex(qtest_readl(qts, GPU + 0xe218 * 4), ==, 0x12);
    g_assert_cmphex(qtest_readl(qts, fence), ==, 0);
    qtest_writeq(qts, L3 + 3 * 8, next_ram | AF | 3);
    clear_fault(qts, 0);
    qtest_writel(qts, GPU + 0x201c, n);
    g_assert_cmphex(qtest_readl(qts, GPU + 0xe218 * 4), ==, 0x11223344);
    g_assert_cmphex(qtest_readl(qts, GPU + 0xe219 * 4), ==, 0x55667788);
    g_assert_cmphex(qtest_readl(qts, GPU + 0x2018), ==, draw_start);
    g_assert_cmphex(qtest_readl(qts, fence), ==, 0);
    qtest_quit(qts);
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
    qtest_add_func("/oemsk/wait-reg-mem", test_wait_reg_mem);
    qtest_add_func("/oemsk/adreno-edge-irq", test_adreno_edge_irq);
    qtest_add_func("/oemsk/cp-smmu-sync", test_cp_smmu_sync);
    qtest_add_func("/oemsk/rectangle", test_rectangle);
    qtest_add_func("/oemsk/texture-rectangle", test_texture_rectangle);
    qtest_add_func("/oemsk/gmem-resolve", test_gmem_resolve);
    qtest_add_func("/oemsk/draw-state-execution", test_draw_state_execution);
    return g_test_run();
}
