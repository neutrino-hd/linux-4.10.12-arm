/*
 * ARM-specific support for Broadcom STB S2/S3/S5 power management
 *
 * S2: clock gate CPUs and as many peripherals as possible
 * S3: power off all of the chip except the Always ON (AON) island; keep DDR is
 *     self-refresh
 * S5: (a.k.a. S3 cold boot) much like S3, except DDR is powered down, so we
 *     treat this mode like a soft power-off, with wakeup allowed from AON
 *
 * Copyright © 2014-2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "brcmstb-pm: " fmt

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/suspend.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/compiler.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/kconfig.h>
#include <linux/memblock.h>
#include <linux/sort.h>
#include <linux/notifier.h>
#include <asm/fncpy.h>
#include <asm/suspend.h>
#include <asm/setup.h>

#include <linux/brcmstb/brcmstb.h>
#include <linux/brcmstb/memory_api.h>
#include <soc/brcmstb/common.h>

#include "pm.h"
#include "aon_defs.h"

extern int brcmstb_regsave_init(void);

#define SHIMPHY_DDR_PAD_CNTRL		0x8c
#define SHIMPHY_PAD_PLL_SEQUENCE	BIT(8)
#define SHIMPHY_PAD_GATE_PLL_S3		BIT(9)

#define MAX_NUM_MEMC			3

/* Capped for performance reasons */
#define MAX_HASH_SIZE			SZ_256M
/* Max per bank, to keep some fairness */
#define MAX_HASH_SIZE_BANK		SZ_64M

struct brcmstb_memc {
	void __iomem *ddr_phy_base;
	void __iomem *ddr_shimphy_base;
};

struct brcmstb_pm_control {
	void __iomem *aon_ctrl_base;
	void __iomem *aon_sram;
	struct brcmstb_memc memcs[MAX_NUM_MEMC];

	void __iomem *boot_sram;
	size_t boot_sram_len;

	bool support_warm_boot;
	size_t pll_status_offset;
	int num_memc;

	struct brcmstb_s3_params *s3_params;
	dma_addr_t s3_params_pa;
};

enum bsp_initiate_command {
	BSP_CLOCK_STOP		= 0x00,
	BSP_GEN_RANDOM_KEY	= 0x4A,
	BSP_RESTORE_RANDOM_KEY	= 0x55,
	BSP_GEN_FIXED_KEY	= 0x63,
};

#define PM_INITIATE		0x01
#define PM_INITIATE_SUCCESS	0x00
#define PM_INITIATE_FAIL	0xfe

static struct brcmstb_pm_control ctrl;

extern const unsigned long brcmstb_pm_do_s2_sz;
extern asmlinkage int brcmstb_pm_do_s2(void __iomem *aon_ctrl_base,
		void __iomem *ddr_phy_pll_status);

static int (*brcmstb_pm_do_s2_sram)(void __iomem *aon_ctrl_base,
		void __iomem *ddr_phy_pll_status);

static int brcmstb_init_sram(struct device_node *dn)
{
	void __iomem *sram;
	struct resource res;
	int ret;

	ret = of_address_to_resource(dn, 0, &res);
	if (ret)
		return ret;

	/* Cached, executable remapping of SRAM */
	sram = __arm_ioremap_exec(res.start, resource_size(&res), true);
	if (!sram)
		return -ENOMEM;

	ctrl.boot_sram = sram;
	ctrl.boot_sram_len = resource_size(&res);

	return 0;
}

/*
 * Latch into the BRCM SRAM compatible property here to be more specific than
 * the standard "mmio-sram". Could be supported with genalloc too, but that
 * would be overkill for its current single use-case.
 */
static const struct of_device_id sram_dt_ids[] = {
	{ .compatible = "brcm,boot-sram" },
	{}
};

static int do_bsp_initiate_command(enum bsp_initiate_command cmd)
{
	void __iomem *base = ctrl.aon_ctrl_base;
	int ret;
	int timeo = 1000 * 1000; /* 1 second */

	__raw_writel(0, base + AON_CTRL_PM_INITIATE);
	(void)__raw_readl(base + AON_CTRL_PM_INITIATE);

	/* Go! */
	__raw_writel((cmd << 1) | PM_INITIATE, base + AON_CTRL_PM_INITIATE);

	/*
	 * If firmware doesn't support the 'ack', then just assume it's done
	 * after 10ms. Note that this only works for command 0, BSP_CLOCK_STOP
	 */
	if (of_machine_is_compatible("brcm,bcm74371a0")) {
		(void)__raw_readl(base + AON_CTRL_PM_INITIATE);
		mdelay(10);
		return 0;
	}

	for (;;) {
		ret = __raw_readl(base + AON_CTRL_PM_INITIATE);
		if (!(ret & PM_INITIATE))
			break;
		if (timeo <= 0) {
			pr_err("error: timeout waiting for BSP (%x)\n", ret);
			break;
		}
		timeo -= 50;
		udelay(50);
	}

	return (ret & 0xff) != PM_INITIATE_SUCCESS;
}

static int brcmstb_pm_handshake(void)
{
	void __iomem *base = ctrl.aon_ctrl_base;
	u32 tmp;
	int ret;

	/* BSP power handshake, v1 */
	tmp = __raw_readl(base + AON_CTRL_HOST_MISC_CMDS);
	tmp &= ~1UL;
	__raw_writel(tmp, base + AON_CTRL_HOST_MISC_CMDS);
	(void)__raw_readl(base + AON_CTRL_HOST_MISC_CMDS);

	ret = do_bsp_initiate_command(BSP_CLOCK_STOP);
	if (ret)
		pr_err("BSP handshake failed\n");

	/*
	 * HACK: BSP may have internal race on the CLOCK_STOP command.
	 * Avoid touching the BSP for a few milliseconds.
	 */
	mdelay(3);

	return ret;
}

/*
 * Run a Power Management State Machine (PMSM) shutdown command and put the CPU
 * into a low-power mode
 */
static void brcmstb_do_pmsm_power_down(unsigned long base_cmd)
{
	void __iomem *base = ctrl.aon_ctrl_base;

	/* pm_start_pwrdn transition 0->1 */
	__raw_writel(base_cmd, base + AON_CTRL_PM_CTRL);
	(void)__raw_readl(base + AON_CTRL_PM_CTRL);

	__raw_writel(base_cmd | PM_PWR_DOWN, base + AON_CTRL_PM_CTRL);
	(void)__raw_readl(base + AON_CTRL_PM_CTRL);

	wfi();
}

/* Support S5 cold boot out of "poweroff" */
static void brcmstb_pm_poweroff(void)
{
	brcmstb_pm_handshake();

	/* Clear magic S3 warm-boot value */
	__raw_writel(0, ctrl.aon_sram + AON_REG_MAGIC_FLAGS);
	(void)__raw_readl(ctrl.aon_sram + AON_REG_MAGIC_FLAGS);

	/* Skip wait-for-interrupt signal; just use a countdown */
	__raw_writel(0x10, ctrl.aon_ctrl_base + AON_CTRL_PM_CPU_WAIT_COUNT);
	(void)__raw_readl(ctrl.aon_ctrl_base + AON_CTRL_PM_CPU_WAIT_COUNT);

	brcmstb_do_pmsm_power_down(PM_COLD_CONFIG);
}

static void *brcmstb_pm_copy_to_sram(void *fn, size_t len)
{
	unsigned int size = ALIGN(len, FNCPY_ALIGN);

	if (ctrl.boot_sram_len < size) {
		pr_err("standby code will not fit in SRAM\n");
		return NULL;
	}

	return fncpy(ctrl.boot_sram, fn, size);
}

/*
 * S2 suspend/resume picks up where we left off, so we must execute carefully
 * from SRAM, in order to allow DDR to come back up safely before we continue.
 */
static int brcmstb_pm_s2(void)
{
	brcmstb_pm_do_s2_sram = brcmstb_pm_copy_to_sram(&brcmstb_pm_do_s2,
			brcmstb_pm_do_s2_sz);
	if (!brcmstb_pm_do_s2_sram)
		return -EINVAL;

	return brcmstb_pm_do_s2_sram(ctrl.aon_ctrl_base,
			ctrl.memcs[0].ddr_phy_base +
			ctrl.pll_status_offset);
}

/*
 * This function is called on a new stack, so don't allow inlining (which will
 * generate stack references on the old stack)
 */
static noinline int brcmstb_pm_s3_finish(void)
{
	struct brcmstb_s3_params *params = ctrl.s3_params;
	phys_addr_t params_pa = ctrl.s3_params_pa;
	phys_addr_t reentry = virt_to_phys(&cpu_resume);
#ifdef CONFIG_BRCMSTB_XPT_HASH
	u32 flags = 0;
#else
	u32 flags = S3_FLAG_NO_MEM_VERIFY;
#endif
	enum bsp_initiate_command cmd;
	int i;

	/* Clear parameter structure */
	memset(params, 0, sizeof(*params));

	flags |= S3_FLAG_LOAD_RANDKEY; /* TODO: make this selectable */

	/* Load random / fixed key */
	if (flags & S3_FLAG_LOAD_RANDKEY)
		cmd = BSP_GEN_RANDOM_KEY;
	else
		cmd = BSP_GEN_FIXED_KEY;
	if (do_bsp_initiate_command(cmd)) {
		pr_info("key loading failed\n");
		return -EIO;
	}

	params->magic = BRCMSTB_S3_MAGIC;
	params->reentry = reentry;

	/* No more writes to DRAM */
	flush_cache_all();

	flags |= BRCMSTB_S3_MAGIC_SHORT;

	__raw_writel(flags, ctrl.aon_sram + AON_REG_MAGIC_FLAGS);
	__raw_writel(lower_32_bits(params_pa),
		     ctrl.aon_sram + AON_REG_CONTROL_LOW);
	__raw_writel(upper_32_bits(params_pa),
		     ctrl.aon_sram + AON_REG_CONTROL_HIGH);

	/* gate PLL | S3 */
	for (i = 0; i < ctrl.num_memc; i++) {
		u32 tmp;
		tmp = __raw_readl(ctrl.memcs[i].ddr_shimphy_base +
				SHIMPHY_DDR_PAD_CNTRL);
		tmp |= SHIMPHY_PAD_GATE_PLL_S3 | SHIMPHY_PAD_PLL_SEQUENCE;
		__raw_writel(tmp, ctrl.memcs[i].ddr_shimphy_base +
				SHIMPHY_DDR_PAD_CNTRL);
	}

	brcmstb_do_pmsm_power_down(PM_WARM_CONFIG);

	/* Must have been interrupted from wfi()? */
	return -EINTR;
}

#define SWAP_STACK(new_sp, saved_sp) \
	__asm__ __volatile__ ( \
		 "mov	%[save], sp\n" \
		 "mov	sp, %[new]\n" \
		 : [save] "=&r" (saved_sp) \
		 : [new] "r" (new_sp) \
		)

/*
 * S3 mode resume to the bootloader before jumping back to Linux, so we can be
 * a little less careful about running from DRAM.
 */
static int brcmstb_pm_do_s3(unsigned long sp)
{
	unsigned long save_sp;
	int ret;

	/* Move to a new stack */
	SWAP_STACK(sp, save_sp);

	/* should not return */
	ret = brcmstb_pm_s3_finish();

	SWAP_STACK(save_sp, sp);

	pr_err("Could not enter S3\n");

	return ret;
}

static int brcmstb_pm_s3(void)
{
	void __iomem *sp = ctrl.boot_sram + ctrl.boot_sram_len - 8;

	return cpu_suspend((unsigned long)sp, brcmstb_pm_do_s3);
}

static int brcmstb_pm_standby(bool deep_standby)
{
	int ret;

	if (brcmstb_pm_handshake())
		return -EIO;

	if (deep_standby)
		ret = brcmstb_pm_s3();
	else
		ret = brcmstb_pm_s2();
	if (ret)
		pr_err("%s: standby failed\n", __func__);

	return ret;
}

static int brcmstb_pm_enter(suspend_state_t state)
{
	int ret = -EINVAL;

	switch (state) {
	case PM_SUSPEND_STANDBY:
		ret = brcmstb_pm_standby(false);
		break;
	case PM_SUSPEND_MEM:
		ret = brcmstb_pm_standby(true);
		break;
	}

	return ret;
}

static int brcmstb_pm_valid(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
		return true;
	case PM_SUSPEND_MEM:
		return ctrl.support_warm_boot;
	default:
		return false;
	}
}

static const struct platform_suspend_ops brcmstb_pm_ops = {
	.enter		= brcmstb_pm_enter,
	.valid		= brcmstb_pm_valid,
};

static const struct of_device_id aon_ctrl_dt_ids[] = {
	{ .compatible = "brcm,brcmstb-aon-ctrl" },
	{}
};

struct ddr_phy_ofdata {
	bool supports_warm_boot;
	size_t pll_status_offset;
};

static struct ddr_phy_ofdata ddr_phy_71_1 = {
	.supports_warm_boot = false,
	.pll_status_offset = 0x0c
};

static struct ddr_phy_ofdata ddr_phy_72_0 = {
	.supports_warm_boot = false,
	.pll_status_offset = 0x10
};

static struct ddr_phy_ofdata ddr_phy_225_1 = {
	.supports_warm_boot = false,
	.pll_status_offset = 0x4
};

static struct ddr_phy_ofdata ddr_phy_240_1 = {
	.supports_warm_boot = true,
	.pll_status_offset = 0x4
};

static const struct of_device_id ddr_phy_dt_ids[] = {
	{
		.compatible = "brcm,brcmstb-ddr-phy-v71.1",
		.data = &ddr_phy_71_1,
	},
	{
		.compatible = "brcm,brcmstb-ddr-phy-v72.0",
		.data = &ddr_phy_72_0,
	},
	{
		.compatible = "brcm,brcmstb-ddr-phy-v225.1",
		.data = &ddr_phy_225_1,
	},
	{
		.compatible = "brcm,brcmstb-ddr-phy-v240.1",
		.data = &ddr_phy_240_1,
	},
	{
		/* Same as v240.1, for the registers we care about */
		.compatible = "brcm,brcmstb-ddr-phy-v240.2",
		.data = &ddr_phy_240_1,
	},
	{}
};

static const struct of_device_id ddr_shimphy_dt_ids[] = {
	{ .compatible = "brcm,brcmstb-ddr-shimphy-v1.0" },
	{}
};

static inline void __iomem *brcmstb_ioremap_node(struct device_node *dn,
						 int index)
{
	return of_io_request_and_map(dn, index, dn->full_name);
}

static void __iomem *brcmstb_ioremap_match(const struct of_device_id *matches,
					   int index, const void **ofdata)
{
	struct device_node *dn;
	const struct of_device_id *match;

	dn = of_find_matching_node_and_match(NULL, matches, &match);
	if (!dn)
		return ERR_PTR(-EINVAL);

	if (ofdata)
		*ofdata = match->data;

	return brcmstb_ioremap_node(dn, index);
}

static int brcmstb_pm_panic_notify(struct notifier_block *nb,
		unsigned long action, void *data)
{
	__raw_writel(BRCMSTB_PANIC_MAGIC,
		ctrl.aon_sram + AON_REG_PANIC);

	return NOTIFY_DONE;
}

static struct notifier_block brcmstb_pm_panic_nb = {
	.notifier_call = brcmstb_pm_panic_notify,
};

static int brcmstb_pm_init(void)
{
	struct device_node *dn;
	void __iomem *base;
	int ret, i;
	const struct ddr_phy_ofdata *ddr_phy_data;

	if (!soc_is_brcmstb())
		return 0;

	/* AON ctrl registers */
	base = brcmstb_ioremap_match(aon_ctrl_dt_ids, 0, NULL);
	if (IS_ERR(base)) {
		pr_err("error mapping AON_CTRL\n");
		return PTR_ERR(base);
	}
	ctrl.aon_ctrl_base = base;

	/* AON SRAM registers */
	base = brcmstb_ioremap_match(aon_ctrl_dt_ids, 1, NULL);
	if (IS_ERR(base)) {
		/* Assume standard offset */
		ctrl.aon_sram = ctrl.aon_ctrl_base +
				     AON_CTRL_SYSTEM_DATA_RAM_OFS;
	} else {
		ctrl.aon_sram = base;
	}

	__raw_writel(0, ctrl.aon_sram + AON_REG_PANIC);

	/* DDR PHY registers */
	base = brcmstb_ioremap_match(ddr_phy_dt_ids, 0,
				     (const void **)&ddr_phy_data);
	if (IS_ERR(base)) {
		pr_err("error mapping DDR PHY\n");
		return PTR_ERR(base);
	}
	ctrl.support_warm_boot = ddr_phy_data->supports_warm_boot;
	ctrl.pll_status_offset = ddr_phy_data->pll_status_offset;
	/* Only need DDR PHY 0 for now? */
	ctrl.memcs[0].ddr_phy_base = base;

	/* DDR SHIM-PHY registers */
	for_each_matching_node(dn, ddr_shimphy_dt_ids) {
		i = ctrl.num_memc;
		if (i >= MAX_NUM_MEMC) {
			pr_warn("too many MEMCs (max %d)\n", MAX_NUM_MEMC);
			break;
		}
		base = brcmstb_ioremap_node(dn, 0);
		if (IS_ERR(base)) {
			if (!ctrl.support_warm_boot)
				break;

			pr_err("error mapping DDR SHIMPHY %d\n", i);
			return PTR_ERR(base);
		}
		ctrl.memcs[i].ddr_shimphy_base = base;
		ctrl.num_memc++;
	}

	dn = of_find_matching_node(NULL, sram_dt_ids);
	if (!dn) {
		pr_err("SRAM not found\n");
		return -EINVAL;
	}

	ret = brcmstb_init_sram(dn);
	if (ret) {
		pr_err("error setting up SRAM for PM\n");
		return ret;
	}

	ctrl.s3_params = kmalloc(sizeof(*ctrl.s3_params), GFP_KERNEL);
	if (!ctrl.s3_params)
		return -ENOMEM;
	ctrl.s3_params_pa = dma_map_single(NULL, ctrl.s3_params,
					   sizeof(*ctrl.s3_params),
					   DMA_TO_DEVICE);
	if (dma_mapping_error(NULL, ctrl.s3_params_pa)) {
		pr_err("error mapping DMA memory\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = brcmstb_regsave_init();
	if (ret)
		goto out2;

	atomic_notifier_chain_register(&panic_notifier_list,
				       &brcmstb_pm_panic_nb);

	pm_power_off = brcmstb_pm_poweroff;
	suspend_set_ops(&brcmstb_pm_ops);
	return 0;

out2:
	dma_unmap_single(NULL, ctrl.s3_params_pa, sizeof(*ctrl.s3_params),
			 DMA_TO_DEVICE);
out:
	kfree(ctrl.s3_params);

	pr_warn("PM: initialization failed with code %d\n", ret);

	return ret;
}

arch_initcall(brcmstb_pm_init);
