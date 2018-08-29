/*
 *  cma_driver.c - Broadcom STB platform CMA driver
 *
 *  Copyright © 2009 - 2015 Broadcom Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  A copy of the GPL is available at
 *  http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 *  Foundation at https://www.gnu.org/licenses/ .
 */

#define pr_fmt(fmt) "cma_driver: " fmt

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/brcmstb/cma_driver.h>
#include <linux/brcmstb/memory_api.h>
#include <linux/mmzone.h>
#include <linux/memblock.h>
#include <linux/device.h>
#include <linux/bitmap.h>
#include <linux/highmem.h>
#include <linux/dma-contiguous.h>
#include <linux/cma.h>
#include <linux/sizes.h>
#include <linux/vme.h>
#include <asm/div64.h>
#include <asm/setup.h>

#define NR_BANKS 8

struct cma_devs_list {
	struct list_head list;
	struct cma_dev *cma_dev;
};

struct cma_root_dev {
	u32 mmap_type;
	struct cdev cdev;
	struct device *dev;
	struct cma_devs_list cma_devs;
};

struct cma_pdev_data {
	phys_addr_t start;
	phys_addr_t size;
	int do_prealloc;
	struct cma *cma_area;
};

struct cma_region_rsv_data {
	int nr_regions_valid;
	struct cma_pdev_data regions[NR_BANKS];
	long long unsigned prm_low_lim_mb;
	int prm_low_kern_rsv_pct;
};

struct cma_rsv_region {
	phys_addr_t addr;
	phys_addr_t size;
};

static struct cma_rsv_region cma_rsv_setup_data[MAX_CMA_AREAS];
static unsigned int n_cma_regions;
static bool cma_disabled;

/* CMA region reservation */

static struct cma_region_rsv_data cma_data __initdata = {
	.prm_low_lim_mb = 32 << 20,
	.prm_low_kern_rsv_pct = 20,
};

enum scan_bitmap_op {
	GET_NUM_REGIONS = 0,
	GET_REGION_INFO,
};

/*
 * The lowest low-memory memblock size needed in order to reserve a cma region
 * on systems with no high-memory.
 */
static int __init cma_mb_low_lim_set(char *p)
{
	cma_data.prm_low_lim_mb = memparse(p, NULL);
	return !cma_data.prm_low_lim_mb ? -EINVAL : 0;
}
early_param("brcm_cma_mb_low_lim", cma_mb_low_lim_set);

/*
 * The percentage of memory reserved for the kernel on systems with no
 * high-memory.
 */
static int __init cma_low_kern_rsv_pct_set(char *p)
{
	return (get_option(&p, &cma_data.prm_low_kern_rsv_pct) == 0) ?
		-EINVAL : 0;
}
early_param("brcm_cma_low_kern_rsv_pct", cma_low_kern_rsv_pct_set);

static int __init __cma_rsv_setup(phys_addr_t addr, phys_addr_t size)
{
	if (n_cma_regions == MAX_CMA_AREAS) {
		pr_warn_once("%s: too many regions, ignoring extras\n",
				__func__);
		return -EINVAL;
	}

	cma_rsv_setup_data[n_cma_regions].addr = addr;
	cma_rsv_setup_data[n_cma_regions].size = size;
	n_cma_regions++;
	return 0;
}

/*
 * We don't do too much checking here because it will be handled by the CMA
 * reservation code
 */
static int __init cma_rsv_setup(char *str)
{
	phys_addr_t addr = 0;
	phys_addr_t size = 0;
	int ret;

	size = (phys_addr_t) memparse(str, &str);
	if (*str == '@')
		addr = (phys_addr_t)memparse(str + 1, &str);

	if (size == 0) {
		pr_info("disabling reserved memory\n");
		cma_disabled = true;
		return 0;
	}

	ret = __cma_rsv_setup(addr, size);
	if (!ret)
		brcmstb_memory_override_defaults = true;
	return ret;
}
early_param("brcm_cma", cma_rsv_setup);

void __init cma_reserve(void)
{
	int i;

	if (cma_disabled) {
		n_cma_regions = 0;
		return;
	}

	for (i = 0; i < n_cma_regions; ++i) {
		struct cma_pdev_data *data = &cma_data.regions[cma_data.nr_regions_valid];
		if (!dma_contiguous_reserve_area(cma_rsv_setup_data[i].size, cma_rsv_setup_data[i].addr, 0, &data->cma_area, true))
		{
			data->start = cma_rsv_setup_data[i].addr;
			data->size = cma_rsv_setup_data[i].size;
			cma_data.nr_regions_valid++;
		}
	}
}

static struct platform_device * __init cma_register_dev_one(int id,
	int region_idx)
{
	struct platform_device *pdev;

	pdev = platform_device_register_data(NULL, "brcm-cma", id,
		&cma_data.regions[region_idx], sizeof(struct cma_pdev_data));
	if (!pdev) {
		pr_err("couldn't register pdev for %d,0x%pa,0x%pa\n", id,
			&cma_data.regions[region_idx].start,
			&cma_data.regions[region_idx].size);
	}

	return pdev;
}

/*
 * Registers platform devices for the regions that were reserved in earlyboot.
 *
 * This function should be called from machine initialization.
 */
void __init cma_register(void)
{
	int i;
	int id = 0;
	struct platform_device *pdev;

	for (i = 0; i < cma_data.nr_regions_valid; i++) {
		pdev = cma_register_dev_one(id, i);
		if (!pdev) {
			pr_err("couldn't register device");
			continue;
		}

		id++;
	}
}

/**
 * cma_dev_get_num_regions() - Get number of allocated regions
 *
 * @cma_dev: The CMA device
 *
 * Return: 0 on success, negative on failure
 */
int cma_dev_get_num_regions(struct cma_dev *cma_dev)
{
	return 0;
}
EXPORT_SYMBOL(cma_dev_get_num_regions);

/**
 * cma_dev_get_region_info() - Get information about allocate region
 *
 * @cma_dev: The CMA device
 * @region_num: Region index
 * @memc: MEMC index associated with the region
 * @addr: Physical address of region
 * @num_bytes: Size of region in bytes
 *
 * Return: 0 on success, negative on failure.
 */
int cma_dev_get_region_info(struct cma_dev *cma_dev, int region_num,
			    s32 *memc, u64 *addr, u64 *num_bytes)
{
	return -1;
}
EXPORT_SYMBOL(cma_dev_get_region_info);

/* CMA driver implementation */

/* Mutex for serializing accesses to private variables */
static DEFINE_SPINLOCK(cma_dev_lock);
static struct cma_root_dev *cma_root_dev;

/**
 * cma_dev_get_cma_dev() - Get a cma_dev * by memc index
 *
 * @index: The CMA device index
 *
 * Return: a pointer to the associated CMA device, or NULL if no such
 * device.
 */
struct cma_dev *cma_dev_get_cma_dev(int index)
{
	struct list_head *pos;
	struct cma_dev *cma_dev = NULL;
	unsigned long flags;

	if (!cma_root_dev)
		return NULL;

	spin_lock_irqsave(&cma_dev_lock, flags);

	list_for_each(pos, &cma_root_dev->cma_devs.list) {
		struct cma_dev *curr_cma_dev;
		curr_cma_dev = list_entry(pos, struct cma_dev, list);
		BUG_ON(curr_cma_dev == NULL);
		if (curr_cma_dev->cma_dev_index == index) {
			cma_dev = curr_cma_dev;
			break;
		}
	}

	spin_unlock_irqrestore(&cma_dev_lock, flags);

	dev_dbg(cma_root_dev->dev, "cma_dev index %d not in list\n", index);
	return cma_dev;
}
EXPORT_SYMBOL(cma_dev_get_cma_dev);

/**
 * cma_dev_get_mem() - Carve out physical memory
 *
 * @cma_dev: The CMA device
 * @addr: Out pointer which will be populated with the start
 * physical address of the contiguous region that was carved out
 * @len: Number of bytes to allocate
 * @align: Byte alignment
 *
 * Return: 0 on success, negative on failure.
 */
int cma_dev_get_mem(struct cma_dev *cma_dev, u64 *addr, u32 len,
			u32 align)
{
	int status = 0;
	struct page *page;
	struct device *dev = cma_dev->dev;

	if ((len & ~PAGE_MASK) || (len == 0)) {
		dev_dbg(dev, "bad length (0x%x)\n", len);
		status = -EINVAL;
		goto done;
	}

	if (align & ~PAGE_MASK) {
		dev_dbg(dev, "bad alignment (0x%x)\n", align);
		status = -EINVAL;
		goto done;
	}

	if (align <= PAGE_SIZE)
		page = dma_alloc_from_contiguous(dev, len >> PAGE_SHIFT, 0);
	else
		page = dma_alloc_from_contiguous(dev, len >> PAGE_SHIFT,
				get_order(align));

	if (page == NULL) {
		status = -ENOMEM;
		goto done;
	}

	*addr = page_to_phys(page);

done:
	return status;
}
EXPORT_SYMBOL(cma_dev_get_mem);

/**
 * cma_dev_put_mem() - Return carved out physical memory
 *
 * @cma_dev: The CMA device
 * @addr: Start physical address of allocated region
 * @len: Number of bytes that were allocated (this must match with get_mem
 * call!)
 *
 * Return: 0 on success, negative on failure.
 */
int cma_dev_put_mem(struct cma_dev *cma_dev, u64 addr, u32 len)
{
	struct page *page = phys_to_page(addr);
	struct device *dev = cma_dev->dev;

	if (page == NULL) {
		dev_dbg(cma_root_dev->dev, "bad addr (0x%llx)\n", addr);
		return -EINVAL;
	}

	if (len % PAGE_SIZE) {
		dev_dbg(cma_root_dev->dev, "bad length (0x%x)\n", len);
		return -EINVAL;
	}

	if (!dma_release_from_contiguous(dev, page, len / PAGE_SIZE))
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(cma_dev_put_mem);

static int cma_drvr_test_cma_dev(struct device *dev)
{
	struct page *page;

	/* Do a dummy alloc to ensure the CMA device is truly ready */
	page = dma_alloc_from_contiguous(dev, 1, 0);
	if (page == NULL)
		return -EINVAL;

	if (!dma_release_from_contiguous(dev, page, 1)) {
		dev_err(dev, "test dma release failed!\n");
		return -EINVAL;
	}

	return 0;
}

static int __init cma_drvr_config_platdev(struct platform_device *pdev,
					struct cma_dev *cma_dev)
{
	struct device *dev = &pdev->dev;
	struct cma_pdev_data *data =
		(struct cma_pdev_data *)dev->platform_data;

	if (!data) {
		dev_err(dev, "platform data not initialized\n");
		return -EINVAL;
	}

	cma_dev->range.base = data->start;
	cma_dev->range.size = data->size;
	cma_dev->cma_dev_index = pdev->id;
	cma_dev->memc = brcmstb_memory_phys_addr_to_memc(data->start);

	if (!data->cma_area) {
		pr_err("null cma area\n");
		return -EINVAL;
	}
	dev_set_cma_area(dev, data->cma_area);

	if (cma_drvr_test_cma_dev(dev)) {
		dev_err(dev, "CMA testing failed!\n");
		return -EINVAL;
	}

	return 0;
}

static int cma_drvr_init_root_dev(struct device *dev)
{
	struct cma_root_dev *my_cma_root_dev;

	my_cma_root_dev = devm_kzalloc(dev, sizeof(*my_cma_root_dev),
					GFP_KERNEL);
	if (my_cma_root_dev == NULL)
		return -ENOMEM;

	/* Initialize list of CMA devices referenced by the root CMA device */
	INIT_LIST_HEAD(&my_cma_root_dev->cma_devs.list);

	cma_root_dev = my_cma_root_dev;

	dev_info(dev, "Initialized Broadcom CMA root device\n");

	return 0;
}

static int __init cma_drvr_do_prealloc(struct device *dev,
	struct cma_dev *cma_dev)
{
	int ret;
	u64 addr;
	u32 size;

	size = cma_dev->range.size;

	ret = cma_dev_get_mem(cma_dev, &addr, size, 0);
	if (ret)
		dev_err(dev, "Unable to pre-allocate 0x%x bytes (%d)", size,
			ret);
	else
		dev_info(dev, "Pre-allocated 0x%x bytes @ 0x%llx", size, addr);

	return ret;
}

static int __init cma_drvr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cma_dev *cma_dev = NULL;
	int ret = 0;
	struct cma_pdev_data *data =
		(struct cma_pdev_data *)dev->platform_data;

	/* Initialize the root device only once */
	if (cma_root_dev == NULL)
		ret = cma_drvr_init_root_dev(dev);

	if (ret)
		goto done;

	cma_dev = devm_kzalloc(dev, sizeof(*cma_dev), GFP_KERNEL);
	if (cma_dev == NULL) {
		dev_err(dev, "can't alloc cma_dev\n");
		ret = -ENOMEM;
		goto done;
	}

	ret = cma_drvr_config_platdev(pdev, cma_dev);
	if (ret)
		goto free_cma_dev;

	cma_dev->dev = dev;

	/*
	 * Keep a pointer to all of the devs so we don't have to search for it
	 * elsewhere.
	 */
	INIT_LIST_HEAD(&cma_dev->list);
	list_add(&cma_dev->list, &cma_root_dev->cma_devs.list);
	dev_info(dev, "Added CMA device @ PA=0x%llx LEN=0x%x\n",
		 cma_dev->range.base, cma_dev->range.size);

	if (data->do_prealloc) {
		/*
		 * Pre-allocation failure isn't truly a probe error because the
		 * device has been initialized at this point. An error will be
		 * logged if there is a failure.
		 */
		cma_drvr_do_prealloc(dev, cma_dev);
	}

	goto done;

free_cma_dev:
	devm_kfree(dev, cma_dev);

done:
	return ret;
}

int cma_drvr_is_ready(void)
{
	return cma_root_dev != NULL;
}

static struct platform_driver cma_driver = {
	.driver = {
		.name = "brcm-cma",
		.owner = THIS_MODULE,
	},
};

static int __init cma_drvr_init(void)
{
	return platform_driver_probe(&cma_driver, cma_drvr_probe);
}
module_init(cma_drvr_init);

static void __exit cma_drvr_exit(void)
{
	platform_driver_unregister(&cma_driver);
}
module_exit(cma_drvr_exit);
