// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018-2019 HiSilicon Limited. */
#include <linux/acpi.h>
#include <linux/aer.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/topology.h>
#include <linux/uacce.h>
#include "hpre.h"

#define HPRE_QM_ABNML_INT_MASK		0x100004
#define HPRE_CTRL_CNT_CLR_CE_BIT	BIT(0)
#define HPRE_COMM_CNT_CLR_CE		0x0
#define HPRE_CTRL_CNT_CLR_CE		0x301000
#define HPRE_FSM_MAX_CNT		0x301008
#define HPRE_VFG_AXQOS			0x30100c
#define HPRE_VFG_AXCACHE		0x301010
#define HPRE_RDCHN_INI_CFG		0x301014
#define HPRE_AWUSR_FP_CFG		0x301018
#define HPRE_BD_ENDIAN			0x301020
#define HPRE_ECC_BYPASS			0x301024
#define HPRE_RAS_WIDTH_CFG		0x301028
#define HPRE_POISON_BYPASS		0x30102c
#define HPRE_BD_ARUSR_CFG		0x301030
#define HPRE_BD_AWUSR_CFG		0x301034
#define HPRE_TYPES_ENB			0x301038
#define HPRE_RSA_ENB			BIT(0)
#define HPRE_ECC_ENB			BIT(1)
#define HPRE_DATA_RUSER_CFG		0x30103c
#define HPRE_DATA_WUSER_CFG		0x301040
#define HPRE_INT_MASK			0x301400
#define HPRE_INT_STATUS			0x301800
#define HPRE_CORE_INT_ENABLE		0
#define HPRE_CORE_INT_DISABLE		GENMASK(21, 0)
#define HPRE_RDCHN_INI_ST		0x301a00
#define HPRE_CLSTR_BASE			0x302000
#define HPRE_CORE_EN_OFFSET		0x04
#define HPRE_CORE_INI_CFG_OFFSET	0x20
#define HPRE_CORE_INI_STATUS_OFFSET	0x80
#define HPRE_CORE_HTBT_WARN_OFFSET	0x8c
#define HPRE_CORE_IS_SCHD_OFFSET	0x90

#define HPRE_RAS_CE_ENB			0x301410
#define HPRE_HAC_RAS_CE_ENABLE		(BIT(0) | BIT(22) | BIT(23))
#define HPRE_RAS_NFE_ENB		0x301414
#define HPRE_HAC_RAS_NFE_ENABLE		0x3ffffe
#define HPRE_RAS_FE_ENB			0x301418
#define HPRE_OOO_SHUTDOWN_SEL		0x301a3c
#define HPRE_HAC_RAS_FE_ENABLE		0

#define HPRE_CORE_ENB		(HPRE_CLSTR_BASE + HPRE_CORE_EN_OFFSET)
#define HPRE_CORE_INI_CFG	(HPRE_CLSTR_BASE + HPRE_CORE_INI_CFG_OFFSET)
#define HPRE_CORE_INI_STATUS (HPRE_CLSTR_BASE + HPRE_CORE_INI_STATUS_OFFSET)
#define HPRE_HAC_ECC1_CNT		0x301a04
#define HPRE_HAC_ECC2_CNT		0x301a08
#define HPRE_HAC_SOURCE_INT		0x301600
#define HPRE_CLSTR_ADDR_INTRVL		0x1000
#define HPRE_CLUSTER_INQURY		0x100
#define HPRE_CLSTR_ADDR_INQRY_RSLT	0x104
#define HPRE_TIMEOUT_ABNML_BIT		6
#define HPRE_PASID_EN_BIT		9
#define HPRE_REG_RD_INTVRL_US		10
#define HPRE_REG_RD_TMOUT_US		1000
#define HPRE_DBGFS_VAL_MAX_LEN		20
#define HPRE_PCI_DEVICE_ID		0xa258
#define HPRE_PCI_VF_DEVICE_ID		0xa259
#define HPRE_QM_USR_CFG_MASK		GENMASK(31, 1)
#define HPRE_QM_AXI_CFG_MASK		GENMASK(15, 0)
#define HPRE_QM_VFG_AX_MASK		GENMASK(7, 0)
#define HPRE_BD_USR_MASK		GENMASK(1, 0)
#define HPRE_CLUSTER_CORE_MASK_V2	GENMASK(3, 0)
#define HPRE_CLUSTER_CORE_MASK_V3	GENMASK(7, 0)
#define HPRE_PREFETCH_CFG		0x301130
#define HPRE_SVA_PREFTCH_DFX		0x30115C
#define HPRE_PREFETCH_ENABLE		(~(BIT(0) | BIT(30)))
#define HPRE_PREFETCH_DISABLE		BIT(30)
#define HPRE_SVA_DISABLE_READY		(BIT(4) | BIT(8))

/* clock gate */
#define HPRE_CLKGATE_CTL		0x301a10
#define HPRE_PEH_CFG_AUTO_GATE		0x301a2c
#define HPRE_CLUSTER_DYN_CTL		0x302010
#define HPRE_CORE_SHB_CFG		0x302088
#define HPRE_CLKGATE_CTL_EN		BIT(0)
#define HPRE_PEH_CFG_AUTO_GATE_EN	BIT(0)
#define HPRE_CLUSTER_DYN_CTL_EN		BIT(0)
#define HPRE_CORE_GATE_EN		(BIT(30) | BIT(31))

#define HPRE_AM_OOO_SHUTDOWN_ENB	0x301044
#define HPRE_AM_OOO_SHUTDOWN_ENABLE	BIT(0)
#define HPRE_WR_MSI_PORT		BIT(2)

#define HPRE_CORE_ECC_2BIT_ERR		BIT(1)
#define HPRE_OOO_ECC_2BIT_ERR		BIT(5)

#define HPRE_QM_BME_FLR			BIT(7)
#define HPRE_QM_PM_FLR			BIT(11)
#define HPRE_QM_SRIOV_FLR		BIT(12)

#define HPRE_SHAPER_TYPE_RATE		128
#define HPRE_VIA_MSI_DSM		1
#define HPRE_SQE_MASK_OFFSET		8
#define HPRE_SQE_MASK_LEN		24

static const char hpre_name[] = "hisi_hpre";
static struct dentry *hpre_debugfs_root;
static const struct pci_device_id hpre_dev_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_HUAWEI, HPRE_PCI_DEVICE_ID) },
	{ PCI_DEVICE(PCI_VENDOR_ID_HUAWEI, HPRE_PCI_VF_DEVICE_ID) },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, hpre_dev_ids);

struct hpre_hw_error {
	u32 int_msk;
	const char *msg;
};

static struct hisi_qm_list hpre_devices = {
	.register_to_crypto	= hpre_algs_register,
	.unregister_from_crypto	= hpre_algs_unregister,
};

static const char * const hpre_debug_file_name[] = {
	[HPRE_CLEAR_ENABLE] = "rdclr_en",
	[HPRE_CLUSTER_CTRL] = "cluster_ctrl",
};

static const struct hpre_hw_error hpre_hw_errors[] = {
	{
		.int_msk = BIT(0),
		.msg = "core_ecc_1bit_err_int_set"
	}, {
		.int_msk = BIT(1),
		.msg = "core_ecc_2bit_err_int_set"
	}, {
		.int_msk = BIT(2),
		.msg = "dat_wb_poison_int_set"
	}, {
		.int_msk = BIT(3),
		.msg = "dat_rd_poison_int_set"
	}, {
		.int_msk = BIT(4),
		.msg = "bd_rd_poison_int_set"
	}, {
		.int_msk = BIT(5),
		.msg = "ooo_ecc_2bit_err_int_set"
	}, {
		.int_msk = BIT(6),
		.msg = "cluster1_shb_timeout_int_set"
	}, {
		.int_msk = BIT(7),
		.msg = "cluster2_shb_timeout_int_set"
	}, {
		.int_msk = BIT(8),
		.msg = "cluster3_shb_timeout_int_set"
	}, {
		.int_msk = BIT(9),
		.msg = "cluster4_shb_timeout_int_set"
	}, {
		.int_msk = GENMASK(15, 10),
		.msg = "ooo_rdrsp_err_int_set"
	}, {
		.int_msk = GENMASK(21, 16),
		.msg = "ooo_wrrsp_err_int_set"
	}, {
		.int_msk = BIT(22),
		.msg = "pt_rng_timeout_int_set"
	}, {
		.int_msk = BIT(23),
		.msg = "sva_fsm_timeout_int_set"
	}, {
		/* sentinel */
	}
};

static const u64 hpre_cluster_offsets[] = {
	[HPRE_CLUSTER0] =
		HPRE_CLSTR_BASE + HPRE_CLUSTER0 * HPRE_CLSTR_ADDR_INTRVL,
	[HPRE_CLUSTER1] =
		HPRE_CLSTR_BASE + HPRE_CLUSTER1 * HPRE_CLSTR_ADDR_INTRVL,
	[HPRE_CLUSTER2] =
		HPRE_CLSTR_BASE + HPRE_CLUSTER2 * HPRE_CLSTR_ADDR_INTRVL,
	[HPRE_CLUSTER3] =
		HPRE_CLSTR_BASE + HPRE_CLUSTER3 * HPRE_CLSTR_ADDR_INTRVL,
};

static const struct debugfs_reg32 hpre_cluster_dfx_regs[] = {
	{"CORES_EN_STATUS          ",  HPRE_CORE_EN_OFFSET},
	{"CORES_INI_CFG              ",  HPRE_CORE_INI_CFG_OFFSET},
	{"CORES_INI_STATUS         ",  HPRE_CORE_INI_STATUS_OFFSET},
	{"CORES_HTBT_WARN         ",  HPRE_CORE_HTBT_WARN_OFFSET},
	{"CORES_IS_SCHD               ",  HPRE_CORE_IS_SCHD_OFFSET},
};

static const struct debugfs_reg32 hpre_com_dfx_regs[] = {
	{"READ_CLR_EN          ",  HPRE_CTRL_CNT_CLR_CE},
	{"AXQOS                   ",  HPRE_VFG_AXQOS},
	{"AWUSR_CFG              ",  HPRE_AWUSR_FP_CFG},
	{"QM_ARUSR_MCFG1           ",  QM_ARUSER_M_CFG_1},
	{"QM_AWUSR_MCFG1           ",  QM_AWUSER_M_CFG_1},
	{"BD_ENDIAN               ",  HPRE_BD_ENDIAN},
	{"ECC_CHECK_CTRL       ",  HPRE_ECC_BYPASS},
	{"RAS_INT_WIDTH       ",  HPRE_RAS_WIDTH_CFG},
	{"POISON_BYPASS       ",  HPRE_POISON_BYPASS},
	{"BD_ARUSER               ",  HPRE_BD_ARUSR_CFG},
	{"BD_AWUSER               ",  HPRE_BD_AWUSR_CFG},
	{"DATA_ARUSER            ",  HPRE_DATA_RUSER_CFG},
	{"DATA_AWUSER           ",  HPRE_DATA_WUSER_CFG},
	{"INT_STATUS               ",  HPRE_INT_STATUS},
};

static const char *hpre_dfx_files[HPRE_DFX_FILE_NUM] = {
	"send_cnt",
	"recv_cnt",
	"send_fail_cnt",
	"send_busy_cnt",
	"over_thrhld_cnt",
	"overtime_thrhld",
	"invalid_req_cnt"
};

static const struct kernel_param_ops hpre_uacce_mode_ops = {
	.set = uacce_mode_set,
	.get = param_get_int,
};

/*
 * uacce_mode = 0 means hpre only register to crypto,
 * uacce_mode = 1 means hpre both register to crypto and uacce.
 */
static u32 uacce_mode = UACCE_MODE_NOUACCE;
module_param_cb(uacce_mode, &hpre_uacce_mode_ops, &uacce_mode, 0444);
MODULE_PARM_DESC(uacce_mode, UACCE_MODE_DESC);

static int pf_q_num_set(const char *val, const struct kernel_param *kp)
{
	return q_num_set(val, kp, HPRE_PCI_DEVICE_ID);
}

static const struct kernel_param_ops hpre_pf_q_num_ops = {
	.set = pf_q_num_set,
	.get = param_get_int,
};

static u32 pf_q_num = HPRE_PF_DEF_Q_NUM;
module_param_cb(pf_q_num, &hpre_pf_q_num_ops, &pf_q_num, 0444);
MODULE_PARM_DESC(pf_q_num, "Number of queues in PF of CS(2-1024)");

static const struct kernel_param_ops vfs_num_ops = {
	.set = vfs_num_set,
	.get = param_get_int,
};

static u32 vfs_num;
module_param_cb(vfs_num, &vfs_num_ops, &vfs_num, 0444);
MODULE_PARM_DESC(vfs_num, "Number of VFs to enable(1-63), 0(default)");

static inline int hpre_cluster_num(struct hisi_qm *qm)
{
	return (qm->ver >= QM_HW_V3) ? HPRE_CLUSTERS_NUM_V3 :
		HPRE_CLUSTERS_NUM_V2;
}

static inline int hpre_cluster_core_mask(struct hisi_qm *qm)
{
	return (qm->ver >= QM_HW_V3) ?
		HPRE_CLUSTER_CORE_MASK_V3 : HPRE_CLUSTER_CORE_MASK_V2;
}

struct hisi_qp *hpre_create_qp(u8 type)
{
	int node = cpu_to_node(smp_processor_id());
	struct hisi_qp *qp = NULL;
	int ret;

	if (type != HPRE_V2_ALG_TYPE && type != HPRE_V3_ECC_ALG_TYPE)
		return NULL;

	/*
	 * type: 0 - RSA/DH. algorithm supported in V2,
	 *       1 - ECC algorithm in V3.
	 */
	ret = hisi_qm_alloc_qps_node(&hpre_devices, 1, type, node, &qp);
	if (!ret)
		return qp;

	return NULL;
}

static void hpre_config_pasid(struct hisi_qm *qm)
{
	u32 val1, val2;

	if (qm->ver >= QM_HW_V3)
		return;

	val1 = readl_relaxed(qm->io_base + HPRE_DATA_RUSER_CFG);
	val2 = readl_relaxed(qm->io_base + HPRE_DATA_WUSER_CFG);
	if (qm->use_sva) {
		val1 |= BIT(HPRE_PASID_EN_BIT);
		val2 |= BIT(HPRE_PASID_EN_BIT);
	} else {
		val1 &= ~BIT(HPRE_PASID_EN_BIT);
		val2 &= ~BIT(HPRE_PASID_EN_BIT);
	}
	writel_relaxed(val1, qm->io_base + HPRE_DATA_RUSER_CFG);
	writel_relaxed(val2, qm->io_base + HPRE_DATA_WUSER_CFG);
}

static int hpre_cfg_by_dsm(struct hisi_qm *qm)
{
	struct device *dev = &qm->pdev->dev;
	union acpi_object *obj;
	guid_t guid;

	if (guid_parse("b06b81ab-0134-4a45-9b0c-483447b95fa7", &guid)) {
		dev_err(dev, "Hpre GUID failed\n");
		return -EINVAL;
	}

	/* Switch over to MSI handling due to non-standard PCI implementation */
	obj = acpi_evaluate_dsm(ACPI_HANDLE(dev), &guid,
				0, HPRE_VIA_MSI_DSM, NULL);
	if (!obj) {
		dev_err(dev, "ACPI handle failed!\n");
		return -EIO;
	}

	ACPI_FREE(obj);

	return 0;
}

static int hpre_set_cluster(struct hisi_qm *qm)
{
	u32 cluster_core_mask = hpre_cluster_core_mask(qm);
	u8 clusters_num = hpre_cluster_num(qm);
	struct device *dev = &qm->pdev->dev;
	unsigned long offset;
	u32 val = 0;
	int ret, i;

	for (i = 0; i < clusters_num; i++) {
		offset = i * HPRE_CLSTR_ADDR_INTRVL;

		/* clusters initiating */
		pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:355", cluster_core_mask,
		       qm->io_base + offset + HPRE_CORE_ENB);
		pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:357", 0x1, qm->io_base + offset + HPRE_CORE_INI_CFG);
		ret = readl_relaxed_poll_timeout(qm->io_base + offset +
					HPRE_CORE_INI_STATUS, val,
					((val & cluster_core_mask) ==
					cluster_core_mask),
					HPRE_REG_RD_INTVRL_US,
					HPRE_REG_RD_TMOUT_US);
		if (ret) {
			dev_err(dev,
				"cluster %d int st status timeout!\n", i);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

/*
 * For Kunpeng 920, we should disable FLR triggered by hardware (BME/PM/SRIOV).
 * Or it may stay in D3 state when we bind and unbind hpre quickly,
 * as it does FLR triggered by hardware.
 */
static void disable_flr_of_bme(struct hisi_qm *qm)
{
	u32 val;

	val = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:383", qm->io_base + QM_PEH_AXUSER_CFG);
	val &= ~(HPRE_QM_BME_FLR | HPRE_QM_SRIOV_FLR);
	val |= HPRE_QM_PM_FLR;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:386", val, qm->io_base + QM_PEH_AXUSER_CFG);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:387", PEH_AXUSER_CFG_ENABLE, qm->io_base + QM_PEH_AXUSER_CFG_ENABLE);
}

static void hpre_open_sva_prefetch(struct hisi_qm *qm)
{
	u32 val;
	int ret;

	if (qm->ver < QM_HW_V3)
		return;

	/* Enable prefetch */
	val = readl_relaxed(qm->io_base + HPRE_PREFETCH_CFG);
	val &= HPRE_PREFETCH_ENABLE;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:401", val, qm->io_base + HPRE_PREFETCH_CFG);

	ret = readl_relaxed_poll_timeout(qm->io_base + HPRE_PREFETCH_CFG,
					 val, !(val & HPRE_PREFETCH_DISABLE),
					 HPRE_REG_RD_INTVRL_US,
					 HPRE_REG_RD_TMOUT_US);
	if (ret)
		pci_err(qm->pdev, "failed to open sva prefetch\n");
}

static void hpre_close_sva_prefetch(struct hisi_qm *qm)
{
	u32 val;
	int ret;

	if (qm->ver < QM_HW_V3)
		return;

	val = readl_relaxed(qm->io_base + HPRE_PREFETCH_CFG);
	val |= HPRE_PREFETCH_DISABLE;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:421", val, qm->io_base + HPRE_PREFETCH_CFG);

	ret = readl_relaxed_poll_timeout(qm->io_base + HPRE_SVA_PREFTCH_DFX,
					 val, !(val & HPRE_SVA_DISABLE_READY),
					 HPRE_REG_RD_INTVRL_US,
					 HPRE_REG_RD_TMOUT_US);
	if (ret)
		pci_err(qm->pdev, "failed to close sva prefetch\n");
}

static void hpre_enable_clock_gate(struct hisi_qm *qm)
{
	u32 val;

	if (qm->ver < QM_HW_V3)
		return;

	val = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:438", qm->io_base + HPRE_CLKGATE_CTL);
	val |= HPRE_CLKGATE_CTL_EN;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:440", val, qm->io_base + HPRE_CLKGATE_CTL);

	val = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:442", qm->io_base + HPRE_PEH_CFG_AUTO_GATE);
	val |= HPRE_PEH_CFG_AUTO_GATE_EN;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:444", val, qm->io_base + HPRE_PEH_CFG_AUTO_GATE);

	val = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:446", qm->io_base + HPRE_CLUSTER_DYN_CTL);
	val |= HPRE_CLUSTER_DYN_CTL_EN;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:448", val, qm->io_base + HPRE_CLUSTER_DYN_CTL);

	val = readl_relaxed(qm->io_base + HPRE_CORE_SHB_CFG);
	val |= HPRE_CORE_GATE_EN;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:452", val, qm->io_base + HPRE_CORE_SHB_CFG);
}

static void hpre_disable_clock_gate(struct hisi_qm *qm)
{
	u32 val;

	if (qm->ver < QM_HW_V3)
		return;

	val = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:462", qm->io_base + HPRE_CLKGATE_CTL);
	val &= ~HPRE_CLKGATE_CTL_EN;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:464", val, qm->io_base + HPRE_CLKGATE_CTL);

	val = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:466", qm->io_base + HPRE_PEH_CFG_AUTO_GATE);
	val &= ~HPRE_PEH_CFG_AUTO_GATE_EN;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:468", val, qm->io_base + HPRE_PEH_CFG_AUTO_GATE);

	val = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:470", qm->io_base + HPRE_CLUSTER_DYN_CTL);
	val &= ~HPRE_CLUSTER_DYN_CTL_EN;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:472", val, qm->io_base + HPRE_CLUSTER_DYN_CTL);

	val = readl_relaxed(qm->io_base + HPRE_CORE_SHB_CFG);
	val &= ~HPRE_CORE_GATE_EN;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:476", val, qm->io_base + HPRE_CORE_SHB_CFG);
}

static int hpre_set_user_domain_and_cache(struct hisi_qm *qm)
{
	struct device *dev = &qm->pdev->dev;
	u32 val;
	int ret;

	/* disabel dynamic clock gate before sram init */
	hpre_disable_clock_gate(qm);

	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:488", HPRE_QM_USR_CFG_MASK, qm->io_base + QM_ARUSER_M_CFG_ENABLE);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:489", HPRE_QM_USR_CFG_MASK, qm->io_base + QM_AWUSER_M_CFG_ENABLE);
	writel_relaxed(HPRE_QM_AXI_CFG_MASK, qm->io_base + QM_AXI_M_CFG);

	/* HPRE need more time, we close this interrupt */
	val = readl_relaxed(qm->io_base + HPRE_QM_ABNML_INT_MASK);
	val |= BIT(HPRE_TIMEOUT_ABNML_BIT);
	writel_relaxed(val, qm->io_base + HPRE_QM_ABNML_INT_MASK);

	if (qm->ver >= QM_HW_V3)
		pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:498", HPRE_RSA_ENB | HPRE_ECC_ENB,
			qm->io_base + HPRE_TYPES_ENB);
	else
		pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:501", HPRE_RSA_ENB, qm->io_base + HPRE_TYPES_ENB);

	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:503", HPRE_QM_VFG_AX_MASK, qm->io_base + HPRE_VFG_AXCACHE);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:504", 0x0, qm->io_base + HPRE_BD_ENDIAN);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:505", 0x0, qm->io_base + HPRE_INT_MASK);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:506", 0x0, qm->io_base + HPRE_POISON_BYPASS);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:507", 0x0, qm->io_base + HPRE_COMM_CNT_CLR_CE);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:508", 0x0, qm->io_base + HPRE_ECC_BYPASS);

	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:510", HPRE_BD_USR_MASK, qm->io_base + HPRE_BD_ARUSR_CFG);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:511", HPRE_BD_USR_MASK, qm->io_base + HPRE_BD_AWUSR_CFG);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:512", 0x1, qm->io_base + HPRE_RDCHN_INI_CFG);
	ret = readl_relaxed_poll_timeout(qm->io_base + HPRE_RDCHN_INI_ST, val,
			val & BIT(0),
			HPRE_REG_RD_INTVRL_US,
			HPRE_REG_RD_TMOUT_US);
	if (ret) {
		dev_err(dev, "read rd channel timeout fail!\n");
		return -ETIMEDOUT;
	}

	ret = hpre_set_cluster(qm);
	if (ret)
		return -ETIMEDOUT;

	/* This setting is only needed by Kunpeng 920. */
	if (qm->ver == QM_HW_V2) {
		ret = hpre_cfg_by_dsm(qm);
		if (ret)
			return ret;

		disable_flr_of_bme(qm);
	}

	/* Config data buffer pasid needed by Kunpeng 920 */
	hpre_config_pasid(qm);

	hpre_enable_clock_gate(qm);

	return ret;
}

static void hpre_cnt_regs_clear(struct hisi_qm *qm)
{
	u8 clusters_num = hpre_cluster_num(qm);
	unsigned long offset;
	int i;

	/* clear clusterX/cluster_ctrl */
	for (i = 0; i < clusters_num; i++) {
		offset = HPRE_CLSTR_BASE + i * HPRE_CLSTR_ADDR_INTRVL;
		pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:552", 0x0, qm->io_base + offset + HPRE_CLUSTER_INQURY);
	}

	/* clear rdclr_en */
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:556", 0x0, qm->io_base + HPRE_CTRL_CNT_CLR_CE);

	hisi_qm_debug_regs_clear(qm);
}

static void hpre_master_ooo_ctrl(struct hisi_qm *qm, bool enable)
{
	u32 val1, val2;

	val1 = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:565", qm->io_base + HPRE_AM_OOO_SHUTDOWN_ENB);
	if (enable) {
		val1 |= HPRE_AM_OOO_SHUTDOWN_ENABLE;
		val2 = HPRE_HAC_RAS_NFE_ENABLE;
	} else {
		val1 &= ~HPRE_AM_OOO_SHUTDOWN_ENABLE;
		val2 = 0x0;
	}

	if (qm->ver > QM_HW_V2)
		pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:575", val2, qm->io_base + HPRE_OOO_SHUTDOWN_SEL);

	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:577", val1, qm->io_base + HPRE_AM_OOO_SHUTDOWN_ENB);
}

static void hpre_hw_error_disable(struct hisi_qm *qm)
{
	/* disable hpre hw error interrupts */
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:583", HPRE_CORE_INT_DISABLE, qm->io_base + HPRE_INT_MASK);

	/* disable HPRE block master OOO when nfe occurs on Kunpeng930 */
	hpre_master_ooo_ctrl(qm, false);
}

static void hpre_hw_error_enable(struct hisi_qm *qm)
{
	/* clear HPRE hw error source if having */
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:592", HPRE_CORE_INT_DISABLE, qm->io_base + HPRE_HAC_SOURCE_INT);

	/* configure error type */
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:595", HPRE_HAC_RAS_CE_ENABLE, qm->io_base + HPRE_RAS_CE_ENB);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:596", HPRE_HAC_RAS_NFE_ENABLE, qm->io_base + HPRE_RAS_NFE_ENB);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:597", HPRE_HAC_RAS_FE_ENABLE, qm->io_base + HPRE_RAS_FE_ENB);

	/* enable HPRE block master OOO when nfe occurs on Kunpeng930 */
	hpre_master_ooo_ctrl(qm, true);

	/* enable hpre hw error interrupts */
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:603", HPRE_CORE_INT_ENABLE, qm->io_base + HPRE_INT_MASK);
}

static inline struct hisi_qm *hpre_file_to_qm(struct hpre_debugfs_file *file)
{
	struct hpre *hpre = container_of(file->debug, struct hpre, debug);

	return &hpre->qm;
}

static u32 hpre_clear_enable_read(struct hpre_debugfs_file *file)
{
	struct hisi_qm *qm = hpre_file_to_qm(file);

	return pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:617", qm->io_base + HPRE_CTRL_CNT_CLR_CE) &
	       HPRE_CTRL_CNT_CLR_CE_BIT;
}

static int hpre_clear_enable_write(struct hpre_debugfs_file *file, u32 val)
{
	struct hisi_qm *qm = hpre_file_to_qm(file);
	u32 tmp;

	if (val != 1 && val != 0)
		return -EINVAL;

	tmp = (pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:629", qm->io_base + HPRE_CTRL_CNT_CLR_CE) &
	       ~HPRE_CTRL_CNT_CLR_CE_BIT) | val;
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:631", tmp, qm->io_base + HPRE_CTRL_CNT_CLR_CE);

	return 0;
}

static u32 hpre_cluster_inqry_read(struct hpre_debugfs_file *file)
{
	struct hisi_qm *qm = hpre_file_to_qm(file);
	int cluster_index = file->index - HPRE_CLUSTER_CTRL;
	unsigned long offset = HPRE_CLSTR_BASE +
			       cluster_index * HPRE_CLSTR_ADDR_INTRVL;

	return pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:643", qm->io_base + offset + HPRE_CLSTR_ADDR_INQRY_RSLT);
}

static int hpre_cluster_inqry_write(struct hpre_debugfs_file *file, u32 val)
{
	struct hisi_qm *qm = hpre_file_to_qm(file);
	int cluster_index = file->index - HPRE_CLUSTER_CTRL;
	unsigned long offset = HPRE_CLSTR_BASE + cluster_index *
			       HPRE_CLSTR_ADDR_INTRVL;

	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:653", val, qm->io_base + offset + HPRE_CLUSTER_INQURY);

	return 0;
}

static ssize_t hpre_ctrl_debug_read(struct file *filp, char __user *buf,
				    size_t count, loff_t *pos)
{
	struct hpre_debugfs_file *file = filp->private_data;
	struct hisi_qm *qm = hpre_file_to_qm(file);
	char tbuf[HPRE_DBGFS_VAL_MAX_LEN];
	u32 val;
	int ret;

	ret = hisi_qm_get_dfx_access(qm);
	if (ret)
		return ret;

	spin_lock_irq(&file->lock);
	switch (file->type) {
	case HPRE_CLEAR_ENABLE:
		val = hpre_clear_enable_read(file);
		break;
	case HPRE_CLUSTER_CTRL:
		val = hpre_cluster_inqry_read(file);
		break;
	default:
		goto err_input;
	}
	spin_unlock_irq(&file->lock);

	hisi_qm_put_dfx_access(qm);
	ret = snprintf(tbuf, HPRE_DBGFS_VAL_MAX_LEN, "%u\n", val);
	return simple_read_from_buffer(buf, count, pos, tbuf, ret);

err_input:
	spin_unlock_irq(&file->lock);
	hisi_qm_put_dfx_access(qm);
	return -EINVAL;
}

static ssize_t hpre_ctrl_debug_write(struct file *filp, const char __user *buf,
				     size_t count, loff_t *pos)
{
	struct hpre_debugfs_file *file = filp->private_data;
	struct hisi_qm *qm = hpre_file_to_qm(file);
	char tbuf[HPRE_DBGFS_VAL_MAX_LEN];
	unsigned long val;
	int len, ret;

	if (*pos != 0)
		return 0;

	if (count >= HPRE_DBGFS_VAL_MAX_LEN)
		return -ENOSPC;

	len = simple_write_to_buffer(tbuf, HPRE_DBGFS_VAL_MAX_LEN - 1,
				     pos, buf, count);
	if (len < 0)
		return len;

	tbuf[len] = '\0';
	if (kstrtoul(tbuf, 0, &val))
		return -EFAULT;

	ret = hisi_qm_get_dfx_access(qm);
	if (ret)
		return ret;

	spin_lock_irq(&file->lock);
	switch (file->type) {
	case HPRE_CLEAR_ENABLE:
		ret = hpre_clear_enable_write(file, val);
		if (ret)
			goto err_input;
		break;
	case HPRE_CLUSTER_CTRL:
		ret = hpre_cluster_inqry_write(file, val);
		if (ret)
			goto err_input;
		break;
	default:
		ret = -EINVAL;
		goto err_input;
	}

	ret = count;

err_input:
	spin_unlock_irq(&file->lock);
	hisi_qm_put_dfx_access(qm);
	return ret;
}

static const struct file_operations hpre_ctrl_debug_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = hpre_ctrl_debug_read,
	.write = hpre_ctrl_debug_write,
};

static int hpre_debugfs_atomic64_get(void *data, u64 *val)
{
	struct hpre_dfx *dfx_item = data;

	*val = atomic64_read(&dfx_item->value);

	return 0;
}

static int hpre_debugfs_atomic64_set(void *data, u64 val)
{
	struct hpre_dfx *dfx_item = data;
	struct hpre_dfx *hpre_dfx = NULL;

	if (dfx_item->type == HPRE_OVERTIME_THRHLD) {
		hpre_dfx = dfx_item - HPRE_OVERTIME_THRHLD;
		atomic64_set(&hpre_dfx[HPRE_OVER_THRHLD_CNT].value, 0);
	} else if (val) {
		return -EINVAL;
	}

	atomic64_set(&dfx_item->value, val);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(hpre_atomic64_ops, hpre_debugfs_atomic64_get,
			 hpre_debugfs_atomic64_set, "%llu\n");

static int hpre_com_regs_show(struct seq_file *s, void *unused)
{
	hisi_qm_regs_dump(s, s->private);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(hpre_com_regs);

static int hpre_cluster_regs_show(struct seq_file *s, void *unused)
{
	hisi_qm_regs_dump(s, s->private);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(hpre_cluster_regs);

static int hpre_create_debugfs_file(struct hisi_qm *qm, struct dentry *dir,
				    enum hpre_ctrl_dbgfs_file type, int indx)
{
	struct hpre *hpre = container_of(qm, struct hpre, qm);
	struct hpre_debug *dbg = &hpre->debug;
	struct dentry *file_dir;

	if (dir)
		file_dir = dir;
	else
		file_dir = qm->debug.debug_root;

	if (type >= HPRE_DEBUG_FILE_NUM)
		return -EINVAL;

	spin_lock_init(&dbg->files[indx].lock);
	dbg->files[indx].debug = dbg;
	dbg->files[indx].type = type;
	dbg->files[indx].index = indx;
	debugfs_create_file(hpre_debug_file_name[type], 0600, file_dir,
			    dbg->files + indx, &hpre_ctrl_debug_fops);

	return 0;
}

static int hpre_pf_comm_regs_debugfs_init(struct hisi_qm *qm)
{
	struct device *dev = &qm->pdev->dev;
	struct debugfs_regset32 *regset;

	regset = devm_kzalloc(dev, sizeof(*regset), GFP_KERNEL);
	if (!regset)
		return -ENOMEM;

	regset->regs = hpre_com_dfx_regs;
	regset->nregs = ARRAY_SIZE(hpre_com_dfx_regs);
	regset->base = qm->io_base;
	regset->dev = dev;

	debugfs_create_file("regs", 0444, qm->debug.debug_root,
			    regset, &hpre_com_regs_fops);

	return 0;
}

static int hpre_cluster_debugfs_init(struct hisi_qm *qm)
{
	u8 clusters_num = hpre_cluster_num(qm);
	struct device *dev = &qm->pdev->dev;
	char buf[HPRE_DBGFS_VAL_MAX_LEN];
	struct debugfs_regset32 *regset;
	struct dentry *tmp_d;
	int i, ret;

	for (i = 0; i < clusters_num; i++) {
		ret = snprintf(buf, HPRE_DBGFS_VAL_MAX_LEN, "cluster%d", i);
		if (ret < 0)
			return -EINVAL;
		tmp_d = debugfs_create_dir(buf, qm->debug.debug_root);

		regset = devm_kzalloc(dev, sizeof(*regset), GFP_KERNEL);
		if (!regset)
			return -ENOMEM;

		regset->regs = hpre_cluster_dfx_regs;
		regset->nregs = ARRAY_SIZE(hpre_cluster_dfx_regs);
		regset->base = qm->io_base + hpre_cluster_offsets[i];
		regset->dev = dev;

		debugfs_create_file("regs", 0444, tmp_d, regset,
				    &hpre_cluster_regs_fops);
		ret = hpre_create_debugfs_file(qm, tmp_d, HPRE_CLUSTER_CTRL,
					       i + HPRE_CLUSTER_CTRL);
		if (ret)
			return ret;
	}

	return 0;
}

static int hpre_ctrl_debug_init(struct hisi_qm *qm)
{
	int ret;

	ret = hpre_create_debugfs_file(qm, NULL, HPRE_CLEAR_ENABLE,
				       HPRE_CLEAR_ENABLE);
	if (ret)
		return ret;

	ret = hpre_pf_comm_regs_debugfs_init(qm);
	if (ret)
		return ret;

	return hpre_cluster_debugfs_init(qm);
}

static void hpre_dfx_debug_init(struct hisi_qm *qm)
{
	struct hpre *hpre = container_of(qm, struct hpre, qm);
	struct hpre_dfx *dfx = hpre->debug.dfx;
	struct dentry *parent;
	int i;

	parent = debugfs_create_dir("hpre_dfx", qm->debug.debug_root);
	for (i = 0; i < HPRE_DFX_FILE_NUM; i++) {
		dfx[i].type = i;
		debugfs_create_file(hpre_dfx_files[i], 0644, parent, &dfx[i],
				    &hpre_atomic64_ops);
	}
}

static int hpre_debugfs_init(struct hisi_qm *qm)
{
	struct device *dev = &qm->pdev->dev;
	int ret;

	qm->debug.debug_root = debugfs_create_dir(dev_name(dev),
						  hpre_debugfs_root);

	qm->debug.sqe_mask_offset = HPRE_SQE_MASK_OFFSET;
	qm->debug.sqe_mask_len = HPRE_SQE_MASK_LEN;
	hisi_qm_debug_init(qm);

	if (qm->pdev->device == HPRE_PCI_DEVICE_ID) {
		ret = hpre_ctrl_debug_init(qm);
		if (ret)
			goto failed_to_create;
	}

	hpre_dfx_debug_init(qm);

	return 0;

failed_to_create:
	debugfs_remove_recursive(qm->debug.debug_root);
	return ret;
}

static void hpre_debugfs_exit(struct hisi_qm *qm)
{
	debugfs_remove_recursive(qm->debug.debug_root);
}

static int hpre_qm_init(struct hisi_qm *qm, struct pci_dev *pdev)
{
	if (pdev->revision == QM_HW_V1) {
		pci_warn(pdev, "HPRE version 1 is not supported!\n");
		return -EINVAL;
	}

	if (pdev->revision >= QM_HW_V3)
		qm->algs = "rsa\ndh\necdh\nx25519\nx448\necdsa\nsm2";
	else
		qm->algs = "rsa\ndh";
	qm->mode = uacce_mode;
	qm->pdev = pdev;
	qm->ver = pdev->revision;
	qm->sqe_size = HPRE_SQE_SIZE;
	qm->dev_name = hpre_name;

	qm->fun_type = (pdev->device == HPRE_PCI_DEVICE_ID) ?
			QM_HW_PF : QM_HW_VF;
	if (qm->fun_type == QM_HW_PF) {
		qm->qp_base = HPRE_PF_DEF_Q_BASE;
		qm->qp_num = pf_q_num;
		qm->debug.curr_qm_qp_num = pf_q_num;
		qm->qm_list = &hpre_devices;
	}

	return hisi_qm_init(qm);
}

static void hpre_log_hw_error(struct hisi_qm *qm, u32 err_sts)
{
	const struct hpre_hw_error *err = hpre_hw_errors;
	struct device *dev = &qm->pdev->dev;

	while (err->msg) {
		if (err->int_msk & err_sts)
			dev_warn(dev, "%s [error status=0x%x] found\n",
				 err->msg, err->int_msk);
		err++;
	}
}

static u32 hpre_get_hw_err_status(struct hisi_qm *qm)
{
	return pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:988", qm->io_base + HPRE_INT_STATUS);
}

static void hpre_clear_hw_err_status(struct hisi_qm *qm, u32 err_sts)
{
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:993", err_sts, qm->io_base + HPRE_HAC_SOURCE_INT);
}

static void hpre_open_axi_master_ooo(struct hisi_qm *qm)
{
	u32 value;

	value = pete_readl("drivers/crypto/hisilicon/hpre/hpre_main.c:1000", qm->io_base + HPRE_AM_OOO_SHUTDOWN_ENB);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:1001", value & ~HPRE_AM_OOO_SHUTDOWN_ENABLE,
	       qm->io_base + HPRE_AM_OOO_SHUTDOWN_ENB);
	pete_writel("drivers/crypto/hisilicon/hpre/hpre_main.c:1003", value | HPRE_AM_OOO_SHUTDOWN_ENABLE,
	       qm->io_base + HPRE_AM_OOO_SHUTDOWN_ENB);
}

static void hpre_err_info_init(struct hisi_qm *qm)
{
	struct hisi_qm_err_info *err_info = &qm->err_info;

	err_info->ce = QM_BASE_CE;
	err_info->fe = 0;
	err_info->ecc_2bits_mask = HPRE_CORE_ECC_2BIT_ERR |
				   HPRE_OOO_ECC_2BIT_ERR;
	err_info->dev_ce_mask = HPRE_HAC_RAS_CE_ENABLE;
	err_info->msi_wr_port = HPRE_WR_MSI_PORT;
	err_info->acpi_rst = "HRST";
	err_info->nfe = QM_BASE_NFE | QM_ACC_DO_TASK_TIMEOUT;
}

static const struct hisi_qm_err_ini hpre_err_ini = {
	.hw_init		= hpre_set_user_domain_and_cache,
	.hw_err_enable		= hpre_hw_error_enable,
	.hw_err_disable		= hpre_hw_error_disable,
	.get_dev_hw_err_status	= hpre_get_hw_err_status,
	.clear_dev_hw_err_status = hpre_clear_hw_err_status,
	.log_dev_hw_err		= hpre_log_hw_error,
	.open_axi_master_ooo	= hpre_open_axi_master_ooo,
	.open_sva_prefetch	= hpre_open_sva_prefetch,
	.close_sva_prefetch	= hpre_close_sva_prefetch,
	.err_info_init		= hpre_err_info_init,
};

static int hpre_pf_probe_init(struct hpre *hpre)
{
	struct hisi_qm *qm = &hpre->qm;
	int ret;

	ret = hpre_set_user_domain_and_cache(qm);
	if (ret)
		return ret;

	hpre_open_sva_prefetch(qm);

	qm->err_ini = &hpre_err_ini;
	qm->err_ini->err_info_init(qm);
	hisi_qm_dev_err_init(qm);

	return 0;
}

static int hpre_probe_init(struct hpre *hpre)
{
	u32 type_rate = HPRE_SHAPER_TYPE_RATE;
	struct hisi_qm *qm = &hpre->qm;
	int ret;

	if (qm->fun_type == QM_HW_PF) {
		ret = hpre_pf_probe_init(hpre);
		if (ret)
			return ret;
		/* Enable shaper type 0 */
		if (qm->ver >= QM_HW_V3) {
			type_rate |= QM_SHAPER_ENABLE;
			qm->type_rate = type_rate;
		}
	}

	return 0;
}

static int hpre_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct hisi_qm *qm;
	struct hpre *hpre;
	int ret;

	hpre = devm_kzalloc(&pdev->dev, sizeof(*hpre), GFP_KERNEL);
	if (!hpre)
		return -ENOMEM;

	qm = &hpre->qm;
	ret = hpre_qm_init(qm, pdev);
	if (ret) {
		pci_err(pdev, "Failed to init HPRE QM (%d)!\n", ret);
		return ret;
	}

	ret = hpre_probe_init(hpre);
	if (ret) {
		pci_err(pdev, "Failed to probe (%d)!\n", ret);
		goto err_with_qm_init;
	}

	ret = hisi_qm_start(qm);
	if (ret)
		goto err_with_err_init;

	ret = hpre_debugfs_init(qm);
	if (ret)
		dev_warn(&pdev->dev, "init debugfs fail!\n");

	ret = hisi_qm_alg_register(qm, &hpre_devices);
	if (ret < 0) {
		pci_err(pdev, "fail to register algs to crypto!\n");
		goto err_with_qm_start;
	}

	if (qm->uacce) {
		ret = uacce_register(qm->uacce);
		if (ret) {
			pci_err(pdev, "failed to register uacce (%d)!\n", ret);
			goto err_with_alg_register;
		}
	}

	if (qm->fun_type == QM_HW_PF && vfs_num) {
		ret = hisi_qm_sriov_enable(pdev, vfs_num);
		if (ret < 0)
			goto err_with_alg_register;
	}

	hisi_qm_pm_init(qm);

	return 0;

err_with_alg_register:
	hisi_qm_alg_unregister(qm, &hpre_devices);

err_with_qm_start:
	hpre_debugfs_exit(qm);
	hisi_qm_stop(qm, QM_NORMAL);

err_with_err_init:
	hisi_qm_dev_err_uninit(qm);

err_with_qm_init:
	hisi_qm_uninit(qm);

	return ret;
}

static void hpre_remove(struct pci_dev *pdev)
{
	struct hisi_qm *qm = pci_get_drvdata(pdev);

	hisi_qm_pm_uninit(qm);
	hisi_qm_wait_task_finish(qm, &hpre_devices);
	hisi_qm_alg_unregister(qm, &hpre_devices);
	if (qm->fun_type == QM_HW_PF && qm->vfs_num)
		hisi_qm_sriov_disable(pdev, true);

	hpre_debugfs_exit(qm);
	hisi_qm_stop(qm, QM_NORMAL);

	if (qm->fun_type == QM_HW_PF) {
		hpre_cnt_regs_clear(qm);
		qm->debug.curr_qm_qp_num = 0;
		hisi_qm_dev_err_uninit(qm);
	}

	hisi_qm_uninit(qm);
}

static const struct dev_pm_ops hpre_pm_ops = {
	SET_RUNTIME_PM_OPS(hisi_qm_suspend, hisi_qm_resume, NULL)
};

static const struct pci_error_handlers hpre_err_handler = {
	.error_detected		= hisi_qm_dev_err_detected,
	.slot_reset		= hisi_qm_dev_slot_reset,
	.reset_prepare		= hisi_qm_reset_prepare,
	.reset_done		= hisi_qm_reset_done,
};

static struct pci_driver hpre_pci_driver = {
	.name			= hpre_name,
	.id_table		= hpre_dev_ids,
	.probe			= hpre_probe,
	.remove			= hpre_remove,
	.sriov_configure	= IS_ENABLED(CONFIG_PCI_IOV) ?
				  hisi_qm_sriov_configure : NULL,
	.err_handler		= &hpre_err_handler,
	.shutdown		= hisi_qm_dev_shutdown,
	.driver.pm		= &hpre_pm_ops,
};

static void hpre_register_debugfs(void)
{
	if (!debugfs_initialized())
		return;

	hpre_debugfs_root = debugfs_create_dir(hpre_name, NULL);
}

static void hpre_unregister_debugfs(void)
{
	debugfs_remove_recursive(hpre_debugfs_root);
}

static int __init hpre_init(void)
{
	int ret;

	hisi_qm_init_list(&hpre_devices);
	hpre_register_debugfs();

	ret = pci_register_driver(&hpre_pci_driver);
	if (ret) {
		hpre_unregister_debugfs();
		pr_err("hpre: can't register hisi hpre driver.\n");
	}

	return ret;
}

static void __exit hpre_exit(void)
{
	pci_unregister_driver(&hpre_pci_driver);
	hpre_unregister_debugfs();
}

module_init(hpre_init);
module_exit(hpre_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Zaibo Xu <xuzaibo@huawei.com>");
MODULE_AUTHOR("Meng Yu <yumeng18@huawei.com>");
MODULE_DESCRIPTION("Driver for HiSilicon HPRE accelerator");
