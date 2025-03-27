// SPDX-License-Identifier: GPL-2.0-only
/*
 * QLogic iSCSI HBA Driver
 * Copyright (c)  2003-2012 QLogic Corporation
 */

#include "ql4_def.h"
#include "ql4_glbl.h"
#include "ql4_dbg.h"
#include "ql4_inline.h"

void qla4xxx_dump_buffer(void *b, uint32_t size)
{
	uint32_t cnt;
	uint8_t *c = b;

	printk(" 0   1   2   3   4   5   6   7   8   9  Ah  Bh  Ch  Dh  Eh  "
	       "Fh\n");
	printk("------------------------------------------------------------"
	       "--\n");
	for (cnt = 0; cnt < size; c++) {
		printk("%02x", *c);
		if (!(++cnt % 16))
			printk("\n");

		else
			printk("  ");
	}
	printk(KERN_INFO "\n");
}

void qla4xxx_dump_registers(struct scsi_qla_host *ha)
{
	uint8_t i;

	if (is_qla8022(ha)) {
		for (i = 1; i < MBOX_REG_COUNT; i++)
			printk(KERN_INFO "mailbox[%d]     = 0x%08X\n",
			    i, pete_readl("drivers/scsi/qla4xxx/ql4_dbg.c:39", &ha->qla4_82xx_reg->mailbox_in[i]));
		return;
	}

	for (i = 0; i < MBOX_REG_COUNT; i++) {
		printk(KERN_INFO "0x%02X mailbox[%d]      = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, mailbox[i]), i,
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:46", &ha->reg->mailbox[i]));
	}

	printk(KERN_INFO "0x%02X flash_address            = 0x%08X\n",
	    (uint8_t) offsetof(struct isp_reg, flash_address),
	    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:51", &ha->reg->flash_address));
	printk(KERN_INFO "0x%02X flash_data               = 0x%08X\n",
	    (uint8_t) offsetof(struct isp_reg, flash_data),
	    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:54", &ha->reg->flash_data));
	printk(KERN_INFO "0x%02X ctrl_status              = 0x%08X\n",
	    (uint8_t) offsetof(struct isp_reg, ctrl_status),
	    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:57", &ha->reg->ctrl_status));

	if (is_qla4010(ha)) {
		printk(KERN_INFO "0x%02X nvram            = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u1.isp4010.nvram),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:62", &ha->reg->u1.isp4010.nvram));
	} else if (is_qla4022(ha) | is_qla4032(ha)) {
		printk(KERN_INFO "0x%02X intr_mask        = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u1.isp4022.intr_mask),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:66", &ha->reg->u1.isp4022.intr_mask));
		printk(KERN_INFO "0x%02X nvram            = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u1.isp4022.nvram),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:69", &ha->reg->u1.isp4022.nvram));
		printk(KERN_INFO "0x%02X semaphore	  = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u1.isp4022.semaphore),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:72", &ha->reg->u1.isp4022.semaphore));
	}
	printk(KERN_INFO "0x%02X req_q_in                 = 0x%08X\n",
	    (uint8_t) offsetof(struct isp_reg, req_q_in),
	    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:76", &ha->reg->req_q_in));
	printk(KERN_INFO "0x%02X rsp_q_out                = 0x%08X\n",
	    (uint8_t) offsetof(struct isp_reg, rsp_q_out),
	    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:79", &ha->reg->rsp_q_out));

	if (is_qla4010(ha)) {
		printk(KERN_INFO "0x%02X ext_hw_conf      = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4010.ext_hw_conf),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:84", &ha->reg->u2.isp4010.ext_hw_conf));
		printk(KERN_INFO "0x%02X port_ctrl        = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4010.port_ctrl),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:87", &ha->reg->u2.isp4010.port_ctrl));
		printk(KERN_INFO "0x%02X port_status      = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4010.port_status),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:90", &ha->reg->u2.isp4010.port_status));
		printk(KERN_INFO "0x%02X req_q_out        = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4010.req_q_out),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:93", &ha->reg->u2.isp4010.req_q_out));
		printk(KERN_INFO "0x%02X gp_out           = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4010.gp_out),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:96", &ha->reg->u2.isp4010.gp_out));
		printk(KERN_INFO "0x%02X gp_in	          = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4010.gp_in),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:99", &ha->reg->u2.isp4010.gp_in));
		printk(KERN_INFO "0x%02X port_err_status  = 0x%08X\n", (uint8_t)
		    offsetof(struct isp_reg, u2.isp4010.port_err_status),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:102", &ha->reg->u2.isp4010.port_err_status));
	} else if (is_qla4022(ha) | is_qla4032(ha)) {
		printk(KERN_INFO "Page 0 Registers:\n");
		printk(KERN_INFO "0x%02X ext_hw_conf      = 0x%08X\n", (uint8_t)
		    offsetof(struct isp_reg, u2.isp4022.p0.ext_hw_conf),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:107", &ha->reg->u2.isp4022.p0.ext_hw_conf));
		printk(KERN_INFO "0x%02X port_ctrl        = 0x%08X\n", (uint8_t)
		    offsetof(struct isp_reg, u2.isp4022.p0.port_ctrl),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:110", &ha->reg->u2.isp4022.p0.port_ctrl));
		printk(KERN_INFO "0x%02X port_status      = 0x%08X\n", (uint8_t)
		    offsetof(struct isp_reg, u2.isp4022.p0.port_status),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:113", &ha->reg->u2.isp4022.p0.port_status));
		printk(KERN_INFO "0x%02X gp_out           = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4022.p0.gp_out),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:116", &ha->reg->u2.isp4022.p0.gp_out));
		printk(KERN_INFO "0x%02X gp_in            = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4022.p0.gp_in),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:119", &ha->reg->u2.isp4022.p0.gp_in));
		printk(KERN_INFO "0x%02X port_err_status  = 0x%08X\n", (uint8_t)
		    offsetof(struct isp_reg, u2.isp4022.p0.port_err_status),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:122", &ha->reg->u2.isp4022.p0.port_err_status));
		printk(KERN_INFO "Page 1 Registers:\n");
		pete_writel("drivers/scsi/qla4xxx/ql4_dbg.c:124", HOST_MEM_CFG_PAGE & set_rmask(CSR_SCSI_PAGE_SELECT),
		    &ha->reg->ctrl_status);
		printk(KERN_INFO "0x%02X req_q_out        = 0x%08X\n",
		    (uint8_t) offsetof(struct isp_reg, u2.isp4022.p1.req_q_out),
		    pete_readw("drivers/scsi/qla4xxx/ql4_dbg.c:128", &ha->reg->u2.isp4022.p1.req_q_out));
		pete_writel("drivers/scsi/qla4xxx/ql4_dbg.c:129", PORT_CTRL_STAT_PAGE & set_rmask(CSR_SCSI_PAGE_SELECT),
		    &ha->reg->ctrl_status);
	}
}

void qla4_8xxx_dump_peg_reg(struct scsi_qla_host *ha)
{
	uint32_t halt_status1, halt_status2;

	halt_status1 = qla4_8xxx_rd_direct(ha, QLA8XXX_PEG_HALT_STATUS1);
	halt_status2 = qla4_8xxx_rd_direct(ha, QLA8XXX_PEG_HALT_STATUS2);

	if (is_qla8022(ha)) {
		ql4_printk(KERN_INFO, ha,
			   "scsi(%ld): %s, ISP%04x Dumping hw/fw registers:\n"
			   " PEG_HALT_STATUS1: 0x%x, PEG_HALT_STATUS2: 0x%x,\n"
			   " PEG_NET_0_PC: 0x%x, PEG_NET_1_PC: 0x%x,\n"
			   " PEG_NET_2_PC: 0x%x, PEG_NET_3_PC: 0x%x,\n"
			   " PEG_NET_4_PC: 0x%x\n", ha->host_no, __func__,
			   ha->pdev->device, halt_status1, halt_status2,
			   qla4_82xx_rd_32(ha, QLA82XX_CRB_PEG_NET_0 + 0x3c),
			   qla4_82xx_rd_32(ha, QLA82XX_CRB_PEG_NET_1 + 0x3c),
			   qla4_82xx_rd_32(ha, QLA82XX_CRB_PEG_NET_2 + 0x3c),
			   qla4_82xx_rd_32(ha, QLA82XX_CRB_PEG_NET_3 + 0x3c),
			   qla4_82xx_rd_32(ha, QLA82XX_CRB_PEG_NET_4 + 0x3c));
	} else if (is_qla8032(ha) || is_qla8042(ha)) {
		ql4_printk(KERN_INFO, ha,
			   "scsi(%ld): %s, ISP%04x Dumping hw/fw registers:\n"
			   " PEG_HALT_STATUS1: 0x%x, PEG_HALT_STATUS2: 0x%x,\n",
			   ha->host_no, __func__, ha->pdev->device,
			   halt_status1, halt_status2);
	}
}
