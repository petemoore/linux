// SPDX-License-Identifier: GPL-2.0-only
/* linux/drivers/media/platform/s5p-jpeg/jpeg-hw.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Andrzej Pietrasiewicz <andrzejtp2010@gmail.com>
 */

#include <linux/io.h>
#include <linux/videodev2.h>

#include "jpeg-core.h"
#include "jpeg-regs.h"
#include "jpeg-hw-s5p.h"

void s5p_jpeg_reset(void __iomem *regs)
{
	unsigned long reg;

	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:21", 1, regs + S5P_JPG_SW_RESET);
	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:22", regs + S5P_JPG_SW_RESET);
	/* no other way but polling for when JPEG IP becomes operational */
	while (reg != 0) {
		cpu_relax();
		reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:26", regs + S5P_JPG_SW_RESET);
	}
}

void s5p_jpeg_poweron(void __iomem *regs)
{
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:32", S5P_POWER_ON, regs + S5P_JPGCLKCON);
}

void s5p_jpeg_input_raw_mode(void __iomem *regs, unsigned long mode)
{
	unsigned long reg, m;

	m = S5P_MOD_SEL_565;
	if (mode == S5P_JPEG_RAW_IN_565)
		m = S5P_MOD_SEL_565;
	else if (mode == S5P_JPEG_RAW_IN_422)
		m = S5P_MOD_SEL_422;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:45", regs + S5P_JPGCMOD);
	reg &= ~S5P_MOD_SEL_MASK;
	reg |= m;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:48", reg, regs + S5P_JPGCMOD);
}

void s5p_jpeg_proc_mode(void __iomem *regs, unsigned long mode)
{
	unsigned long reg, m;

	m = S5P_PROC_MODE_DECOMPR;
	if (mode == S5P_JPEG_ENCODE)
		m = S5P_PROC_MODE_COMPR;
	else
		m = S5P_PROC_MODE_DECOMPR;
	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:60", regs + S5P_JPGMOD);
	reg &= ~S5P_PROC_MODE_MASK;
	reg |= m;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:63", reg, regs + S5P_JPGMOD);
}

void s5p_jpeg_subsampling_mode(void __iomem *regs, unsigned int mode)
{
	unsigned long reg, m;

	if (mode == V4L2_JPEG_CHROMA_SUBSAMPLING_420)
		m = S5P_SUBSAMPLING_MODE_420;
	else
		m = S5P_SUBSAMPLING_MODE_422;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:75", regs + S5P_JPGMOD);
	reg &= ~S5P_SUBSAMPLING_MODE_MASK;
	reg |= m;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:78", reg, regs + S5P_JPGMOD);
}

unsigned int s5p_jpeg_get_subsampling_mode(void __iomem *regs)
{
	return pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:83", regs + S5P_JPGMOD) & S5P_SUBSAMPLING_MODE_MASK;
}

void s5p_jpeg_dri(void __iomem *regs, unsigned int dri)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:90", regs + S5P_JPGDRI_U);
	reg &= ~0xff;
	reg |= (dri >> 8) & 0xff;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:93", reg, regs + S5P_JPGDRI_U);

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:95", regs + S5P_JPGDRI_L);
	reg &= ~0xff;
	reg |= dri & 0xff;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:98", reg, regs + S5P_JPGDRI_L);
}

void s5p_jpeg_qtbl(void __iomem *regs, unsigned int t, unsigned int n)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:105", regs + S5P_JPG_QTBL);
	reg &= ~S5P_QT_NUMt_MASK(t);
	reg |= (n << S5P_QT_NUMt_SHIFT(t)) & S5P_QT_NUMt_MASK(t);
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:108", reg, regs + S5P_JPG_QTBL);
}

void s5p_jpeg_htbl_ac(void __iomem *regs, unsigned int t)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:115", regs + S5P_JPG_HTBL);
	reg &= ~S5P_HT_NUMt_AC_MASK(t);
	/* this driver uses table 0 for all color components */
	reg |= (0 << S5P_HT_NUMt_AC_SHIFT(t)) & S5P_HT_NUMt_AC_MASK(t);
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:119", reg, regs + S5P_JPG_HTBL);
}

void s5p_jpeg_htbl_dc(void __iomem *regs, unsigned int t)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:126", regs + S5P_JPG_HTBL);
	reg &= ~S5P_HT_NUMt_DC_MASK(t);
	/* this driver uses table 0 for all color components */
	reg |= (0 << S5P_HT_NUMt_DC_SHIFT(t)) & S5P_HT_NUMt_DC_MASK(t);
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:130", reg, regs + S5P_JPG_HTBL);
}

void s5p_jpeg_y(void __iomem *regs, unsigned int y)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:137", regs + S5P_JPGY_U);
	reg &= ~0xff;
	reg |= (y >> 8) & 0xff;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:140", reg, regs + S5P_JPGY_U);

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:142", regs + S5P_JPGY_L);
	reg &= ~0xff;
	reg |= y & 0xff;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:145", reg, regs + S5P_JPGY_L);
}

void s5p_jpeg_x(void __iomem *regs, unsigned int x)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:152", regs + S5P_JPGX_U);
	reg &= ~0xff;
	reg |= (x >> 8) & 0xff;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:155", reg, regs + S5P_JPGX_U);

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:157", regs + S5P_JPGX_L);
	reg &= ~0xff;
	reg |= x & 0xff;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:160", reg, regs + S5P_JPGX_L);
}

void s5p_jpeg_rst_int_enable(void __iomem *regs, bool enable)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:167", regs + S5P_JPGINTSE);
	reg &= ~S5P_RSTm_INT_EN_MASK;
	if (enable)
		reg |= S5P_RSTm_INT_EN;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:171", reg, regs + S5P_JPGINTSE);
}

void s5p_jpeg_data_num_int_enable(void __iomem *regs, bool enable)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:178", regs + S5P_JPGINTSE);
	reg &= ~S5P_DATA_NUM_INT_EN_MASK;
	if (enable)
		reg |= S5P_DATA_NUM_INT_EN;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:182", reg, regs + S5P_JPGINTSE);
}

void s5p_jpeg_final_mcu_num_int_enable(void __iomem *regs, bool enbl)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:189", regs + S5P_JPGINTSE);
	reg &= ~S5P_FINAL_MCU_NUM_INT_EN_MASK;
	if (enbl)
		reg |= S5P_FINAL_MCU_NUM_INT_EN;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:193", reg, regs + S5P_JPGINTSE);
}

int s5p_jpeg_timer_stat(void __iomem *regs)
{
	return (int)((pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:198", regs + S5P_JPG_TIMER_ST) & S5P_TIMER_INT_STAT_MASK)
		     >> S5P_TIMER_INT_STAT_SHIFT);
}

void s5p_jpeg_clear_timer_stat(void __iomem *regs)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:206", regs + S5P_JPG_TIMER_SE);
	reg &= ~S5P_TIMER_INT_STAT_MASK;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:208", reg, regs + S5P_JPG_TIMER_SE);
}

void s5p_jpeg_enc_stream_int(void __iomem *regs, unsigned long size)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:215", regs + S5P_JPG_ENC_STREAM_INTSE);
	reg &= ~S5P_ENC_STREAM_BOUND_MASK;
	reg |= S5P_ENC_STREAM_INT_EN;
	reg |= size & S5P_ENC_STREAM_BOUND_MASK;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:219", reg, regs + S5P_JPG_ENC_STREAM_INTSE);
}

int s5p_jpeg_enc_stream_stat(void __iomem *regs)
{
	return (int)(pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:224", regs + S5P_JPG_ENC_STREAM_INTST) &
		     S5P_ENC_STREAM_INT_STAT_MASK);
}

void s5p_jpeg_clear_enc_stream_stat(void __iomem *regs)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:232", regs + S5P_JPG_ENC_STREAM_INTSE);
	reg &= ~S5P_ENC_STREAM_INT_MASK;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:234", reg, regs + S5P_JPG_ENC_STREAM_INTSE);
}

void s5p_jpeg_outform_raw(void __iomem *regs, unsigned long format)
{
	unsigned long reg, f;

	f = S5P_DEC_OUT_FORMAT_422;
	if (format == S5P_JPEG_RAW_OUT_422)
		f = S5P_DEC_OUT_FORMAT_422;
	else if (format == S5P_JPEG_RAW_OUT_420)
		f = S5P_DEC_OUT_FORMAT_420;
	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:246", regs + S5P_JPG_OUTFORM);
	reg &= ~S5P_DEC_OUT_FORMAT_MASK;
	reg |= f;
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:249", reg, regs + S5P_JPG_OUTFORM);
}

void s5p_jpeg_jpgadr(void __iomem *regs, unsigned long addr)
{
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:254", addr, regs + S5P_JPG_JPGADR);
}

void s5p_jpeg_imgadr(void __iomem *regs, unsigned long addr)
{
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:259", addr, regs + S5P_JPG_IMGADR);
}

void s5p_jpeg_coef(void __iomem *regs, unsigned int i,
			     unsigned int j, unsigned int coef)
{
	unsigned long reg;

	reg = pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:267", regs + S5P_JPG_COEF(i));
	reg &= ~S5P_COEFn_MASK(j);
	reg |= (coef << S5P_COEFn_SHIFT(j)) & S5P_COEFn_MASK(j);
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:270", reg, regs + S5P_JPG_COEF(i));
}

void s5p_jpeg_start(void __iomem *regs)
{
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:275", 1, regs + S5P_JSTART);
}

int s5p_jpeg_result_stat_ok(void __iomem *regs)
{
	return (int)((pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:280", regs + S5P_JPGINTST) & S5P_RESULT_STAT_MASK)
		     >> S5P_RESULT_STAT_SHIFT);
}

int s5p_jpeg_stream_stat_ok(void __iomem *regs)
{
	return !(int)((pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:286", regs + S5P_JPGINTST) & S5P_STREAM_STAT_MASK)
		      >> S5P_STREAM_STAT_SHIFT);
}

void s5p_jpeg_clear_int(void __iomem *regs)
{
	pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:292", regs + S5P_JPGINTST);
	pete_writel("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:293", S5P_INT_RELEASE, regs + S5P_JPGCOM);
	pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:294", regs + S5P_JPGOPR);
}

unsigned int s5p_jpeg_compressed_size(void __iomem *regs)
{
	unsigned long jpeg_size = 0;

	jpeg_size |= (pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:301", regs + S5P_JPGCNT_U) & 0xff) << 16;
	jpeg_size |= (pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:302", regs + S5P_JPGCNT_M) & 0xff) << 8;
	jpeg_size |= (pete_readl("drivers/media/platform/s5p-jpeg/jpeg-hw-s5p.c:303", regs + S5P_JPGCNT_L) & 0xff);

	return (unsigned int)jpeg_size;
}
