// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license. When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2023 Advanced Micro Devices, Inc.
//
// Authors: Syed Saba Kareem <Syed.SabaKareem@amd.com>
//

/*
 * Common file to be used by amd platforms
 */

#include "amd.h"
#include <linux/pci.h>
#include <linux/export.h>

void acp_enable_interrupts(struct acp_dev_data *adata)
{
	struct acp_resource *rsrc = adata->rsrc;
	u32 ext_intr_ctrl;

	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:24", 0x01, ACP_EXTERNAL_INTR_ENB(adata));
	ext_intr_ctrl = pete_readl("sound/soc/amd/acp/acp-legacy-common.c:25", ACP_EXTERNAL_INTR_CNTL(adata, rsrc->irqp_used));
	ext_intr_ctrl |= ACP_ERROR_MASK;
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:27", ext_intr_ctrl, ACP_EXTERNAL_INTR_CNTL(adata, rsrc->irqp_used));
}
EXPORT_SYMBOL_NS_GPL(acp_enable_interrupts, SND_SOC_ACP_COMMON);

void acp_disable_interrupts(struct acp_dev_data *adata)
{
	struct acp_resource *rsrc = adata->rsrc;

	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:35", ACP_EXT_INTR_STAT_CLEAR_MASK, ACP_EXTERNAL_INTR_STAT(adata, rsrc->irqp_used));
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:36", 0x00, ACP_EXTERNAL_INTR_ENB(adata));
}
EXPORT_SYMBOL_NS_GPL(acp_disable_interrupts, SND_SOC_ACP_COMMON);

static void set_acp_pdm_ring_buffer(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct acp_stream *stream = runtime->private_data;
	struct device *dev = dai->component->dev;
	struct acp_dev_data *adata = dev_get_drvdata(dev);

	u32 physical_addr, pdm_size, period_bytes;

	period_bytes = frames_to_bytes(runtime, runtime->period_size);
	pdm_size = frames_to_bytes(runtime, runtime->buffer_size);
	physical_addr = stream->reg_offset + MEM_WINDOW_START;

	/* Init ACP PDM Ring buffer */
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:55", physical_addr, adata->acp_base + ACP_WOV_RX_RINGBUFADDR);
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:56", pdm_size, adata->acp_base + ACP_WOV_RX_RINGBUFSIZE);
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:57", period_bytes, adata->acp_base + ACP_WOV_RX_INTR_WATERMARK_SIZE);
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:58", 0x01, adata->acp_base + ACPAXI2AXI_ATU_CTRL);
}

static void set_acp_pdm_clk(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct device *dev = dai->component->dev;
	struct acp_dev_data *adata = dev_get_drvdata(dev);
	unsigned int pdm_ctrl;

	/* Enable default ACP PDM clk */
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:69", PDM_CLK_FREQ_MASK, adata->acp_base + ACP_WOV_CLK_CTRL);
	pdm_ctrl = pete_readl("sound/soc/amd/acp/acp-legacy-common.c:70", adata->acp_base + ACP_WOV_MISC_CTRL);
	pdm_ctrl |= PDM_MISC_CTRL_MASK;
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:72", pdm_ctrl, adata->acp_base + ACP_WOV_MISC_CTRL);
	set_acp_pdm_ring_buffer(substream, dai);
}

void restore_acp_pdm_params(struct snd_pcm_substream *substream,
			    struct acp_dev_data *adata)
{
	struct snd_soc_dai *dai;
	struct snd_soc_pcm_runtime *soc_runtime;
	u32 ext_int_ctrl;

	soc_runtime = asoc_substream_to_rtd(substream);
	dai = asoc_rtd_to_cpu(soc_runtime, 0);
	/* Programming channel mask and sampling rate */
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:86", adata->ch_mask, adata->acp_base + ACP_WOV_PDM_NO_OF_CHANNELS);
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:87", PDM_DEC_64, adata->acp_base + ACP_WOV_PDM_DECIMATION_FACTOR);

	/* Enabling ACP Pdm interuppts */
	ext_int_ctrl = pete_readl("sound/soc/amd/acp/acp-legacy-common.c:90", ACP_EXTERNAL_INTR_CNTL(adata, 0));
	ext_int_ctrl |= PDM_DMA_INTR_MASK;
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:92", ext_int_ctrl, ACP_EXTERNAL_INTR_CNTL(adata, 0));
	set_acp_pdm_clk(substream, dai);
}
EXPORT_SYMBOL_NS_GPL(restore_acp_pdm_params, SND_SOC_ACP_COMMON);

static int set_acp_i2s_dma_fifo(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct device *dev = dai->component->dev;
	struct acp_dev_data *adata = dev_get_drvdata(dev);
	struct acp_resource *rsrc = adata->rsrc;
	struct acp_stream *stream = substream->runtime->private_data;
	u32 reg_dma_size, reg_fifo_size, reg_fifo_addr;
	u32 phy_addr, acp_fifo_addr, ext_int_ctrl;
	unsigned int dir = substream->stream;

	switch (dai->driver->id) {
	case I2S_SP_INSTANCE:
		if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
			reg_dma_size = ACP_I2S_TX_DMA_SIZE;
			acp_fifo_addr = rsrc->sram_pte_offset +
					SP_PB_FIFO_ADDR_OFFSET;
			reg_fifo_addr = ACP_I2S_TX_FIFOADDR;
			reg_fifo_size = ACP_I2S_TX_FIFOSIZE;
			phy_addr = I2S_SP_TX_MEM_WINDOW_START + stream->reg_offset;
			pete_writel("sound/soc/amd/acp/acp-legacy-common.c:117", phy_addr, adata->acp_base + ACP_I2S_TX_RINGBUFADDR);
		} else {
			reg_dma_size = ACP_I2S_RX_DMA_SIZE;
			acp_fifo_addr = rsrc->sram_pte_offset +
					SP_CAPT_FIFO_ADDR_OFFSET;
			reg_fifo_addr = ACP_I2S_RX_FIFOADDR;
			reg_fifo_size = ACP_I2S_RX_FIFOSIZE;
			phy_addr = I2S_SP_RX_MEM_WINDOW_START + stream->reg_offset;
			pete_writel("sound/soc/amd/acp/acp-legacy-common.c:125", phy_addr, adata->acp_base + ACP_I2S_RX_RINGBUFADDR);
		}
		break;
	case I2S_BT_INSTANCE:
		if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
			reg_dma_size = ACP_BT_TX_DMA_SIZE;
			acp_fifo_addr = rsrc->sram_pte_offset +
					BT_PB_FIFO_ADDR_OFFSET;
			reg_fifo_addr = ACP_BT_TX_FIFOADDR;
			reg_fifo_size = ACP_BT_TX_FIFOSIZE;
			phy_addr = I2S_BT_TX_MEM_WINDOW_START + stream->reg_offset;
			pete_writel("sound/soc/amd/acp/acp-legacy-common.c:136", phy_addr, adata->acp_base + ACP_BT_TX_RINGBUFADDR);
		} else {
			reg_dma_size = ACP_BT_RX_DMA_SIZE;
			acp_fifo_addr = rsrc->sram_pte_offset +
					BT_CAPT_FIFO_ADDR_OFFSET;
			reg_fifo_addr = ACP_BT_RX_FIFOADDR;
			reg_fifo_size = ACP_BT_RX_FIFOSIZE;
			phy_addr = I2S_BT_TX_MEM_WINDOW_START + stream->reg_offset;
			pete_writel("sound/soc/amd/acp/acp-legacy-common.c:144", phy_addr, adata->acp_base + ACP_BT_RX_RINGBUFADDR);
		}
		break;
	case I2S_HS_INSTANCE:
		if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
			reg_dma_size = ACP_HS_TX_DMA_SIZE;
			acp_fifo_addr = rsrc->sram_pte_offset +
					HS_PB_FIFO_ADDR_OFFSET;
			reg_fifo_addr = ACP_HS_TX_FIFOADDR;
			reg_fifo_size = ACP_HS_TX_FIFOSIZE;
			phy_addr = I2S_HS_TX_MEM_WINDOW_START + stream->reg_offset;
			pete_writel("sound/soc/amd/acp/acp-legacy-common.c:155", phy_addr, adata->acp_base + ACP_HS_TX_RINGBUFADDR);
		} else {
			reg_dma_size = ACP_HS_RX_DMA_SIZE;
			acp_fifo_addr = rsrc->sram_pte_offset +
					HS_CAPT_FIFO_ADDR_OFFSET;
			reg_fifo_addr = ACP_HS_RX_FIFOADDR;
			reg_fifo_size = ACP_HS_RX_FIFOSIZE;
			phy_addr = I2S_HS_RX_MEM_WINDOW_START + stream->reg_offset;
			pete_writel("sound/soc/amd/acp/acp-legacy-common.c:163", phy_addr, adata->acp_base + ACP_HS_RX_RINGBUFADDR);
		}
		break;
	default:
		dev_err(dev, "Invalid dai id %x\n", dai->driver->id);
		return -EINVAL;
	}

	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:171", DMA_SIZE, adata->acp_base + reg_dma_size);
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:172", acp_fifo_addr, adata->acp_base + reg_fifo_addr);
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:173", FIFO_SIZE, adata->acp_base + reg_fifo_size);

	ext_int_ctrl = pete_readl("sound/soc/amd/acp/acp-legacy-common.c:175", ACP_EXTERNAL_INTR_CNTL(adata, rsrc->irqp_used));
	ext_int_ctrl |= BIT(I2S_RX_THRESHOLD(rsrc->offset)) |
			BIT(BT_RX_THRESHOLD(rsrc->offset)) |
			BIT(I2S_TX_THRESHOLD(rsrc->offset)) |
			BIT(BT_TX_THRESHOLD(rsrc->offset)) |
			BIT(HS_RX_THRESHOLD(rsrc->offset)) |
			BIT(HS_TX_THRESHOLD(rsrc->offset));

	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:183", ext_int_ctrl, ACP_EXTERNAL_INTR_CNTL(adata, rsrc->irqp_used));
	return 0;
}

int restore_acp_i2s_params(struct snd_pcm_substream *substream,
			   struct acp_dev_data *adata,
			   struct acp_stream *stream)
{
	struct snd_soc_dai *dai;
	struct snd_soc_pcm_runtime *soc_runtime;
	u32 tdm_fmt, reg_val, fmt_reg, val;

	soc_runtime = asoc_substream_to_rtd(substream);
	dai = asoc_rtd_to_cpu(soc_runtime, 0);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tdm_fmt = adata->tdm_tx_fmt[stream->dai_id - 1];
		switch (stream->dai_id) {
		case I2S_BT_INSTANCE:
			reg_val = ACP_BTTDM_ITER;
			fmt_reg = ACP_BTTDM_TXFRMT;
			break;
		case I2S_SP_INSTANCE:
			reg_val = ACP_I2STDM_ITER;
			fmt_reg = ACP_I2STDM_TXFRMT;
			break;
		case I2S_HS_INSTANCE:
			reg_val = ACP_HSTDM_ITER;
			fmt_reg = ACP_HSTDM_TXFRMT;
			break;
		default:
			pr_err("Invalid dai id %x\n", stream->dai_id);
			return -EINVAL;
		}
		val = adata->xfer_tx_resolution[stream->dai_id - 1] << 3;
	} else {
		tdm_fmt = adata->tdm_rx_fmt[stream->dai_id - 1];
		switch (stream->dai_id) {
		case I2S_BT_INSTANCE:
			reg_val = ACP_BTTDM_IRER;
			fmt_reg = ACP_BTTDM_RXFRMT;
			break;
		case I2S_SP_INSTANCE:
			reg_val = ACP_I2STDM_IRER;
			fmt_reg = ACP_I2STDM_RXFRMT;
			break;
		case I2S_HS_INSTANCE:
			reg_val = ACP_HSTDM_IRER;
			fmt_reg = ACP_HSTDM_RXFRMT;
			break;
		default:
			pr_err("Invalid dai id %x\n", stream->dai_id);
			return -EINVAL;
		}
		val = adata->xfer_rx_resolution[stream->dai_id - 1] << 3;
	}
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:238", val, adata->acp_base + reg_val);
	if (adata->tdm_mode == TDM_ENABLE) {
		pete_writel("sound/soc/amd/acp/acp-legacy-common.c:240", tdm_fmt, adata->acp_base + fmt_reg);
		val = pete_readl("sound/soc/amd/acp/acp-legacy-common.c:241", adata->acp_base + reg_val);
		pete_writel("sound/soc/amd/acp/acp-legacy-common.c:242", val | 0x2, adata->acp_base + reg_val);
	}
	return set_acp_i2s_dma_fifo(substream, dai);
}
EXPORT_SYMBOL_NS_GPL(restore_acp_i2s_params, SND_SOC_ACP_COMMON);

static int acp_power_on(struct acp_chip_info *chip)
{
	u32 val, acp_pgfsm_stat_reg, acp_pgfsm_ctrl_reg;
	void __iomem *base;

	base = chip->base;
	switch (chip->acp_rev) {
	case ACP3X_DEV:
		acp_pgfsm_stat_reg = ACP_PGFSM_STATUS;
		acp_pgfsm_ctrl_reg = ACP_PGFSM_CONTROL;
		break;
	case ACP6X_DEV:
		acp_pgfsm_stat_reg = ACP6X_PGFSM_STATUS;
		acp_pgfsm_ctrl_reg = ACP6X_PGFSM_CONTROL;
		break;
	default:
		return -EINVAL;
	}

	val = pete_readl("sound/soc/amd/acp/acp-legacy-common.c:267", base + acp_pgfsm_stat_reg);
	if (val == ACP_POWERED_ON)
		return 0;

	if ((val & ACP_PGFSM_STATUS_MASK) != ACP_POWER_ON_IN_PROGRESS)
		pete_writel("sound/soc/amd/acp/acp-legacy-common.c:272", ACP_PGFSM_CNTL_POWER_ON_MASK, base + acp_pgfsm_ctrl_reg);

	return readl_poll_timeout(base + acp_pgfsm_stat_reg, val,
				  !val, DELAY_US, ACP_TIMEOUT);
}

static int acp_reset(void __iomem *base)
{
	u32 val;
	int ret;

	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:283", 1, base + ACP_SOFT_RESET);
	ret = readl_poll_timeout(base + ACP_SOFT_RESET, val, val & ACP_SOFT_RST_DONE_MASK,
				 DELAY_US, ACP_TIMEOUT);
	if (ret)
		return ret;

	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:289", 0, base + ACP_SOFT_RESET);
	return readl_poll_timeout(base + ACP_SOFT_RESET, val, !val, DELAY_US, ACP_TIMEOUT);
}

int acp_init(struct acp_chip_info *chip)
{
	int ret;

	/* power on */
	ret = acp_power_on(chip);
	if (ret) {
		pr_err("ACP power on failed\n");
		return ret;
	}
	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:303", 0x01, chip->base + ACP_CONTROL);

	/* Reset */
	ret = acp_reset(chip->base);
	if (ret) {
		pr_err("ACP reset failed\n");
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_NS_GPL(acp_init, SND_SOC_ACP_COMMON);

int acp_deinit(void __iomem *base)
{
	int ret;

	/* Reset */
	ret = acp_reset(base);
	if (ret)
		return ret;

	pete_writel("sound/soc/amd/acp/acp-legacy-common.c:324", 0, base + ACP_CONTROL);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(acp_deinit, SND_SOC_ACP_COMMON);

int smn_write(struct pci_dev *dev, u32 smn_addr, u32 data)
{
	pci_write_config_dword(dev, 0x60, smn_addr);
	pci_write_config_dword(dev, 0x64, data);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(smn_write, SND_SOC_ACP_COMMON);

int smn_read(struct pci_dev *dev, u32 smn_addr)
{
	u32 data;

	pci_write_config_dword(dev, 0x60, smn_addr);
	pci_read_config_dword(dev, 0x64, &data);
	return data;
}
EXPORT_SYMBOL_NS_GPL(smn_read, SND_SOC_ACP_COMMON);

MODULE_LICENSE("Dual BSD/GPL");
