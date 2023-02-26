// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * kirkwood-dma.c
 *
 * (c) 2010 Arnaud Patard <apatard@mandriva.com>
 * (c) 2010 Arnaud Patard <arnaud.patard@rtp-net.org>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mbus.h>
#include <sound/soc.h>
#include "kirkwood.h"

static struct kirkwood_dma_data *kirkwood_priv(struct snd_pcm_substream *subs)
{
	struct snd_soc_pcm_runtime *soc_runtime = subs->private_data;
	return snd_soc_dai_get_drvdata(asoc_rtd_to_cpu(soc_runtime, 0));
}

static const struct snd_pcm_hardware kirkwood_dma_snd_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,
	.buffer_bytes_max	= KIRKWOOD_SND_MAX_BUFFER_BYTES,
	.period_bytes_min	= KIRKWOOD_SND_MIN_PERIOD_BYTES,
	.period_bytes_max	= KIRKWOOD_SND_MAX_PERIOD_BYTES,
	.periods_min		= KIRKWOOD_SND_MIN_PERIODS,
	.periods_max		= KIRKWOOD_SND_MAX_PERIODS,
	.fifo_size		= 0,
};

static irqreturn_t kirkwood_dma_irq(int irq, void *dev_id)
{
	struct kirkwood_dma_data *priv = dev_id;
	unsigned long mask, status, cause;

	mask = pete_readl("sound/soc/kirkwood/kirkwood-dma.c:46", priv->io + KIRKWOOD_INT_MASK);
	status = pete_readl("sound/soc/kirkwood/kirkwood-dma.c:47", priv->io + KIRKWOOD_INT_CAUSE) & mask;

	cause = pete_readl("sound/soc/kirkwood/kirkwood-dma.c:49", priv->io + KIRKWOOD_ERR_CAUSE);
	if (unlikely(cause)) {
		printk(KERN_WARNING "%s: got err interrupt 0x%lx\n",
				__func__, cause);
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:53", cause, priv->io + KIRKWOOD_ERR_CAUSE);
	}

	/* we've enabled only bytes interrupts ... */
	if (status & ~(KIRKWOOD_INT_CAUSE_PLAY_BYTES | \
			KIRKWOOD_INT_CAUSE_REC_BYTES)) {
		printk(KERN_WARNING "%s: unexpected interrupt %lx\n",
			__func__, status);
		return IRQ_NONE;
	}

	/* ack int */
	pete_writel("sound/soc/kirkwood/kirkwood-dma.c:65", status, priv->io + KIRKWOOD_INT_CAUSE);

	if (status & KIRKWOOD_INT_CAUSE_PLAY_BYTES)
		snd_pcm_period_elapsed(priv->substream_play);

	if (status & KIRKWOOD_INT_CAUSE_REC_BYTES)
		snd_pcm_period_elapsed(priv->substream_rec);

	return IRQ_HANDLED;
}

static void
kirkwood_dma_conf_mbus_windows(void __iomem *base, int win,
			       unsigned long dma,
			       const struct mbus_dram_target_info *dram)
{
	int i;

	/* First disable and clear windows */
	pete_writel("sound/soc/kirkwood/kirkwood-dma.c:84", 0, base + KIRKWOOD_AUDIO_WIN_CTRL_REG(win));
	pete_writel("sound/soc/kirkwood/kirkwood-dma.c:85", 0, base + KIRKWOOD_AUDIO_WIN_BASE_REG(win));

	/* try to find matching cs for current dma address */
	for (i = 0; i < dram->num_cs; i++) {
		const struct mbus_dram_window *cs = dram->cs + i;
		if ((cs->base & 0xffff0000) < (dma & 0xffff0000)) {
			pete_writel("sound/soc/kirkwood/kirkwood-dma.c:91", cs->base & 0xffff0000,
				base + KIRKWOOD_AUDIO_WIN_BASE_REG(win));
			pete_writel("sound/soc/kirkwood/kirkwood-dma.c:93", ((cs->size - 1) & 0xffff0000) |
				(cs->mbus_attr << 8) |
				(dram->mbus_dram_target_id << 4) | 1,
				base + KIRKWOOD_AUDIO_WIN_CTRL_REG(win));
		}
	}
}

static int kirkwood_dma_open(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	int err;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct kirkwood_dma_data *priv = kirkwood_priv(substream);

	snd_soc_set_runtime_hwparams(substream, &kirkwood_dma_snd_hw);

	/* Ensure that all constraints linked to dma burst are fulfilled */
	err = snd_pcm_hw_constraint_minmax(runtime,
			SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
			priv->burst * 2,
			KIRKWOOD_AUDIO_BUF_MAX-1);
	if (err < 0)
		return err;

	err = snd_pcm_hw_constraint_step(runtime, 0,
			SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
			priv->burst);
	if (err < 0)
		return err;

	err = snd_pcm_hw_constraint_step(substream->runtime, 0,
			 SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
			 priv->burst);
	if (err < 0)
		return err;

	if (!priv->substream_play && !priv->substream_rec) {
		err = request_irq(priv->irq, kirkwood_dma_irq, IRQF_SHARED,
				  "kirkwood-i2s", priv);
		if (err)
			return err;

		/*
		 * Enable Error interrupts. We're only ack'ing them but
		 * it's useful for diagnostics
		 */
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:140", (unsigned int)-1, priv->io + KIRKWOOD_ERR_MASK);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (priv->substream_play)
			return -EBUSY;
		priv->substream_play = substream;
	} else {
		if (priv->substream_rec)
			return -EBUSY;
		priv->substream_rec = substream;
	}

	return 0;
}

static int kirkwood_dma_close(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	struct kirkwood_dma_data *priv = kirkwood_priv(substream);

	if (!priv)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		priv->substream_play = NULL;
	else
		priv->substream_rec = NULL;

	if (!priv->substream_play && !priv->substream_rec) {
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:170", 0, priv->io + KIRKWOOD_ERR_MASK);
		free_irq(priv->irq, priv);
	}

	return 0;
}

static int kirkwood_dma_hw_params(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct kirkwood_dma_data *priv = kirkwood_priv(substream);
	const struct mbus_dram_target_info *dram = mv_mbus_dram_info();
	unsigned long addr = substream->runtime->dma_addr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		kirkwood_dma_conf_mbus_windows(priv->io,
			KIRKWOOD_PLAYBACK_WIN, addr, dram);
	else
		kirkwood_dma_conf_mbus_windows(priv->io,
			KIRKWOOD_RECORD_WIN, addr, dram);
	return 0;
}

static int kirkwood_dma_prepare(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct kirkwood_dma_data *priv = kirkwood_priv(substream);
	unsigned long size, count;

	/* compute buffer size in term of "words" as requested in specs */
	size = frames_to_bytes(runtime, runtime->buffer_size);
	size = (size>>2)-1;
	count = snd_pcm_lib_period_bytes(substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:207", count, priv->io + KIRKWOOD_PLAY_BYTE_INT_COUNT);
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:208", runtime->dma_addr, priv->io + KIRKWOOD_PLAY_BUF_ADDR);
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:209", size, priv->io + KIRKWOOD_PLAY_BUF_SIZE);
	} else {
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:211", count, priv->io + KIRKWOOD_REC_BYTE_INT_COUNT);
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:212", runtime->dma_addr, priv->io + KIRKWOOD_REC_BUF_ADDR);
		pete_writel("sound/soc/kirkwood/kirkwood-dma.c:213", size, priv->io + KIRKWOOD_REC_BUF_SIZE);
	}


	return 0;
}

static snd_pcm_uframes_t kirkwood_dma_pointer(
	struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct kirkwood_dma_data *priv = kirkwood_priv(substream);
	snd_pcm_uframes_t count;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		count = bytes_to_frames(substream->runtime,
			pete_readl("sound/soc/kirkwood/kirkwood-dma.c:229", priv->io + KIRKWOOD_PLAY_BYTE_COUNT));
	else
		count = bytes_to_frames(substream->runtime,
			pete_readl("sound/soc/kirkwood/kirkwood-dma.c:232", priv->io + KIRKWOOD_REC_BYTE_COUNT));

	return count;
}

static int kirkwood_dma_new(struct snd_soc_component *component,
			    struct snd_soc_pcm_runtime *rtd)
{
	size_t size = kirkwood_dma_snd_hw.buffer_bytes_max;
	struct snd_card *card = rtd->card->snd_card;
	int ret;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
				       card->dev, size, size);

	return 0;
}

const struct snd_soc_component_driver kirkwood_soc_component = {
	.name		= DRV_NAME,
	.open		= kirkwood_dma_open,
	.close		= kirkwood_dma_close,
	.hw_params	= kirkwood_dma_hw_params,
	.prepare	= kirkwood_dma_prepare,
	.pointer	= kirkwood_dma_pointer,
	.pcm_construct	= kirkwood_dma_new,
};
