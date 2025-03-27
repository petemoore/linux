// SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause

/* Packet receive logic for Mellanox Gigabit Ethernet driver
 *
 * Copyright (C) 2020-2021 NVIDIA CORPORATION & AFFILIATES
 */

#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "mlxbf_gige.h"
#include "mlxbf_gige_regs.h"

void mlxbf_gige_enable_multicast_rx(struct mlxbf_gige *priv)
{
	void __iomem *base = priv->base;
	u64 data;

	data = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:19", base + MLXBF_GIGE_RX_MAC_FILTER_GENERAL);
	data |= MLXBF_GIGE_RX_MAC_FILTER_EN_MULTICAST;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:21", data, base + MLXBF_GIGE_RX_MAC_FILTER_GENERAL);
}

void mlxbf_gige_disable_multicast_rx(struct mlxbf_gige *priv)
{
	void __iomem *base = priv->base;
	u64 data;

	data = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:29", base + MLXBF_GIGE_RX_MAC_FILTER_GENERAL);
	data &= ~MLXBF_GIGE_RX_MAC_FILTER_EN_MULTICAST;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:31", data, base + MLXBF_GIGE_RX_MAC_FILTER_GENERAL);
}

void mlxbf_gige_enable_mac_rx_filter(struct mlxbf_gige *priv,
				     unsigned int index)
{
	void __iomem *base = priv->base;
	u64 control;

	/* Enable MAC receive filter mask for specified index */
	control = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:41", base + MLXBF_GIGE_CONTROL);
	control |= (MLXBF_GIGE_CONTROL_EN_SPECIFIC_MAC << index);
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:43", control, base + MLXBF_GIGE_CONTROL);
}

void mlxbf_gige_disable_mac_rx_filter(struct mlxbf_gige *priv,
				      unsigned int index)
{
	void __iomem *base = priv->base;
	u64 control;

	/* Disable MAC receive filter mask for specified index */
	control = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:53", base + MLXBF_GIGE_CONTROL);
	control &= ~(MLXBF_GIGE_CONTROL_EN_SPECIFIC_MAC << index);
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:55", control, base + MLXBF_GIGE_CONTROL);
}

void mlxbf_gige_set_mac_rx_filter(struct mlxbf_gige *priv,
				  unsigned int index, u64 dmac)
{
	void __iomem *base = priv->base;

	/* Write destination MAC to specified MAC RX filter */
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:64", dmac, base + MLXBF_GIGE_RX_MAC_FILTER +
	       (index * MLXBF_GIGE_RX_MAC_FILTER_STRIDE));
}

void mlxbf_gige_get_mac_rx_filter(struct mlxbf_gige *priv,
				  unsigned int index, u64 *dmac)
{
	void __iomem *base = priv->base;

	/* Read destination MAC from specified MAC RX filter */
	*dmac = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:74", base + MLXBF_GIGE_RX_MAC_FILTER +
		      (index * MLXBF_GIGE_RX_MAC_FILTER_STRIDE));
}

void mlxbf_gige_enable_promisc(struct mlxbf_gige *priv)
{
	void __iomem *base = priv->base;
	u64 control;
	u64 end_mac;

	/* Enable MAC_ID_RANGE match functionality */
	control = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:85", base + MLXBF_GIGE_CONTROL);
	control |= MLXBF_GIGE_CONTROL_MAC_ID_RANGE_EN;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:87", control, base + MLXBF_GIGE_CONTROL);

	/* Set start of destination MAC range check to 0 */
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:90", 0, base + MLXBF_GIGE_RX_MAC_FILTER_DMAC_RANGE_START);

	/* Set end of destination MAC range check to all FFs */
	end_mac = BCAST_MAC_ADDR;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:94", end_mac, base + MLXBF_GIGE_RX_MAC_FILTER_DMAC_RANGE_END);
}

void mlxbf_gige_disable_promisc(struct mlxbf_gige *priv)
{
	void __iomem *base = priv->base;
	u64 control;

	/* Disable MAC_ID_RANGE match functionality */
	control = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:103", base + MLXBF_GIGE_CONTROL);
	control &= ~MLXBF_GIGE_CONTROL_MAC_ID_RANGE_EN;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:105", control, base + MLXBF_GIGE_CONTROL);

	/* NOTE: no need to change DMAC_RANGE_START or END;
	 * those values are ignored since MAC_ID_RANGE_EN=0
	 */
}

/* Receive Initialization
 * 1) Configures RX MAC filters via MMIO registers
 * 2) Allocates RX WQE array using coherent DMA mapping
 * 3) Initializes each element of RX WQE array with a receive
 *    buffer pointer (also using coherent DMA mapping)
 * 4) Allocates RX CQE array using coherent DMA mapping
 * 5) Completes other misc receive initialization
 */
int mlxbf_gige_rx_init(struct mlxbf_gige *priv)
{
	size_t wq_size, cq_size;
	dma_addr_t *rx_wqe_ptr;
	dma_addr_t rx_buf_dma;
	u64 data;
	int i, j;

	/* Configure MAC RX filter #0 to allow RX of broadcast pkts */
	mlxbf_gige_set_mac_rx_filter(priv, MLXBF_GIGE_BCAST_MAC_FILTER_IDX,
				     BCAST_MAC_ADDR);

	wq_size = MLXBF_GIGE_RX_WQE_SZ * priv->rx_q_entries;
	priv->rx_wqe_base = dma_alloc_coherent(priv->dev, wq_size,
					       &priv->rx_wqe_base_dma,
					       GFP_KERNEL);
	if (!priv->rx_wqe_base)
		return -ENOMEM;

	/* Initialize 'rx_wqe_ptr' to point to first RX WQE in array
	 * Each RX WQE is simply a receive buffer pointer, so walk
	 * the entire array, allocating a 2KB buffer for each element
	 */
	rx_wqe_ptr = priv->rx_wqe_base;

	for (i = 0; i < priv->rx_q_entries; i++) {
		priv->rx_skb[i] = mlxbf_gige_alloc_skb(priv, MLXBF_GIGE_DEFAULT_BUF_SZ,
						       &rx_buf_dma, DMA_FROM_DEVICE);
		if (!priv->rx_skb[i])
			goto free_wqe_and_skb;
		*rx_wqe_ptr++ = rx_buf_dma;
	}

	/* Write RX WQE base address into MMIO reg */
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:154", priv->rx_wqe_base_dma, priv->base + MLXBF_GIGE_RX_WQ_BASE);

	cq_size = MLXBF_GIGE_RX_CQE_SZ * priv->rx_q_entries;
	priv->rx_cqe_base = dma_alloc_coherent(priv->dev, cq_size,
					       &priv->rx_cqe_base_dma,
					       GFP_KERNEL);
	if (!priv->rx_cqe_base)
		goto free_wqe_and_skb;

	for (i = 0; i < priv->rx_q_entries; i++)
		priv->rx_cqe_base[i] |= MLXBF_GIGE_RX_CQE_VALID_MASK;

	/* Write RX CQE base address into MMIO reg */
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:167", priv->rx_cqe_base_dma, priv->base + MLXBF_GIGE_RX_CQ_BASE);

	/* Write RX_WQE_PI with current number of replenished buffers */
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:170", priv->rx_q_entries, priv->base + MLXBF_GIGE_RX_WQE_PI);

	/* Enable removal of CRC during RX */
	data = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:173", priv->base + MLXBF_GIGE_RX);
	data |= MLXBF_GIGE_RX_STRIP_CRC_EN;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:175", data, priv->base + MLXBF_GIGE_RX);

	/* Enable RX MAC filter pass and discard counters */
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:178", MLXBF_GIGE_RX_MAC_FILTER_COUNT_DISC_EN,
	       priv->base + MLXBF_GIGE_RX_MAC_FILTER_COUNT_DISC);
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:180", MLXBF_GIGE_RX_MAC_FILTER_COUNT_PASS_EN,
	       priv->base + MLXBF_GIGE_RX_MAC_FILTER_COUNT_PASS);

	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:183", ilog2(priv->rx_q_entries),
	       priv->base + MLXBF_GIGE_RX_WQE_SIZE_LOG2);

	/* Clear MLXBF_GIGE_INT_MASK 'receive pkt' bit to
	 * indicate readiness to receive interrupts
	 */
	data = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:189", priv->base + MLXBF_GIGE_INT_MASK);
	data &= ~MLXBF_GIGE_INT_MASK_RX_RECEIVE_PACKET;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:191", data, priv->base + MLXBF_GIGE_INT_MASK);

	/* Enable RX DMA to write new packets to memory */
	data = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:194", priv->base + MLXBF_GIGE_RX_DMA);
	data |= MLXBF_GIGE_RX_DMA_EN;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:196", data, priv->base + MLXBF_GIGE_RX_DMA);

	return 0;

free_wqe_and_skb:
	rx_wqe_ptr = priv->rx_wqe_base;
	for (j = 0; j < i; j++) {
		dma_unmap_single(priv->dev, *rx_wqe_ptr,
				 MLXBF_GIGE_DEFAULT_BUF_SZ, DMA_FROM_DEVICE);
		dev_kfree_skb(priv->rx_skb[j]);
		rx_wqe_ptr++;
	}
	dma_free_coherent(priv->dev, wq_size,
			  priv->rx_wqe_base, priv->rx_wqe_base_dma);
	return -ENOMEM;
}

/* Receive Deinitialization
 * This routine will free allocations done by mlxbf_gige_rx_init(),
 * namely the RX WQE and RX CQE arrays, as well as all RX buffers
 */
void mlxbf_gige_rx_deinit(struct mlxbf_gige *priv)
{
	dma_addr_t *rx_wqe_ptr;
	size_t size;
	u64 data;
	int i;

	/* Disable RX DMA to prevent packet transfers to memory */
	data = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:225", priv->base + MLXBF_GIGE_RX_DMA);
	data &= ~MLXBF_GIGE_RX_DMA_EN;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:227", data, priv->base + MLXBF_GIGE_RX_DMA);

	rx_wqe_ptr = priv->rx_wqe_base;

	for (i = 0; i < priv->rx_q_entries; i++) {
		dma_unmap_single(priv->dev, *rx_wqe_ptr, MLXBF_GIGE_DEFAULT_BUF_SZ,
				 DMA_FROM_DEVICE);
		dev_kfree_skb(priv->rx_skb[i]);
		rx_wqe_ptr++;
	}

	size = MLXBF_GIGE_RX_WQE_SZ * priv->rx_q_entries;
	dma_free_coherent(priv->dev, size,
			  priv->rx_wqe_base, priv->rx_wqe_base_dma);

	size = MLXBF_GIGE_RX_CQE_SZ * priv->rx_q_entries;
	dma_free_coherent(priv->dev, size,
			  priv->rx_cqe_base, priv->rx_cqe_base_dma);

	priv->rx_wqe_base = NULL;
	priv->rx_wqe_base_dma = 0;
	priv->rx_cqe_base = NULL;
	priv->rx_cqe_base_dma = 0;
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:250", 0, priv->base + MLXBF_GIGE_RX_WQ_BASE);
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:251", 0, priv->base + MLXBF_GIGE_RX_CQ_BASE);
}

static bool mlxbf_gige_rx_packet(struct mlxbf_gige *priv, int *rx_pkts)
{
	struct net_device *netdev = priv->netdev;
	struct sk_buff *skb = NULL, *rx_skb;
	u16 rx_pi_rem, rx_ci_rem;
	dma_addr_t *rx_wqe_addr;
	dma_addr_t rx_buf_dma;
	u64 *rx_cqe_addr;
	u64 datalen;
	u64 rx_cqe;
	u16 rx_ci;
	u16 rx_pi;

	/* Index into RX buffer array is rx_pi w/wrap based on RX_CQE_SIZE */
	rx_pi = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:268", priv->base + MLXBF_GIGE_RX_WQE_PI);
	rx_pi_rem = rx_pi % priv->rx_q_entries;

	rx_wqe_addr = priv->rx_wqe_base + rx_pi_rem;
	rx_cqe_addr = priv->rx_cqe_base + rx_pi_rem;
	rx_cqe = *rx_cqe_addr;

	if ((!!(rx_cqe & MLXBF_GIGE_RX_CQE_VALID_MASK)) != priv->valid_polarity)
		return false;

	if ((rx_cqe & MLXBF_GIGE_RX_CQE_PKT_STATUS_MASK) == 0) {
		/* Packet is OK, increment stats */
		datalen = rx_cqe & MLXBF_GIGE_RX_CQE_PKT_LEN_MASK;
		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += datalen;

		skb = priv->rx_skb[rx_pi_rem];

		/* Alloc another RX SKB for this same index */
		rx_skb = mlxbf_gige_alloc_skb(priv, MLXBF_GIGE_DEFAULT_BUF_SZ,
					      &rx_buf_dma, DMA_FROM_DEVICE);
		if (!rx_skb)
			return false;
		priv->rx_skb[rx_pi_rem] = rx_skb;
		dma_unmap_single(priv->dev, *rx_wqe_addr,
				 MLXBF_GIGE_DEFAULT_BUF_SZ, DMA_FROM_DEVICE);

		skb_put(skb, datalen);

		skb->ip_summed = CHECKSUM_NONE; /* device did not checksum packet */

		skb->protocol = eth_type_trans(skb, netdev);

		*rx_wqe_addr = rx_buf_dma;
	} else if (rx_cqe & MLXBF_GIGE_RX_CQE_PKT_STATUS_MAC_ERR) {
		priv->stats.rx_mac_errors++;
	} else if (rx_cqe & MLXBF_GIGE_RX_CQE_PKT_STATUS_TRUNCATED) {
		priv->stats.rx_truncate_errors++;
	}

	/* Read receive consumer index before replenish so that this routine
	 * returns accurate return value even if packet is received into
	 * just-replenished buffer prior to exiting this routine.
	 */
	rx_ci = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:312", priv->base + MLXBF_GIGE_RX_CQE_PACKET_CI);
	rx_ci_rem = rx_ci % priv->rx_q_entries;

	/* Let hardware know we've replenished one buffer */
	rx_pi++;

	/* Ensure completion of all writes before notifying HW of replenish */
	wmb();
	pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:320", rx_pi, priv->base + MLXBF_GIGE_RX_WQE_PI);

	(*rx_pkts)++;

	rx_pi_rem = rx_pi % priv->rx_q_entries;
	if (rx_pi_rem == 0)
		priv->valid_polarity ^= 1;

	if (skb)
		netif_receive_skb(skb);

	return rx_pi_rem != rx_ci_rem;
}

/* Driver poll() function called by NAPI infrastructure */
int mlxbf_gige_poll(struct napi_struct *napi, int budget)
{
	struct mlxbf_gige *priv;
	bool remaining_pkts;
	int work_done = 0;
	u64 data;

	priv = container_of(napi, struct mlxbf_gige, napi);

	mlxbf_gige_handle_tx_complete(priv);

	do {
		remaining_pkts = mlxbf_gige_rx_packet(priv, &work_done);
	} while (remaining_pkts && work_done < budget);

	/* If amount of work done < budget, turn off NAPI polling
	 * via napi_complete_done(napi, work_done) and then
	 * re-enable interrupts.
	 */
	if (work_done < budget && napi_complete_done(napi, work_done)) {
		/* Clear MLXBF_GIGE_INT_MASK 'receive pkt' bit to
		 * indicate receive readiness
		 */
		data = pete_readq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:358", priv->base + MLXBF_GIGE_INT_MASK);
		data &= ~MLXBF_GIGE_INT_MASK_RX_RECEIVE_PACKET;
		pete_writeq("drivers/net/ethernet/mellanox/mlxbf_gige/mlxbf_gige_rx.c:360", data, priv->base + MLXBF_GIGE_INT_MASK);
	}

	return work_done;
}
