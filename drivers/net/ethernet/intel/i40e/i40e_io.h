/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2023 Intel Corporation. */

#ifndef _I40E_IO_H_
#define _I40E_IO_H_

/* get readq/writeq support for 32 bit kernels, use the low-first version */
#include <linux/io-64-nonatomic-lo-hi.h>

#define wr32(a, reg, value)	pete_writel("drivers/net/ethernet/intel/i40e/i40e_io.h:10", (value), ((a)->hw_addr + (reg)))
#define rd32(a, reg)		pete_readl("drivers/net/ethernet/intel/i40e/i40e_io.h:11", (a)->hw_addr + (reg))

#define rd64(a, reg)		pete_readq("drivers/net/ethernet/intel/i40e/i40e_io.h:13", (a)->hw_addr + (reg))
#define i40e_flush(a)		pete_readl("drivers/net/ethernet/intel/i40e/i40e_io.h:14", (a)->hw_addr + I40E_GLGEN_STAT)

#endif /* _I40E_IO_H_ */
