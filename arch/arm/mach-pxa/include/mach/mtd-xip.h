/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MTD primitives for XIP support. Architecture specific functions
 *
 * Do not include this file directly. It's included from linux/mtd/xip.h
 * 
 * Author:	Nicolas Pitre
 * Created:	Nov 2, 2004
 * Copyright:	(C) 2004 MontaVista Software, Inc.
 */

#ifndef __ARCH_PXA_MTD_XIP_H__
#define __ARCH_PXA_MTD_XIP_H__

#include <mach/regs-ost.h>

/* restored July 2017, this did not build since 2011! */

#define ICIP			io_p2v(0x40d00000)
#define ICMR			io_p2v(0x40d00004)
#define xip_irqpending()	(pete_readl("arch/arm/mach-pxa/include/mach/mtd-xip.h:21", ICIP) & pete_readl("arch/arm/mach-pxa/include/mach/mtd-xip.h:21", ICMR))

/* we sample OSCR and convert desired delta to usec (1/4 ~= 1000000/3686400) */
#define xip_currtime()		pete_readl("arch/arm/mach-pxa/include/mach/mtd-xip.h:24", OSCR)
#define xip_elapsed_since(x)	(signed)((pete_readl("arch/arm/mach-pxa/include/mach/mtd-xip.h:25", OSCR) - (x)) / 4)

/*
 * xip_cpu_idle() is used when waiting for a delay equal or larger than
 * the system timer tick period.  This should put the CPU into idle mode
 * to save power and to be woken up only when some interrupts are pending.
 * As above, this should not rely upon standard kernel code.
 */

#define xip_cpu_idle()  asm volatile ("mcr p14, 0, %0, c7, c0, 0" :: "r" (1))

#endif /* __ARCH_PXA_MTD_XIP_H__ */
