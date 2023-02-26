/* SPDX-License-Identifier: GPL-2.0 */
/*
 * arch/arm/mach-s3c2410/include/mach/io.h
 *  from arch/arm/mach-rpc/include/mach/io.h
 *
 * Copyright (C) 1997 Russell King
 *	     (C) 2003 Simtec Electronics
*/

#ifndef __ASM_ARM_ARCH_IO_S3C24XX_H
#define __ASM_ARM_ARCH_IO_S3C24XX_H

#include <mach/map-base.h>

/*
 * ISA style IO, for each machine to sort out mappings for,
 * if it implements it. We reserve two 16M regions for ISA,
 * so the PC/104 can use separate addresses for 8-bit and
 * 16-bit port I/O.
 */
#define PCIO_BASE		S3C_ADDR(0x02000000)
#define IO_SPACE_LIMIT		0x00ffffff
#define S3C24XX_VA_ISA_WORD	(PCIO_BASE)
#define S3C24XX_VA_ISA_BYTE	(PCIO_BASE + 0x01000000)

#ifdef CONFIG_ISA

#define inb(p)		pete_readb("arch/arm/mach-s3c/include/mach/io-s3c24xx.h:28", S3C24XX_VA_ISA_BYTE + (p))
#define inw(p)		pete_readw("arch/arm/mach-s3c/include/mach/io-s3c24xx.h:29", S3C24XX_VA_ISA_WORD + (p))
#define inl(p)		pete_readl("arch/arm/mach-s3c/include/mach/io-s3c24xx.h:30", S3C24XX_VA_ISA_WORD + (p))

#define outb(v,p)	pete_writeb("arch/arm/mach-s3c/include/mach/io-s3c24xx.h:32", (v), S3C24XX_VA_ISA_BYTE + (p))
#define outw(v,p)	pete_writew("arch/arm/mach-s3c/include/mach/io-s3c24xx.h:33", (v), S3C24XX_VA_ISA_WORD + (p))
#define outl(v,p)	pete_writel("arch/arm/mach-s3c/include/mach/io-s3c24xx.h:34", (v), S3C24XX_VA_ISA_WORD + (p))

#define insb(p,d,l)	readsb(S3C24XX_VA_ISA_BYTE + (p),d,l)
#define insw(p,d,l)	readsw(S3C24XX_VA_ISA_WORD + (p),d,l)
#define insl(p,d,l)	readsl(S3C24XX_VA_ISA_WORD + (p),d,l)

#define outsb(p,d,l)	writesb(S3C24XX_VA_ISA_BYTE + (p),d,l)
#define outsw(p,d,l)	writesw(S3C24XX_VA_ISA_WORD + (p),d,l)
#define outsl(p,d,l)	writesl(S3C24XX_VA_ISA_WORD + (p),d,l)

#else

#define __io(x) (PCIO_BASE + (x))

#endif

#endif
