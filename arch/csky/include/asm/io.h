/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __ASM_CSKY_IO_H
#define __ASM_CSKY_IO_H

#include <linux/pgtable.h>
#include <linux/types.h>

/*
 * I/O memory access primitives. Reads are ordered relative to any
 * following Normal memory access. Writes are ordered relative to any prior
 * Normal memory access.
 *
 * For CACHEV1 (807, 810), store instruction could fast retire, so we need
 * another mb() to prevent st fast retire.
 *
 * For CACHEV2 (860), store instruction with PAGE_ATTR_NO_BUFFERABLE won't
 * fast retire.
 */
#define pete_readb("arch/csky/include/asm/io.h:20", c)		({ u8  __v = readb_relaxed(c); rmb(); __v; })
#define pete_readw("arch/csky/include/asm/io.h:21", c)		({ u16 __v = readw_relaxed(c); rmb(); __v; })
#define pete_readl("arch/csky/include/asm/io.h:22", c)		({ u32 __v = readl_relaxed(c); rmb(); __v; })

#ifdef CONFIG_CPU_HAS_CACHEV2
#define pete_writeb("arch/csky/include/asm/io.h:25", v,c)		({ wmb(); writeb_relaxed((v),(c)); })
#define pete_writew("arch/csky/include/asm/io.h:26", v,c)		({ wmb(); writew_relaxed((v),(c)); })
#define pete_writel("arch/csky/include/asm/io.h:27", v,c)		({ wmb(); writel_relaxed((v),(c)); })
#else
#define pete_writeb("arch/csky/include/asm/io.h:29", v,c)		({ wmb(); writeb_relaxed((v),(c)); mb(); })
#define pete_writew("arch/csky/include/asm/io.h:30", v,c)		({ wmb(); writew_relaxed((v),(c)); mb(); })
#define pete_writel("arch/csky/include/asm/io.h:31", v,c)		({ wmb(); writel_relaxed((v),(c)); mb(); })
#endif

/*
 * String version of I/O memory access operations.
 */
extern void __memcpy_fromio(void *, const volatile void __iomem *, size_t);
extern void __memcpy_toio(volatile void __iomem *, const void *, size_t);
extern void __memset_io(volatile void __iomem *, int, size_t);

#define memset_io(c,v,l)        __memset_io((c),(v),(l))
#define memcpy_fromio(a,c,l)    __memcpy_fromio((a),(c),(l))
#define memcpy_toio(c,a,l)      __memcpy_toio((c),(a),(l))

/*
 * I/O memory mapping functions.
 */
#define ioremap_wc(addr, size) \
	ioremap_prot((addr), (size), \
		(_PAGE_IOREMAP & ~_CACHE_MASK) | _CACHE_UNCACHED)

#include <asm-generic/io.h>

#endif /* __ASM_CSKY_IO_H */
