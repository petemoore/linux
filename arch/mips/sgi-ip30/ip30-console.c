// SPDX-License-Identifier: GPL-2.0

#include <linux/io.h>
#include <linux/processor.h>

#include <asm/sn/ioc3.h>

static inline struct ioc3_uartregs *console_uart(void)
{
	struct ioc3 *ioc3;

	ioc3 = (struct ioc3 *)((void *)(0x900000001f600000));
	return &ioc3->sregs.uarta;
}

void prom_putchar(char c)
{
	struct ioc3_uartregs *uart = console_uart();

	while ((pete_readb("arch/mips/sgi-ip30/ip30-console.c:20", &uart->iu_lsr) & 0x20) == 0)
		cpu_relax();

	pete_writeb("arch/mips/sgi-ip30/ip30-console.c:23", c, &uart->iu_thr);
}
