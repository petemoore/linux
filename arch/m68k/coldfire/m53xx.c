// SPDX-License-Identifier: GPL-2.0-or-later
/***************************************************************************/

/*
 *	m53xx.c -- platform support for ColdFire 53xx based boards
 *
 *	Copyright (C) 1999-2002, Greg Ungerer (gerg@snapgear.com)
 *	Copyright (C) 2000, Lineo (www.lineo.com)
 *	Yaroslav Vinogradov yaroslav.vinogradov@freescale.com
 *	Copyright Freescale Semiconductor, Inc 2006
 *	Copyright (c) 2006, emlix, Sebastian Hess <shess@hessware.de>
 */

/***************************************************************************/

#include <linux/clkdev.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/io.h>
#include <asm/machdep.h>
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include <asm/mcfuart.h>
#include <asm/mcfdma.h>
#include <asm/mcfwdebug.h>
#include <asm/mcfclk.h>

/***************************************************************************/

DEFINE_CLK(0, "flexbus", 2, MCF_CLK);
DEFINE_CLK(0, "mcfcan.0", 8, MCF_CLK);
DEFINE_CLK(0, "fec.0", 12, MCF_CLK);
DEFINE_CLK(0, "edma", 17, MCF_CLK);
DEFINE_CLK(0, "intc.0", 18, MCF_CLK);
DEFINE_CLK(0, "intc.1", 19, MCF_CLK);
DEFINE_CLK(0, "iack.0", 21, MCF_CLK);
DEFINE_CLK(0, "imx1-i2c.0", 22, MCF_CLK);
DEFINE_CLK(0, "mcfqspi.0", 23, MCF_CLK);
DEFINE_CLK(0, "mcfuart.0", 24, MCF_BUSCLK);
DEFINE_CLK(0, "mcfuart.1", 25, MCF_BUSCLK);
DEFINE_CLK(0, "mcfuart.2", 26, MCF_BUSCLK);
DEFINE_CLK(0, "mcftmr.0", 28, MCF_CLK);
DEFINE_CLK(0, "mcftmr.1", 29, MCF_CLK);
DEFINE_CLK(0, "mcftmr.2", 30, MCF_CLK);
DEFINE_CLK(0, "mcftmr.3", 31, MCF_CLK);

DEFINE_CLK(0, "mcfpit.0", 32, MCF_CLK);
DEFINE_CLK(0, "mcfpit.1", 33, MCF_CLK);
DEFINE_CLK(0, "mcfpit.2", 34, MCF_CLK);
DEFINE_CLK(0, "mcfpit.3", 35, MCF_CLK);
DEFINE_CLK(0, "mcfpwm.0", 36, MCF_CLK);
DEFINE_CLK(0, "mcfeport.0", 37, MCF_CLK);
DEFINE_CLK(0, "mcfwdt.0", 38, MCF_CLK);
DEFINE_CLK(0, "sys.0", 40, MCF_BUSCLK);
DEFINE_CLK(0, "gpio.0", 41, MCF_BUSCLK);
DEFINE_CLK(0, "mcfrtc.0", 42, MCF_CLK);
DEFINE_CLK(0, "mcflcd.0", 43, MCF_CLK);
DEFINE_CLK(0, "mcfusb-otg.0", 44, MCF_CLK);
DEFINE_CLK(0, "mcfusb-host.0", 45, MCF_CLK);
DEFINE_CLK(0, "sdram.0", 46, MCF_CLK);
DEFINE_CLK(0, "ssi.0", 47, MCF_CLK);
DEFINE_CLK(0, "pll.0", 48, MCF_CLK);

DEFINE_CLK(1, "mdha.0", 32, MCF_CLK);
DEFINE_CLK(1, "skha.0", 33, MCF_CLK);
DEFINE_CLK(1, "rng.0", 34, MCF_CLK);

static struct clk_lookup m53xx_clk_lookup[] = {
	CLKDEV_INIT("flexbus", NULL, &__clk_0_2),
	CLKDEV_INIT("mcfcan.0", NULL, &__clk_0_8),
	CLKDEV_INIT("fec.0", NULL, &__clk_0_12),
	CLKDEV_INIT("edma", NULL, &__clk_0_17),
	CLKDEV_INIT("intc.0", NULL, &__clk_0_18),
	CLKDEV_INIT("intc.1", NULL, &__clk_0_19),
	CLKDEV_INIT("iack.0", NULL, &__clk_0_21),
	CLKDEV_INIT("imx1-i2c.0", NULL, &__clk_0_22),
	CLKDEV_INIT("mcfqspi.0", NULL, &__clk_0_23),
	CLKDEV_INIT("mcfuart.0", NULL, &__clk_0_24),
	CLKDEV_INIT("mcfuart.1", NULL, &__clk_0_25),
	CLKDEV_INIT("mcfuart.2", NULL, &__clk_0_26),
	CLKDEV_INIT("mcftmr.0", NULL, &__clk_0_28),
	CLKDEV_INIT("mcftmr.1", NULL, &__clk_0_29),
	CLKDEV_INIT("mcftmr.2", NULL, &__clk_0_30),
	CLKDEV_INIT("mcftmr.3", NULL, &__clk_0_31),
	CLKDEV_INIT("mcfpit.0", NULL, &__clk_0_32),
	CLKDEV_INIT("mcfpit.1", NULL, &__clk_0_33),
	CLKDEV_INIT("mcfpit.2", NULL, &__clk_0_34),
	CLKDEV_INIT("mcfpit.3", NULL, &__clk_0_35),
	CLKDEV_INIT("mcfpwm.0", NULL, &__clk_0_36),
	CLKDEV_INIT("mcfeport.0", NULL, &__clk_0_37),
	CLKDEV_INIT("mcfwdt.0", NULL, &__clk_0_38),
	CLKDEV_INIT(NULL, "sys.0", &__clk_0_40),
	CLKDEV_INIT("gpio.0", NULL, &__clk_0_41),
	CLKDEV_INIT("mcfrtc.0", NULL, &__clk_0_42),
	CLKDEV_INIT("mcflcd.0", NULL, &__clk_0_43),
	CLKDEV_INIT("mcfusb-otg.0", NULL, &__clk_0_44),
	CLKDEV_INIT("mcfusb-host.0", NULL, &__clk_0_45),
	CLKDEV_INIT("sdram.0", NULL, &__clk_0_46),
	CLKDEV_INIT("ssi.0", NULL, &__clk_0_47),
	CLKDEV_INIT(NULL, "pll.0", &__clk_0_48),
	CLKDEV_INIT("mdha.0", NULL, &__clk_1_32),
	CLKDEV_INIT("skha.0", NULL, &__clk_1_33),
	CLKDEV_INIT("rng.0", NULL, &__clk_1_34),
};

static struct clk * const enable_clks[] __initconst = {
	&__clk_0_2,	/* flexbus */
	&__clk_0_18,	/* intc.0 */
	&__clk_0_19,	/* intc.1 */
	&__clk_0_21,	/* iack.0 */
	&__clk_0_24,	/* mcfuart.0 */
	&__clk_0_25,	/* mcfuart.1 */
	&__clk_0_26,	/* mcfuart.2 */
	&__clk_0_28,	/* mcftmr.0 */
	&__clk_0_29,	/* mcftmr.1 */
	&__clk_0_32,	/* mcfpit.0 */
	&__clk_0_33,	/* mcfpit.1 */
	&__clk_0_37,	/* mcfeport.0 */
	&__clk_0_40,	/* sys.0 */
	&__clk_0_41,	/* gpio.0 */
	&__clk_0_46,	/* sdram.0 */
	&__clk_0_48,	/* pll.0 */
};

static struct clk * const disable_clks[] __initconst = {
	&__clk_0_8,	/* mcfcan.0 */
	&__clk_0_12,	/* fec.0 */
	&__clk_0_17,	/* edma */
	&__clk_0_22,	/* imx1-i2c.0 */
	&__clk_0_23,	/* mcfqspi.0 */
	&__clk_0_30,	/* mcftmr.2 */
	&__clk_0_31,	/* mcftmr.3 */
	&__clk_0_34,	/* mcfpit.2 */
	&__clk_0_35,	/* mcfpit.3 */
	&__clk_0_36,	/* mcfpwm.0 */
	&__clk_0_38,	/* mcfwdt.0 */
	&__clk_0_42,	/* mcfrtc.0 */
	&__clk_0_43,	/* mcflcd.0 */
	&__clk_0_44,	/* mcfusb-otg.0 */
	&__clk_0_45,	/* mcfusb-host.0 */
	&__clk_0_47,	/* ssi.0 */
	&__clk_1_32,	/* mdha.0 */
	&__clk_1_33,	/* skha.0 */
	&__clk_1_34,	/* rng.0 */
};


static void __init m53xx_clk_init(void)
{
	unsigned i;

	/* make sure these clocks are enabled */
	for (i = 0; i < ARRAY_SIZE(enable_clks); ++i)
		__clk_init_enabled(enable_clks[i]);
	/* make sure these clocks are disabled */
	for (i = 0; i < ARRAY_SIZE(disable_clks); ++i)
		__clk_init_disabled(disable_clks[i]);

	clkdev_add_table(m53xx_clk_lookup, ARRAY_SIZE(m53xx_clk_lookup));
}

/***************************************************************************/

static void __init m53xx_qspi_init(void)
{
#if IS_ENABLED(CONFIG_SPI_COLDFIRE_QSPI)
	/* setup QSPS pins for QSPI with gpio CS control */
	pete_writew("arch/m68k/coldfire/m53xx.c:169", 0x01f0, MCFGPIO_PAR_QSPI);
#endif /* IS_ENABLED(CONFIG_SPI_COLDFIRE_QSPI) */
}

/***************************************************************************/

static void __init m53xx_i2c_init(void)
{
#if IS_ENABLED(CONFIG_I2C_IMX)
	/* setup Port AS Pin Assignment Register for I2C */
	/*  set PASPA0 to SCL and PASPA1 to SDA */
	u8 r = pete_readb("arch/m68k/coldfire/m53xx.c:180", MCFGPIO_PAR_FECI2C);
	r |= 0x0f;
	pete_writeb("arch/m68k/coldfire/m53xx.c:182", r, MCFGPIO_PAR_FECI2C);
#endif /* IS_ENABLED(CONFIG_I2C_IMX) */
}

/***************************************************************************/

static void __init m53xx_uarts_init(void)
{
	/* UART GPIO initialization */
	pete_writew("arch/m68k/coldfire/m53xx.c:191", pete_readw("arch/m68k/coldfire/m53xx.c:191", MCFGPIO_PAR_UART) | 0x0FFF, MCFGPIO_PAR_UART);
}

/***************************************************************************/

static void __init m53xx_fec_init(void)
{
	u8 v;

	/* Set multi-function pins to ethernet mode for fec0 */
	v = pete_readb("arch/m68k/coldfire/m53xx.c:201", MCFGPIO_PAR_FECI2C);
	v |= MCF_GPIO_PAR_FECI2C_PAR_MDC_EMDC |
		MCF_GPIO_PAR_FECI2C_PAR_MDIO_EMDIO;
	pete_writeb("arch/m68k/coldfire/m53xx.c:204", v, MCFGPIO_PAR_FECI2C);

	v = pete_readb("arch/m68k/coldfire/m53xx.c:206", MCFGPIO_PAR_FEC);
	v = MCF_GPIO_PAR_FEC_PAR_FEC_7W_FEC | MCF_GPIO_PAR_FEC_PAR_FEC_MII_FEC;
	pete_writeb("arch/m68k/coldfire/m53xx.c:208", v, MCFGPIO_PAR_FEC);
}

/***************************************************************************/

void __init config_BSP(char *commandp, int size)
{
#if !defined(CONFIG_BOOTPARAM)
	/* Copy command line from FLASH to local buffer... */
	memcpy(commandp, (char *) 0x4000, 4);
	if(strncmp(commandp, "kcl ", 4) == 0){
		memcpy(commandp, (char *) 0x4004, size);
		commandp[size-1] = 0;
	} else {
		memset(commandp, 0, size);
	}
#endif
	mach_sched_init = hw_timer_init;
	m53xx_clk_init();
	m53xx_uarts_init();
	m53xx_fec_init();
	m53xx_qspi_init();
	m53xx_i2c_init();

#ifdef CONFIG_BDM_DISABLE
	/*
	 * Disable the BDM clocking.  This also turns off most of the rest of
	 * the BDM device.  This is good for EMC reasons. This option is not
	 * incompatible with the memory protection option.
	 */
	wdebug(MCFDEBUG_CSR, MCFDEBUG_CSR_PSTCLK);
#endif
}

/***************************************************************************/
/* Board initialization */
/***************************************************************************/
/* 
 * PLL min/max specifications
 */
#define MAX_FVCO	500000	/* KHz */
#define MAX_FSYS	80000 	/* KHz */
#define MIN_FSYS	58333 	/* KHz */
#define FREF		16000   /* KHz */


#define MAX_MFD		135     /* Multiplier */
#define MIN_MFD		88      /* Multiplier */
#define BUSDIV		6       /* Divider */

/*
 * Low Power Divider specifications
 */
#define MIN_LPD		(1 << 0)    /* Divider (not encoded) */
#define MAX_LPD		(1 << 15)   /* Divider (not encoded) */
#define DEFAULT_LPD	(1 << 1)	/* Divider (not encoded) */

#define SYS_CLK_KHZ	80000
#define SYSTEM_PERIOD	12.5
/*
 *  SDRAM Timing Parameters
 */  
#define SDRAM_BL	8	/* # of beats in a burst */
#define SDRAM_TWR	2	/* in clocks */
#define SDRAM_CASL	2.5	/* CASL in clocks */
#define SDRAM_TRCD	2	/* in clocks */
#define SDRAM_TRP	2	/* in clocks */
#define SDRAM_TRFC	7	/* in clocks */
#define SDRAM_TREFI	7800	/* in ns */

#define EXT_SRAM_ADDRESS	(0xC0000000)
#define FLASH_ADDRESS		(0x00000000)
#define SDRAM_ADDRESS		(0x40000000)

#define NAND_FLASH_ADDRESS	(0xD0000000)

void wtm_init(void);
void scm_init(void);
void gpio_init(void);
void fbcs_init(void);
void sdramc_init(void);
int  clock_pll (int fsys, int flags);
int  clock_limp (int);
int  clock_exit_limp (void);
int  get_sys_clock (void);

asmlinkage void __init sysinit(void)
{
	clock_pll(0, 0);

	wtm_init();
	scm_init();
	gpio_init();
	fbcs_init();
	sdramc_init();
}

void wtm_init(void)
{
	/* Disable watchdog timer */
	pete_writew("arch/m68k/coldfire/m53xx.c:308", 0, MCF_WTM_WCR);
}

#define MCF_SCM_BCR_GBW		(0x00000100)
#define MCF_SCM_BCR_GBR		(0x00000200)

void scm_init(void)
{
	/* All masters are trusted */
	pete_writel("arch/m68k/coldfire/m53xx.c:317", 0x77777777, MCF_SCM_MPR);
    
	/* Allow supervisor/user, read/write, and trusted/untrusted
	   access to all slaves */
	pete_writel("arch/m68k/coldfire/m53xx.c:321", 0, MCF_SCM_PACRA);
	pete_writel("arch/m68k/coldfire/m53xx.c:322", 0, MCF_SCM_PACRB);
	pete_writel("arch/m68k/coldfire/m53xx.c:323", 0, MCF_SCM_PACRC);
	pete_writel("arch/m68k/coldfire/m53xx.c:324", 0, MCF_SCM_PACRD);
	pete_writel("arch/m68k/coldfire/m53xx.c:325", 0, MCF_SCM_PACRE);
	pete_writel("arch/m68k/coldfire/m53xx.c:326", 0, MCF_SCM_PACRF);

	/* Enable bursts */
	pete_writel("arch/m68k/coldfire/m53xx.c:329", MCF_SCM_BCR_GBR | MCF_SCM_BCR_GBW, MCF_SCM_BCR);
}


void fbcs_init(void)
{
	pete_writeb("arch/m68k/coldfire/m53xx.c:335", 0x3E, MCFGPIO_PAR_CS);

	/* Latch chip select */
	pete_writel("arch/m68k/coldfire/m53xx.c:338", 0x10080000, MCF_FBCS1_CSAR);

	pete_writel("arch/m68k/coldfire/m53xx.c:340", 0x002A3780, MCF_FBCS1_CSCR);
	pete_writel("arch/m68k/coldfire/m53xx.c:341", MCF_FBCS_CSMR_BAM_2M | MCF_FBCS_CSMR_V, MCF_FBCS1_CSMR);

	/* Initialize latch to drive signals to inactive states */
	pete_writew("arch/m68k/coldfire/m53xx.c:344", 0xffff, 0x10080000);

	/* External SRAM */
	pete_writel("arch/m68k/coldfire/m53xx.c:347", EXT_SRAM_ADDRESS, MCF_FBCS1_CSAR);
	pete_writel("arch/m68k/coldfire/m53xx.c:348", MCF_FBCS_CSCR_PS_16 |
		MCF_FBCS_CSCR_AA |
		MCF_FBCS_CSCR_SBM |
		MCF_FBCS_CSCR_WS(1),
		MCF_FBCS1_CSCR);
	pete_writel("arch/m68k/coldfire/m53xx.c:353", MCF_FBCS_CSMR_BAM_512K | MCF_FBCS_CSMR_V, MCF_FBCS1_CSMR);

	/* Boot Flash connected to FBCS0 */
	pete_writel("arch/m68k/coldfire/m53xx.c:356", FLASH_ADDRESS, MCF_FBCS0_CSAR);
	pete_writel("arch/m68k/coldfire/m53xx.c:357", MCF_FBCS_CSCR_PS_16 |
		MCF_FBCS_CSCR_BEM |
		MCF_FBCS_CSCR_AA |
		MCF_FBCS_CSCR_SBM |
		MCF_FBCS_CSCR_WS(7),
		MCF_FBCS0_CSCR);
	pete_writel("arch/m68k/coldfire/m53xx.c:363", MCF_FBCS_CSMR_BAM_32M | MCF_FBCS_CSMR_V, MCF_FBCS0_CSMR);
}

void sdramc_init(void)
{
	/*
	 * Check to see if the SDRAM has already been initialized
	 * by a run control tool
	 */
	if (!(pete_readl("arch/m68k/coldfire/m53xx.c:372", MCF_SDRAMC_SDCR) & MCF_SDRAMC_SDCR_REF)) {
		/* SDRAM chip select initialization */
		
		/* Initialize SDRAM chip select */
		pete_writel("arch/m68k/coldfire/m53xx.c:376", MCF_SDRAMC_SDCS_BA(SDRAM_ADDRESS) |
			MCF_SDRAMC_SDCS_CSSZ(MCF_SDRAMC_SDCS_CSSZ_32MBYTE),
			MCF_SDRAMC_SDCS0);

	/*
	 * Basic configuration and initialization
	 */
	pete_writel("arch/m68k/coldfire/m53xx.c:383", MCF_SDRAMC_SDCFG1_SRD2RW((int)((SDRAM_CASL + 2) + 0.5)) |
		MCF_SDRAMC_SDCFG1_SWT2RD(SDRAM_TWR + 1) |
		MCF_SDRAMC_SDCFG1_RDLAT((int)((SDRAM_CASL * 2) + 2)) |
		MCF_SDRAMC_SDCFG1_ACT2RW((int)(SDRAM_TRCD + 0.5)) |
		MCF_SDRAMC_SDCFG1_PRE2ACT((int)(SDRAM_TRP + 0.5)) |
		MCF_SDRAMC_SDCFG1_REF2ACT((int)(SDRAM_TRFC + 0.5)) |
		MCF_SDRAMC_SDCFG1_WTLAT(3),
		MCF_SDRAMC_SDCFG1);
	pete_writel("arch/m68k/coldfire/m53xx.c:391", MCF_SDRAMC_SDCFG2_BRD2PRE(SDRAM_BL / 2 + 1) |
		MCF_SDRAMC_SDCFG2_BWT2RW(SDRAM_BL / 2 + SDRAM_TWR) |
		MCF_SDRAMC_SDCFG2_BRD2WT((int)((SDRAM_CASL + SDRAM_BL / 2 - 1.0) + 0.5)) |
		MCF_SDRAMC_SDCFG2_BL(SDRAM_BL - 1),
		MCF_SDRAMC_SDCFG2);

            
	/*
	 * Precharge and enable write to SDMR
	 */
	pete_writel("arch/m68k/coldfire/m53xx.c:401", MCF_SDRAMC_SDCR_MODE_EN |
		MCF_SDRAMC_SDCR_CKE |
		MCF_SDRAMC_SDCR_DDR |
		MCF_SDRAMC_SDCR_MUX(1) |
		MCF_SDRAMC_SDCR_RCNT((int)(((SDRAM_TREFI / (SYSTEM_PERIOD * 64)) - 1) + 0.5)) |
		MCF_SDRAMC_SDCR_PS_16 |
		MCF_SDRAMC_SDCR_IPALL,
		MCF_SDRAMC_SDCR);

	/*
	 * Write extended mode register
	 */
	pete_writel("arch/m68k/coldfire/m53xx.c:413", MCF_SDRAMC_SDMR_BNKAD_LEMR |
		MCF_SDRAMC_SDMR_AD(0x0) |
		MCF_SDRAMC_SDMR_CMD,
		MCF_SDRAMC_SDMR);

	/*
	 * Write mode register and reset DLL
	 */
	pete_writel("arch/m68k/coldfire/m53xx.c:421", MCF_SDRAMC_SDMR_BNKAD_LMR |
		MCF_SDRAMC_SDMR_AD(0x163) |
		MCF_SDRAMC_SDMR_CMD,
		MCF_SDRAMC_SDMR);

	/*
	 * Execute a PALL command
	 */
	pete_writel("arch/m68k/coldfire/m53xx.c:429", pete_readl("arch/m68k/coldfire/m53xx.c:429", MCF_SDRAMC_SDCR) | MCF_SDRAMC_SDCR_IPALL, MCF_SDRAMC_SDCR);

	/*
	 * Perform two REF cycles
	 */
	pete_writel("arch/m68k/coldfire/m53xx.c:434", pete_readl("arch/m68k/coldfire/m53xx.c:434", MCF_SDRAMC_SDCR) | MCF_SDRAMC_SDCR_IREF, MCF_SDRAMC_SDCR);
	pete_writel("arch/m68k/coldfire/m53xx.c:435", pete_readl("arch/m68k/coldfire/m53xx.c:435", MCF_SDRAMC_SDCR) | MCF_SDRAMC_SDCR_IREF, MCF_SDRAMC_SDCR);

	/*
	 * Write mode register and clear reset DLL
	 */
	pete_writel("arch/m68k/coldfire/m53xx.c:440", MCF_SDRAMC_SDMR_BNKAD_LMR |
		MCF_SDRAMC_SDMR_AD(0x063) |
		MCF_SDRAMC_SDMR_CMD,
		MCF_SDRAMC_SDMR);
				
	/*
	 * Enable auto refresh and lock SDMR
	 */
	pete_writel("arch/m68k/coldfire/m53xx.c:448", pete_readl("arch/m68k/coldfire/m53xx.c:448", MCF_SDRAMC_SDCR) & ~MCF_SDRAMC_SDCR_MODE_EN,
		MCF_SDRAMC_SDCR);
	pete_writel("arch/m68k/coldfire/m53xx.c:450", MCF_SDRAMC_SDCR_REF | MCF_SDRAMC_SDCR_DQS_OE(0xC),
		MCF_SDRAMC_SDCR);
	}
}

void gpio_init(void)
{
	/* Enable UART0 pins */
	pete_writew("arch/m68k/coldfire/m53xx.c:458", MCF_GPIO_PAR_UART_PAR_URXD0 | MCF_GPIO_PAR_UART_PAR_UTXD0,
		MCFGPIO_PAR_UART);

	/*
	 * Initialize TIN3 as a GPIO output to enable the write
	 * half of the latch.
	 */
	pete_writeb("arch/m68k/coldfire/m53xx.c:465", 0x00, MCFGPIO_PAR_TIMER);
	pete_writeb("arch/m68k/coldfire/m53xx.c:466", 0x08, MCFGPIO_PDDR_TIMER);
	pete_writeb("arch/m68k/coldfire/m53xx.c:467", 0x00, MCFGPIO_PCLRR_TIMER);
}

int clock_pll(int fsys, int flags)
{
	int fref, temp, fout, mfd;
	u32 i;

	fref = FREF;
        
	if (fsys == 0) {
		/* Return current PLL output */
		mfd = pete_readb("arch/m68k/coldfire/m53xx.c:479", MCF_PLL_PFDR);

		return (fref * mfd / (BUSDIV * 4));
	}

	/* Check bounds of requested system clock */
	if (fsys > MAX_FSYS)
		fsys = MAX_FSYS;
	if (fsys < MIN_FSYS)
		fsys = MIN_FSYS;

	/* Multiplying by 100 when calculating the temp value,
	   and then dividing by 100 to calculate the mfd allows
	   for exact values without needing to include floating
	   point libraries. */
	temp = 100 * fsys / fref;
	mfd = 4 * BUSDIV * temp / 100;
    	    	    	
	/* Determine the output frequency for selected values */
	fout = (fref * mfd / (BUSDIV * 4));

	/*
	 * Check to see if the SDRAM has already been initialized.
	 * If it has then the SDRAM needs to be put into self refresh
	 * mode before reprogramming the PLL.
	 */
	if (pete_readl("arch/m68k/coldfire/m53xx.c:505", MCF_SDRAMC_SDCR) & MCF_SDRAMC_SDCR_REF)
		/* Put SDRAM into self refresh mode */
		pete_writel("arch/m68k/coldfire/m53xx.c:507", pete_readl("arch/m68k/coldfire/m53xx.c:507", MCF_SDRAMC_SDCR) & ~MCF_SDRAMC_SDCR_CKE,
			MCF_SDRAMC_SDCR);

	/*
	 * Initialize the PLL to generate the new system clock frequency.
	 * The device must be put into LIMP mode to reprogram the PLL.
	 */

	/* Enter LIMP mode */
	clock_limp(DEFAULT_LPD);
     					
	/* Reprogram PLL for desired fsys */
	pete_writeb("arch/m68k/coldfire/m53xx.c:519", MCF_PLL_PODR_CPUDIV(BUSDIV/3) | MCF_PLL_PODR_BUSDIV(BUSDIV),
		MCF_PLL_PODR);
						
	pete_writeb("arch/m68k/coldfire/m53xx.c:522", mfd, MCF_PLL_PFDR);
		
	/* Exit LIMP mode */
	clock_exit_limp();
	
	/*
	 * Return the SDRAM to normal operation if it is in use.
	 */
	if (pete_readl("arch/m68k/coldfire/m53xx.c:530", MCF_SDRAMC_SDCR) & MCF_SDRAMC_SDCR_REF)
		/* Exit self refresh mode */
		pete_writel("arch/m68k/coldfire/m53xx.c:532", pete_readl("arch/m68k/coldfire/m53xx.c:532", MCF_SDRAMC_SDCR) | MCF_SDRAMC_SDCR_CKE,
			MCF_SDRAMC_SDCR);

	/* Errata - workaround for SDRAM opeartion after exiting LIMP mode */
	pete_writel("arch/m68k/coldfire/m53xx.c:536", MCF_SDRAMC_REFRESH, MCF_SDRAMC_LIMP_FIX);

	/* wait for DQS logic to relock */
	for (i = 0; i < 0x200; i++)
		;

	return fout;
}

int clock_limp(int div)
{
	u32 temp;

	/* Check bounds of divider */
	if (div < MIN_LPD)
		div = MIN_LPD;
	if (div > MAX_LPD)
		div = MAX_LPD;
    
	/* Save of the current value of the SSIDIV so we don't
	   overwrite the value*/
	temp = pete_readw("arch/m68k/coldfire/m53xx.c:557", MCF_CCM_CDR) & MCF_CCM_CDR_SSIDIV(0xF);
      
	/* Apply the divider to the system clock */
	pete_writew("arch/m68k/coldfire/m53xx.c:560", MCF_CCM_CDR_LPDIV(div) | MCF_CCM_CDR_SSIDIV(temp), MCF_CCM_CDR);
    
	pete_writew("arch/m68k/coldfire/m53xx.c:562", pete_readw("arch/m68k/coldfire/m53xx.c:562", MCF_CCM_MISCCR) | MCF_CCM_MISCCR_LIMP, MCF_CCM_MISCCR);
    
	return (FREF/(3*(1 << div)));
}

int clock_exit_limp(void)
{
	int fout;
	
	/* Exit LIMP mode */
	pete_writew("arch/m68k/coldfire/m53xx.c:572", pete_readw("arch/m68k/coldfire/m53xx.c:572", MCF_CCM_MISCCR) & ~MCF_CCM_MISCCR_LIMP, MCF_CCM_MISCCR);

	/* Wait for PLL to lock */
	while (!(pete_readw("arch/m68k/coldfire/m53xx.c:575", MCF_CCM_MISCCR) & MCF_CCM_MISCCR_PLL_LOCK))
		;
	
	fout = get_sys_clock();

	return fout;
}

int get_sys_clock(void)
{
	int divider;
	
	/* Test to see if device is in LIMP mode */
	if (pete_readw("arch/m68k/coldfire/m53xx.c:588", MCF_CCM_MISCCR) & MCF_CCM_MISCCR_LIMP) {
		divider = pete_readw("arch/m68k/coldfire/m53xx.c:589", MCF_CCM_CDR) & MCF_CCM_CDR_LPDIV(0xF);
		return (FREF/(2 << divider));
	}
	else
		return (FREF * pete_readb("arch/m68k/coldfire/m53xx.c:593", MCF_PLL_PFDR)) / (BUSDIV * 4);
}
