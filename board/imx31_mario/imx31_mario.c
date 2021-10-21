/*
 * Mario board init. 
 *
 * Copyright (C) 2007, Lab126, Inc.
 *
 * (C) 2007. Embedded Alley Solutions, Inc. 
 *
 * based on imx31_lite 
 * (c) 2007 Pengutronix, Sascha Hauer <s.hauer@...>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#include <common.h>
#include <asm/arch/mx31.h>
#include <asm/arch/mx31-regs.h>
#include <asm/arch/minispi.h>

DECLARE_GLOBAL_DATA_PTR;

int charger_is_present(void);

int dram_init (void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

	return 0;
}

#define MC13783_GPO1EN		0x00000040
#define MC13783_GPO1STBY	0x00000080
#define MC13783_GPO4EN		0x00001000
#define MC13783_GPO4STBY	0x00002000

/* Atlas register 2 bit definitions */
#define MC13783_CHGDETS		0x00000040
#define MC13783_CHGCURRS	0x00000800
#define MC13783_USB4V4S		0x00010000

/* Atlas register 13 bit definitions */
#define MC13783_CLK32KMCUEN	0x00000040
#define MC13783_BPDET_3_4V	(3 << 16)

/* Atlas register 48 bit definitions */
#define MC13783_ICHRG_70MA	(1 << 3)
#define MC13783_ICHRG_266MA	(3 << 3)
#define MC13783_ICHRG_443MA	(5 << 3)
#define MC13783_VCHRG_4_2V	(3 << 0)
#define MC13783_CHRGLEDEN	0x00040000

#undef  USB_COMPLIANCE_REQUIRED

#ifdef USB_COMPLIANCE_REQUIRED
#define MC13783_ICHRG_DFLT	MC13783_ICHRG_70MA
#else
#define MC13783_ICHRG_DFLT	MC13783_ICHRG_266MA
#endif

int board_init (void)
{
	u32 tmp;

	/* set necessary ATLAS MC13783 bits */

	cspi_init();
	
	/* write MC13783 register 34 - USB 3.3V and 26MHz osc */
	cspi_write(0xc4000000
		| MC13783_GPO1EN | MC13783_GPO1STBY  /* 3V3_EN - usb */
		| MC13783_GPO4EN | MC13783_GPO4STBY  /* HSOSC_EN - hi-spd osc */
	);
	cspi_read(&tmp); /* discard the value */

	/* Set reg 13 - Power Control 0 - BP Detection Thresholds */
	cspi_write(0x9a000000
		| MC13783_BPDET_3_4V	/* raise BP detect threshold to 3.4V */
		| MC13783_CLK32KMCUEN	/* defaults to 1 on reset */
	);
	cspi_read(&tmp);

	/* Determine proper setting for charge LED */
	if (charger_is_present())
		tmp = MC13783_CHRGLEDEN;
	else
		tmp = 0;
	
	/* Set reg 48 - Charger 0 - 4.2V at 70mA, and possibly charge LED */
	cspi_write(0xe0000000
		| MC13783_VCHRG_4_2V	/* VCHRG = 0b011  - 4.2V */
		| MC13783_ICHRG_DFLT	/* ICHRG = 70mA (or 266mA) */
		| tmp		/* charge LED on, if appropriate */
	);
	cspi_read(&tmp); /* discard the value */

	/* write MC13783 register 15 - ON1BRSTEN */
	cspi_write(0x9e000002); /* ON1BRSTEN */
	cspi_read(&tmp); /* discard the value */

#if CFG_ATLAS_TRICOLOR_LEDS
#define PMIC_WRITE(reg, value) do { cspi_write(0x80000000|((reg)<<25)|(value)); cspi_read(&tmp); } while(0)

	PMIC_WRITE(51, 0x000001); /* enable LEDs */
#endif

	/* cspi_shutdown(); * leave SPI on for other routines to use. */

	/* Use the high speed osc */
	tmp=__REG(CCM_CCMR);
	__REG(CCM_CCMR)=(tmp&~CCMR_PRCS_MASK)|0x4;

	/*  CS0 init is done by low_level init */
	/* remove this once NOR issue is resolved */
	/* sed@ */

	/*  __REG(CSCR_U(0)) = 0x0000cf03; /\* CS0: Nor Flash *\/ */
	/*  __REG(CSCR_L(0)) = 0xa0330d01; */
	/*  __REG(CSCR_A(0)) = 0x00220800; */

	__REG(CSCR_U(4)) = 0x0000dcf6; /* CS4: Network Controller */
	__REG(CSCR_L(4)) = 0x444a4541;
	__REG(CSCR_A(4)) = 0x44443302;

	/* setup pins for UART1 */
	mx31_gpio_mux(MUX_RXD1__UART1_RXD_MUX);
	mx31_gpio_mux(MUX_TXD1__UART1_TXD_MUX);
	mx31_gpio_mux(MUX_RTS1__UART1_RTS_B);
	mx31_gpio_mux(MUX_RTS1__UART1_CTS_B);

	gd->bd->bi_arch_number = 0x559; /* board id for linux */
	gd->bd->bi_boot_params = (0x80000100); /* adress of boot parameters */

	return 0;
}

int checkboard (void)
{
	char sn[17], bid[17];

	printf("Board: i.MX31 Mario\n");

	strncpy(sn, (const char*)0xa0008000, 16);
	sn[16]=0;
	printf("S/N: %s\n", sn);

	strncpy(bid, (const char*)0xa0008010, 16);
	bid[16]=0;

	printf("Board Id: %s\n", bid);

	/* mx31_dump_clocks(); */
	return 0;
}


/* Returns 1 if a charger is present, 0 if not. */

int charger_is_present(void)
{
	u32	tmp;

	/* Try and determine if a charger is connected	*/
	
	cspi_write(0x04000000);	/* read reg 2 - Interrupt Sense 0 */
	cspi_read(&tmp);

	if (tmp == 0) {
		cspi_init();
		cspi_write(0x04000000);
		cspi_read(&tmp);
	}

	/* If the voltage comparators indicate VBUS is above 4.4V, we either
	 * are attached to a PC or a 5V USB adapter.
	 */

	if (tmp & MC13783_USB4V4S)
		return /* 1 */ tmp;

	/* If we're attached to a charger, we should see both current detect
	 * *and* charger detect set.
	 */

	if ((tmp & (MC13783_CHGCURRS | MC13783_CHGDETS)) ==
		(MC13783_CHGCURRS | MC13783_CHGDETS))
		return /* 1 */ tmp;

	return 0;	/* charger not present */
}


void set_charge_current_500mA(void)
{
	u32 tmp;

	/* Determine proper setting for charge LED */

	tmp = charger_is_present();
	printf("set_charge_current_500mA: reg 2 = 0x%x\n", tmp);

	if (tmp)
		tmp = MC13783_CHRGLEDEN;
	else
		tmp = 0;
	
	cspi_write(0xe0000000
		| MC13783_VCHRG_4_2V
		| MC13783_ICHRG_443MA
		| tmp
	);
	cspi_read(&tmp);
}

