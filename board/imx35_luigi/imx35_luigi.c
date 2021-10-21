/*
 * Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
 *
 * (C) Copyright 2008-2009 Freescale Semiconductor, Inc.
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
#include <asm/io.h>
#include <asm/arch/mx35.h>
#include <asm/arch/mx35_pins.h>
#include <asm/arch/iomux.h>
#include <i2c.h>
#include <linux/ctype.h>

#ifdef CONFIG_HW_WATCHDOG
#include <watchdog.h>
#endif

/* watchdog registers */
#define WDOG_WCR __REG16(WDOG_BASE_ADDR + 0x00)
#define WDOG_WSR __REG16(WDOG_BASE_ADDR + 0x02)
#define WDOG_WRSR __REG16(WDOG_BASE_ADDR + 0x04)
#define WDOG_WICR __REG16(WDOG_BASE_ADDR + 0x06)
#define WDOG_WMCR __REG16(WDOG_BASE_ADDR + 0x08)

DECLARE_GLOBAL_DATA_PTR;

static u32 system_rev;

/* board id and serial number. */
static u8 serial_number[16+1];
static u8 board_id[16+1];

u32 get_board_rev(void)
{
	return system_rev;
}

static inline void setup_soc_rev(void)
{
	int reg;
	reg = __REG(IIM_BASE_ADDR + IIM_SREV);
	if (!reg) {
		reg = __REG(ROMPATCH_REV);
		reg <<= 4;
	} else
		reg += CHIP_REV_1_0;
	system_rev = 0x35000 + (reg & 0xFF);
}

static inline void set_board_rev(int rev)
{
	system_rev =  (system_rev & ~(0xF << 8)) | (rev & 0xF) << 8;
}

int is_soc_rev(int rev)
{
	return (system_rev & 0xFF) - rev;
}

int dram_init(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

	return 0;
}

/*************************************************************************
 * get_board_rev16() - setup to pass kernel board revision information
 *      16-byte alphanumeric containing the board revision.
 *************************************************************************/
const u8 *get_board_rev16(void)
{
	if (!isalnum(board_id[0]))
		return "0000000000000000";
	else
		return board_id;
}

/*************************************************************************
 * get_board_serial16() - setup to pass kernel serial number information
 *      16-byte alphanumeric containing the serial number.
 *************************************************************************/
const u8 *get_board_serial16(void)
{
	if (!isalnum(serial_number[0]))
		return "0000000000000000";
	else
		return serial_number;
}
/* initialize the board id and serial number globals. */
void setup_serial_number()
{
	unsigned i;

	/* SN is the first 16 bytes before the bootloader. */
	strncpy (board_id, (const char*)TEXT_BASE-16, 16);
	board_id[16] = 0;
	for (i = 0; i < 16; i++)
		if (!isalnum(board_id[i]))
			board_id[i]=0; /* clean up garbage characters. */

	/* serial number offset = base addr + FNS offset - u-boot offset + serial number offset */
	strncpy (serial_number, (const char*)TEXT_BASE+0x40c00-0xc00+0, 16);
	serial_number[16] = 0;
	for (i = 0; i < 16; i++)
		if (!isalnum(serial_number[i]))
			serial_number[i]=0; /* clean up garbage characters. */
}

int board_init(void)
{
	int i;

#ifdef CONFIG_HW_WATCHDOG
	/* set the timeout to the max number of ticks
	 * and WDZST
	 * leave other settings the same */
	WDOG_WCR = 0xff01 | (WDOG_WCR & 0xff);
	WDOG_WMCR = 0; /* Power Down Counter of WDOG is disabled. */
#endif

	setup_soc_rev();

	setup_serial_number(); /* initialize the board id and serial number globals. */

	/* enable clocks */
	__REG(CCM_BASE_ADDR + CLKCTL_CGR0) |= 0x003F0000;
	__REG(CCM_BASE_ADDR + CLKCTL_CGR1) |= 0x00030FFF;

#ifdef CONFIG_HARD_I2C
	/* setup pins for I2C1 */
	mxc_request_iomux(MX35_PIN_I2C1_CLK, MUX_CONFIG_SION);
	mxc_request_iomux(MX35_PIN_I2C1_DAT, MUX_CONFIG_SION);
#define PAD_CONFIG (PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE \
			| PAD_CTL_PUE_PUD | PAD_CTL_ODE_OpenDrain)

	mxc_iomux_set_pad(MX35_PIN_I2C1_CLK, PAD_CONFIG);
	mxc_iomux_set_pad(MX35_PIN_I2C1_DAT, PAD_CONFIG);
#undef PAD_CONFIG
#endif /* CONFIG_HARD_I2C */

	/* setup pins for FEC */
	mxc_request_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TX_EN, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_MDC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_MDIO, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TX_ERR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_ERR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_CRS, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA3, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA3, MUX_CONFIG_FUNC);

#define FEC_PAD_CTL_COMMON (PAD_CTL_DRV_3_3V|PAD_CTL_PUE_PUD| \
			PAD_CTL_ODE_CMOS|PAD_CTL_DRV_NORMAL|PAD_CTL_SRE_SLOW)
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_CLK, FEC_PAD_CTL_COMMON |
			  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_CLK,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_DV,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_COL,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA0,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA0,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_EN,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_MDC,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_MDIO,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_22K_PU);
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_ERR,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_ERR,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_CRS,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA1,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA1,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA2,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA2,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA3,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA3,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
#undef FEC_PAD_CTL_COMMON

	gd->bd->bi_arch_number = MACH_TYPE_MX35_3DS;	/* board id for linux */
	gd->bd->bi_boot_params = 0x80000100;	/* address of boot parameters */

	return 0;
}

#ifdef BOARD_LATE_INIT
static inline int board_detect(void)
{
	u8 buf[4];
	int id;

#ifdef CONFIG_HARD_I2C
	if (i2c_read(0x08, 0x7, 1, buf, 3) < 0) {
		printf("board_late_init: read PMIC@0x08:0x7 fail\n");
		return 0;
	}
	id = (buf[0] << 16) + (buf[1] << 8) + buf[2];
	printf("PMIC@0x08:0x7 is %x\n", id);
	id = (id >> 6) & 0x7;
	if (id == 0x7) {
		set_board_rev(1);
		return 1;
	}
#endif
	set_board_rev(1);
	return 1;
}

int board_late_init(void)
{
	u8 reg[3];
	int i;

	if (board_detect()) {
		mxc_request_iomux(MX35_PIN_WATCHDOG_RST, MUX_CONFIG_SION |
					MUX_CONFIG_ALT1);
		printf("i.MX35 CPU board version 2.0\n");

#ifdef CONFIG_HARD_I2C
		if (i2c_read(0x08, 0x1E, 1, reg, 3)) {
			printf("board_late_init: read PMIC@0x08:0x1E fail\n");
			return 0;
		}
		reg[2] |= 0x3;
		if (i2c_write(0x08, 0x1E, 1, reg, 3)) {
			printf("board_late_init: write PMIC@0x08:0x1E fail\n");
			return 0;
		}
		if (i2c_read(0x08, 0x20, 1, reg, 3)) {
			printf("board_late_init: read PMIC@0x08:0x20 fail\n");
			return 0;
		}
		reg[2] |= 0x1;
		if (i2c_write(0x08, 0x20, 1, reg, 3)) {
			printf("board_late_init: write PMIC@0x08:0x20 fail\n");
			return 0;
		}
#endif

		mxc_request_iomux(MX35_PIN_COMPARE, MUX_CONFIG_GPIO);
		mxc_iomux_set_input(MUX_IN_GPIO1_IN_5, INPUT_CTL_PATH0);
		__REG(GPIO1_BASE_ADDR + 0x04) |= 1 << 5;
		__REG(GPIO1_BASE_ADDR) |= 1 << 5;
	} else
		printf("i.MX35 CPU board version 1.0\n");

#ifdef CONFIG_HARD_I2C

	if (i2c_read(0x69, 0x20, 1, reg, 1) < 0) {
		printf("board_late_init: read PMIC@0x69:0x20 fail\n");
		return 0;
	}

	reg[0] |= 0x4;
	if (i2c_write(0x69, 0x20, 1, reg, 1) < 0) {
		printf("board_late_init: write back PMIC@0x69:0x20 fail\n");
		return 0;
	}

	for (i = 0; i < 1000; i++)
		udelay(200);

	if (i2c_read(0x69, 0x1A, 1, reg, 1) < 0) {
		printf("board_late_init: read PMIC@0x69:0x1A fail\n");
		return 0;
	}

	reg[0] &= 0x7F;
	if (i2c_write(0x69, 0x1A, 1, reg, 1) < 0) {
		printf("board_late_init: write back PMIC@0x69:0x1A fail\n");
		return 0;
	}
	for (i = 0; i < 1000; i++)
		udelay(200);

	reg[0] |= 0x80;
	if (i2c_write(0x69, 0x1A, 1, reg, 1) < 0) {
		printf("board_late_init: 2st write back PMIC@0x69:0x1A fail\n");
		return 0;
	}
	for (i = 0; i < 1000; i++)
		udelay(200);
	return 0;
#endif /* CONFIG_HARD_I2C */
	return 0;
}
#endif

int checkboard(void)
{
	const char *sn, *rev;
	unsigned i;

	printf("Board: MX35 Luigi [ ");
	switch (__REG(CCM_BASE_ADDR + CLKCTL_RCSR) & 0x0F) {
	case 0x0000:
		printf("POR");
		break;
	case 0x0002:
		printf("JTAG");
		break;
	case 0x0004:
		printf("RST");
		break;
	case 0x0008:
		printf("WDT");
		break;
	default:
		printf("unknown");
	}
	printf("]\n");
	printf("WDOG_WCR = 0x%hx\n", WDOG_WCR);
	printf("WDOG_WMCR = 0x%hx\n", WDOG_WMCR);

	/* serial number and board id */
	sn = get_board_serial16();
	rev = get_board_rev16();

	if (rev)
	printf ("Board Id: %.*s\n", 16, rev);

	if (sn)
	printf ("S/N: %.*s\n", 16, sn);

	return 0;
}

#ifdef CONFIG_HW_WATCHDOG
void hw_watchdog_reset(void)
{
	/* service the watchdog */
	WDOG_WSR = 0x5555;
	WDOG_WSR = 0xaaaa;
}
#endif
