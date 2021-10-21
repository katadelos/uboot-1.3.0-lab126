/*
 * Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
 *
 * (C) Copyright 2008 Freescale Semiconductor, Inc.
 * Fred Fan (r01011@freescale.com)
 *
 * Configuration settings for the MX31ADS Freescale board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/mx35.h>

 /* High Level Configuration Options */
#define CONFIG_ARM1136		1	/* This is an arm1136 CPU core */
#define CONFIG_MXC		1
#define CONFIG_MX35		1	/* in a mx31 */
#define CONFIG_MX35_HCLK_FREQ	24000000	/* RedBoot says 26MHz */
#define CONFIG_MX35_CLK32	32768

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define BOARD_LATE_INIT

#define UBOOT_IMAGE_SIZE 0x20000

/*
 * Disabled for now due to build problems under Debian and a significant increase
 * in the final file size: 144260 vs. 109536 Bytes.
 */
#if 0
#define CONFIG_OF_LIBFDT		1
#define CONFIG_FIT			1
#define CONFIG_FIT_VERBOSE		1
#endif

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1
#define CONFIG_SERIAL16_TAG		1
#define CONFIG_REVISION16_TAG	1

/*
 * Size of malloc() pool
 */
#define CFG_MALLOC_LEN		(CFG_ENV_SIZE + 128 * 1024)
#define CFG_GBL_DATA_SIZE	128/* size in bytes reserved for initial data */

/*
 * Hardware drivers
 */
/*
#define CONFIG_HARD_I2C		1
#define CONFIG_I2C_MXC		1
#define CFG_I2C_PORT		I2C_BASE_ADDR
#define CFG_I2C_SPEED		100000
#define CFG_I2C_SLAVE		0xfe
*/

#define CONFIG_MX35_UART	1
#define CFG_MX35_UART1		1

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX	1
#define CONFIG_BAUDRATE		115200
#define CFG_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

/* enable this if the bootloader should be tagged with version and crc info
 * #define CONFIG_BOARD_VERSION_STRUCTURE "../../board/imx35_luigi/verinfo.inc"
 */

/***********************************************************
 * Command definition
 ***********************************************************/

#include <config_cmd_default.h>

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
/*#define CONFIG_CMD_SPI*/
/*#define CONFIG_CMD_DATE*/
/*#define CONFIG_CMD_NAND*/

#ifdef CONFIG_HARD_I2C
#define CONFIG_CMD_I2C
#endif

#define CONFIG_CMD_MII

#undef CONFIG_CMD_PMIC

/* save boot time if not in dev mode */
#ifdef LUIGI_DEVMODE
#define CONFIG_BOOTDELAY	3
#else
#define CONFIG_BOOTDELAY	1
#endif

#define CONFIG_LOADADDR		0x80800000	/* loadaddr env var */

#define _STRINGIZE(s) #s
#define TOSTRING(s) _STRINGIZE(s)

#define	CONFIG_EXTRA_ENV_SETTINGS					\
 "uboot_net=tftpboot 0x84000000 u-boot.bin\0" \
 "uboot_serial=loady 0x84000000\0" \
 "uboot_ram=go 0x84000000\0" \
 "bootargs_diag=setenv bootargs tests=all\0" \
 "diags_net=tftpboot 0x84000000 diagmon.uimage; run bootargs_diag; bootm 0x84000000\0" \
 "diags_serial=loady 0x84000000; run bootargs_diag; bootm 0x84000000\0" \
 "bootargs_base=console=ttymxc0,115200 mem=256M panic=10\0" \
 "bootcmd_root_nfs=setenv bootargs $(bootargs_base) root=/dev/nfs rw nfsroot=$(nfsrootfs),v3,tcp rw ip=$(ipaddr):$(serverip):$(serverip):$(netmask):mario1 rootdelay=3\0" \
 "bootcmd_root_mmc=setenv bootargs $(bootargs_base) root=/dev/mmcblk1p1 rw ip=none\0" \
 "bootcmd_root_mvn=setenv bootargs $(bootargs_base) root=/dev/mmcblk0p1 rw ip=none\0" \
 "bootcmd_kernel_nfs=nfs 0x87f40400 $(nfsrootfs)/uImage; bootm\0" \
 "bootcmd_kernel_tftp=tftp 0x87f40400 uImage; bootm\0" \
 "bootcmd_nfs=run bootcmd_root_nfs; run bootcmd_kernel_nfs\0" \
 "bootcmd_flash=run bootcmd_root_mvn; run bootcmd_kernel_nor\0" \
 "bootcmd_card=run bootcmd_root_mmc; run bootcmd_kernel_nor\0" \
 "bootcmd_recovery=run bootcmd_root_recovery; run bootcmd_kernel_nor\0" \
 "bootcmd_defaultflash=setenv bootargs; run bootcmd_kernel_nor\0" \
 "bootcmd=bootm 0x87f40400\0" \
 "testmem=mtest 0x80000000 0x86ffffff\0" \
 "nfsrootfs=/nfsboot\0" \
 "ethaddr=00:22:33:44:55:66\0" \
 "cfgreset=protect off all ; erase " TOSTRING(CFG_ENV_ADDR) " +" TOSTRING(CFG_ENV_SECT_SIZE) "\0" \
 "bootretry=-1\0" \

#define CONFIG_HAS_ETH1
#define CONFIG_NET_MULTI 1
#define CONFIG_MXC_FEC
#define CONFIG_MII
#define CFG_DISCOVER_PHY

#define CFG_FEC0_IOBASE FEC_BASE_ADDR
#define CFG_FEC0_PINMUX	-1
#define CFG_FEC0_PHY_ADDR	0x1F
#define CFG_FEC0_MIIBASE -1

/*
#define NAND_MAX_CHIPS                1
#define CFG_NAND_BASE         (NFC_BASE_ADDR + 0x1E00)
#define CFG_MAX_NAND_DEVICE 1
*/

/*
 * The MX31ADS board seems to have a hardware "peculiarity" confirmed under
 * U-Boot, RedBoot and Linux: the ethernet Rx signal is reaching the CS8900A
 * controller inverted. The controller is capable of detecting and correcting
 * this, but it needs 4 network packets for that. Which means, at startup, you
 * will not receive answers to the first 4 packest, unless there have been some
 * broadcasts on the network, or your board is on a hub. Reducing the ARP
 * timeout from default 5 seconds to 200ms we speed up the initial TFTP
 * transfer, should the user wish one, significantly.
 */
#define CONFIG_ARP_TIMEOUT	200UL

/*
 * Miscellaneous configurable options
 */
#define CFG_LONGHELP		/* undef to save memory */
#define CFG_PROMPT		"uboot> "
#define CFG_CBSIZE		256	/* Console I/O Buffer Size */
/* Print Buffer Size */
#define CFG_PBSIZE		(CFG_CBSIZE + sizeof(CFG_PROMPT) + 16)
#define CFG_MAXARGS		16	/* max number of command args */
#define CFG_BARGSIZE		CFG_CBSIZE	/* Boot Argument Buffer Size */

#define CFG_MEMTEST_START	0	/* memtest works on */
#define CFG_MEMTEST_END		0x10000

#undef	CFG_CLKS_IN_HZ		/* everything, incl board info, in Hz */

#define CFG_LOAD_ADDR		CONFIG_LOADADDR

#define CFG_HZ			CONFIG_MX35_CLK32/* use 32kHz clock as source */

#define CONFIG_CMDLINE_EDITING	1

#define CONFIG_HW_WATCHDOG 1

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128 * 1024)	/* regular stack */

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1
#define PHYS_SDRAM_1		CSD0_BASE_ADDR
#define PHYS_SDRAM_1_SIZE	(256 * 1024 * 1024)

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CFG_NO_FLASH		1
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_FLASH

/* these don't really exist on our hardware */
#define CFG_FLASH_BASE		CS0_BASE_ADDR
#define CFG_MAX_FLASH_BANKS	1	/* max number of memory banks */
#define CFG_MAX_FLASH_SECT	512	/* max number of sectors on one chip */
/* Monitor at beginning of flash */
#define CFG_MONITOR_BASE	CFG_FLASH_BASE
#define CFG_MONITOR_LEN		(512 * 1024)	/* Reserve 256KiB */

#define CFG_ENV_IS_NOWHERE	1
#define CFG_ENV_SECT_SIZE	(128 * 1024)
#define CFG_ENV_SIZE		CFG_ENV_SECT_SIZE

/* Address and size of Redundant Environment Sector	*/
#define CFG_ENV_OFFSET_REDUND	(CFG_ENV_OFFSET + CFG_ENV_SIZE)
#define CFG_ENV_SIZE_REDUND	CFG_ENV_SIZE

/*
 * S29WS256N NOR flash has 4 32KiB small sectors at the beginning and at the
 * end. The rest of 32MiB is in 128KiB big sectors. U-Boot occupies the low
 * 4 sectors, if we put environment next to it, we will have to occupy 128KiB
 * for it. Putting it at the top of flash we use only 32KiB.
 */
#define CFG_ENV_ADDR		(CFG_MONITOR_BASE + CFG_ENV_SECT_SIZE)

#endif				/* __CONFIG_H */
