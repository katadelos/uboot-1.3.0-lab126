/*
 * (C) Copyright 2004
 * Texas Instruments.
 * Richard Woodruff <r-woodruff2@...>
 * Kshitij Gupta <kshitij@...>
 *
 * Configuration settings for the Mario board
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <sizes.h>

 /* High Level Configuration Options */
#define CONFIG_ARM1136 1    /* This is an arm1136 CPU core */
#define CONFIG_MX31 1    /* in a mx31 */
#define CONFIG_MX31_HCLK_FREQ 26000000
#define CONFIG_MX31_CLK32 32000

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_CMDLINE_TAG       1    /* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS 1
#define CONFIG_INITRD_TAG        1
/* run out of RAM for now */
//#define CONFIG_SKIP_RELOCATE_UBOOT 1
//#define CONFIG_SKIP_LOWLEVEL_INIT 1
/*
 * Size of malloc() pool
 */
#define CFG_MALLOC_LEN           (CFG_ENV_SIZE + SZ_128K)
#define CFG_GBL_DATA_SIZE        128  /* size in bytes reserved for initial data */

/*
 * Hardware drivers
 */

#define CONFIG_MX31_UART 1
#define CFG_MX31_UART1 1

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX        1
#define CONFIG_BAUDRATE          115200
#define CFG_BAUDRATE_TABLE       {9600, 19200, 38400, 57600, 115200}

/* WAN hardware */
#define CONFIG_WAN_HW		1

#define CONFIG_BOARD_VERSION_STRUCTURE "../../board/imx31_mario/verinfo.inc"

/***********************************************************
 * Command definition
 ***********************************************************/

#include <config_cmd_default.h>
#define CONFIG_CMD_ENV
#define CONFIG_CMD_MII
#define CONFIG_CMD_PING
#undef CONFIG_CMD_USB
#define CONFIG_CMD_SPI
#define CONFIG_CMD_PMIC
#undef CONFIG_CMD_MMC
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION 1
#define CONFIG_CMD_BOOTF

#define CONFIG_LAB126_CHARGER 1

/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */
//#include <cmd_confdefs.h>

#define CONFIG_BOOTDELAY         3
#define CONFIG_BOOT_RETRY_TIME 	15		/* reset if command-line is idle for too long */
#define CONFIG_BOOT_RETRY_MIN 	5		/* cannot set "bootretry" to less than 5 seconds */

#define CONFIG_NETMASK          255.255.255.0
#define CONFIG_IPADDR 			192.168.15.244
#define CONFIG_SERVERIP 		192.168.15.200

#define _STRINGIZE(s) #s
#define TOSTRING(s) _STRINGIZE(s)

/* for 4M images kernel is at 0xa0220000
 * for 8M images kernel is at 0xa0400000
 */
#define CONFIG_EXTRA_ENV_SETTINGS \
 "_kernel1=0xa0060000\0" \
 "_kernel2=0xa0400000\0" \
 "_bladdr=0xa0020000\0" \
 "_prg_vectors=protect off all; erase 0xa0000000 +$(filesize); cp.b 0x84000000 0xa0000000 $(filesize); protect on all\0" \
 "_prg_bl=protect off all; erase $(_bladdr) +$(filesize); cp.b 0x84000000 $(_bladdr) $(filesize); protect on all\0" \
 "_prg_k=protect off all; erase $(_kernel1) +$(filesize); cp.b 0x80000000 $(_kernel1) $(filesize); protect on all\0" \
 "uboot_net=tftpboot 0x84000000 u-boot.bin\0" \
 "uboot_serial=loady 0x84000000\0" \
 "uboot_ram=go 0x84000000\0" \
 "prg_uboot_net=run uboot_net; run _prg_bl\0" \
 "prg_uboot_serial=run uboot_serial; run _prg_bl\0" \
 "prg_kernel_serial=loady 0x80000000; run _prg_k\0" \
 "prg_kernel_net=tftp 0x80000000 uImage; run _prg_k\0" \
 "bootargs_diag=setenv bootargs tests=all\0" \
 "diags_net=tftpboot 0x84000000 diagmon.uimage; run bootargs_diag; bootm 0x84000000\0" \
 "diags_serial=loady 0x84000000; run bootargs_diag; bootm 0x84000000\0" \
 "bootargs_base=console=ttymxc0,115200 mem=128M panic=10\0" \
 "bootcmd_root_nfs=setenv bootargs $(bootargs_base) root=/dev/nfs rw nfsroot=$(nfsrootfs),v3,tcp rw ip=$(ipaddr):$(serverip):$(serverip):$(netmask):mario1 rootdelay=3\0" \
 "bootcmd_root_mmc=setenv bootargs $(bootargs_base) root=/dev/mmcblk1p1 rw ip=none\0" \
 "bootcmd_root_mvn=setenv bootargs $(bootargs_base) root=/dev/mmcblk0p1 rw ip=none\0" \
 "bootcmd_kernel_nfs=nfs 0x80000000 $(nfsrootfs)/uImage; bootm\0" \
 "bootcmd_kernel_tftp=tftp 0x80000000 uImage; bootm\0" \
 "bootcmd_kernel_nor=bootf $(_kernel1) $(_kernel2)\0" \
 "bootcmd_nfs=run bootcmd_root_nfs; run bootcmd_kernel_nfs\0" \
 "bootcmd_flash=run bootcmd_root_mvn; run bootcmd_kernel_nor\0" \
 "bootcmd_card=run bootcmd_root_mmc; run bootcmd_kernel_nor\0" \
 "bootcmd_recovery=run bootcmd_root_recovery; run bootcmd_kernel_nor\0" \
 "bootcmd_defaultflash=setenv bootargs; run bootcmd_kernel_nor\0" \
 "bootcmd=run bootcmd_defaultflash\0" \
 "testmem=mtest 0x80000000 0x86ffffff\0" \
 "nfsrootfs=/nfsboot\0" \
 "ethaddr=00:22:33:44:55:66\0" \
 "cfgreset=protect off all ; erase " TOSTRING(CFG_ENV_ADDR) " +" TOSTRING(CFG_ENV_SECT_SIZE) "\0" \
 "bootretry=15\0" \

#define CONFIG_DRIVER_SMC911X 1
//#define CONFIG_DRIVER_SMC911X_BASE 0xb4020000
//#define CONFIG_DRIVER_SMC911X_BASE 0xb4000000

#define CONFIG_SMC911X_BASE 0xb4020000

/*
 * Miscellaneous configurable options
 */
#define CFG_LONGHELP             /* undef to save memory */
#define CFG_PROMPT               "uboot> "
#define CFG_CBSIZE               256  /* Console I/O Buffer Size */
/* Print Buffer Size */
#define CFG_PBSIZE               (CFG_CBSIZE+sizeof(CFG_PROMPT)+16)
#define CFG_MAXARGS              16          /* max number of command args */
#define CFG_BARGSIZE             CFG_CBSIZE  /* Boot Argument Buffer Size */

#define CFG_MEMTEST_START        0  /* memtest works on */
#define CFG_MEMTEST_END          0x10000

#undef CFG_CLKS_IN_HZ           /* everything, incl board info, in Hz */

#define CFG_LOAD_ADDR            (0) /* default load address */

#define CFG_HZ                 32000

#define CONFIG_CMDLINE_EDITING  1
/* #define CONFIG_MISC_INIT_R 1 */

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE         SZ_128K /* regular stack */

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS     1
#define PHYS_SDRAM_1             0x80000000

#define PHYS_SDRAM_1_SIZE        SZ_128M
/*
#define PHYS_SDRAM_1_SIZE        SZ_256M
*/
/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CFG_FLASH_BASE           0xa0000000
#define CFG_MAX_FLASH_BANKS      1           /* max number of memory banks */
#define CFG_MAX_FLASH_SECT       320     /* max number of sectors on one chip */
#define CFG_MONITOR_BASE CFG_FLASH_BASE /* Monitor at beginning of flash */
//#define CFG_MONITOR_LEN SZ_128K      /* Reserve 1 sector */

#define CFG_ENV_ADDR 0xA0010000
#define CFG_ENV_IS_IN_FLASH 1
#define CFG_ENV_SECT_SIZE SZ_64K
#define CFG_ENV_SIZE SZ_64K

/*-----------------------------------------------------------------------
 * CFI FLASH driver setup
 */
#define CFG_FLASH_CFI 1 /* Flash memory is CFI compliant */
#define CFG_FLASH_CFI_DRIVER 1 /* Use drivers/cfi_flash.c */
#define CFG_FLASH_USE_BUFFER_WRITE 1 /* Use buffered writes (~10x faster) */
#define CFG_FLASH_PROTECTION 1 /* Use hardware sector protection */

/* timeout values are in ticks */
#define CFG_FLASH_ERASE_TOUT     (100*CFG_HZ) /* Timeout for Flash Erase */
#define CFG_FLASH_WRITE_TOUT     (100*CFG_HZ) /* Timeout for Flash Write */

/*
 * JFFS2 partitions
 */
#undef CONFIG_JFFS2_CMDLINE
#define CONFIG_JFFS2_DEV "nor0"

/*-----------------------------------------------------------------------
 * Extra stuff
 */
#define CFG_ATLAS_TRICOLOR_LEDS 1 /* enable LED patterns on PMIC */

#endif /* __CONFIG_H */
