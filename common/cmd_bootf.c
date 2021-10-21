/*
 * (C) Copyright 2008, Lab126, Inc.
 * Jon Mayo, jonmayo@lab126.com
 *
 * Contents:
 *   commands for doing automatic boot selection.
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

/*
 * Commands for doing automatic boot selection.
 *
 */

#include <common.h>
#include <command.h>
#include <image.h>
#include <asm/byteorder.h>
#if 0
/* enable to show reset status below */
#include <asm/arch/mx31-regs.h>
#endif

#define CRC32_RESIDUE 0xdebb20e3UL /* ANSI X3.66 residue */

#if defined(CONFIG_CMD_BOOTF)

#define CMD_BOOTF_DEBUG

#ifdef	CMD_BOOTF_DEBUG
# define	DEBUG(fmt,args...)	printf (fmt ,##args)
#else
# define DEBUG(fmt,args...)
#endif

#ifndef PAGE_SIZE
# define PAGE_SIZE 4096
#endif

#define BOOT_GLOBALS_SIZE PAGE_SIZE

struct boot_globals_t
{
    u8 reserved[BOOT_GLOBALS_SIZE - sizeof(u32)];
    u32 checksum;
};
typedef struct boot_globals_t boot_globals_t;

static boot_globals_t * const boot_globals=(void*)(PHYS_SDRAM_1+PHYS_SDRAM_1_SIZE-BOOT_GLOBALS_SIZE); /* tail of 128M */

extern int do_bootm (cmd_tbl_t *, int, int, char *[]);

/* Return the CRC of the bytes buf[0..len-1]. */
static unsigned bg_crc32(unsigned char *buf, int len) {
   /* uboot crc32() has opposite semantics than the linux kernel crc32_le() */
   return crc32(0L, buf, len);
}

static int check_boot_globals(void)
{
    int result = 0;

    if ( boot_globals )
    {
		u32 computed_residue, checksum;

		checksum = boot_globals->checksum;

		computed_residue = ~bg_crc32((u8 *)boot_globals, BOOT_GLOBALS_SIZE);

		printf("boot globals: computed residue = 0x%08X, checksum = 0x%08X\n", computed_residue, checksum);

		result = CRC32_RESIDUE == computed_residue;
    }

	if ( result )
	{
		printf("boot globals: passed\n");
	} else {
		printf("boot globals: failed\n");
	}

    return ( result );
}

static void save_boot_globals(void)
{
    if ( boot_globals ) {
        boot_globals->checksum = bg_crc32((u8 *)boot_globals, BOOT_GLOBALS_SIZE - sizeof (u32) );
    }
}

static void clear_boot_globals(unsigned long *kernel_boot_flag)
{
	unsigned long tmp;
	tmp = *kernel_boot_flag;
	memset(boot_globals, 0, BOOT_GLOBALS_SIZE);
	*kernel_boot_flag = tmp;
}

static int pick_kernel(char *addr1, char *addr2) {
	ulong n[2];
	int i;
	image_header_t *hdr;
	char *name[2];

	n[0] = simple_strtoul(addr1, NULL, 16);
	n[1] = simple_strtoul(addr2, NULL, 16);

	if(n[1]==0)
		return 1; /* default to primary */
	if(n[0]==0)
		return 2; /* primary address not right, use secondary */

	for(i=0;i<2;i++) {
		hdr = (image_header_t*)(n[i]);
		if (ntohl(hdr->ih_magic) == IH_MAGIC) {
			name[i] = (char *)&hdr->ih_name;
		} else {
			name[i]=0; /* refuse */
		}
	}

	if(name[1]==0)
		return 1; /* default to primary */
	if(name[0]==0)
		return 2; /* primary address not right, use secondary */

	/* return whichever one has the higher name */
	return strncmp(name[0], name[1], IH_NMLEN) < 0 ? 2 : 1;
}

static void boot_and_update ( cmd_tbl_t *cmdtp, int primary_or_secondary, unsigned long *kernel_boot_flag, char *addr)
{
	const char *name[]={"Primary", "Secondary"};
	char *local_args[3];

	local_args[0]="bootm";
	local_args[1]=addr;
	local_args[2]=NULL;
	printf("Booting %s kernel...\n", name[primary_or_secondary-1]);
	/* toggle the flag to try secondary if fails to boot */
	*kernel_boot_flag=primary_or_secondary==1?2:1;
	save_boot_globals();
	/* check_boot_globals(); -- an extra check for debug */
	do_bootm(cmdtp, 1, 2, local_args);
}

int do_bootf_cmd ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned long *kernel_boot_flag=(void*)((char*)boot_globals+0x0410); /* +0x0410 into the struct */

	u32 rcsr;
#if 0
	rcsr=__REG(CCM_RCSR);
	printf("Reset status: %d\n", rcsr&7); /* reason for reset */
#endif

	if(!check_boot_globals()) {
		printf("Boot globals invalid. Clearing\n"); /* but saving the kernel_boot_flag */
		clear_boot_globals(kernel_boot_flag);
	}

	if(*kernel_boot_flag==0) {
		/* select the newest kernel */
		*kernel_boot_flag=pick_kernel(argv[1], argv[2]);
		printf("Auto-selecting kernel %lu\n", *kernel_boot_flag);
		boot_and_update(cmdtp, *kernel_boot_flag, kernel_boot_flag, argv[*kernel_boot_flag]); /* try first choice */
		boot_and_update(cmdtp, *kernel_boot_flag, kernel_boot_flag, argv[*kernel_boot_flag]); /* try second choice */
	}

	printf("Using fallback kernel. Clearing boot globals\n"); /* but saving the kernel_boot_flag */
	clear_boot_globals(kernel_boot_flag);

	if(*kernel_boot_flag==1) {
		boot_and_update(cmdtp, 1, kernel_boot_flag, argv[1]); /* try primary */
		boot_and_update(cmdtp, 2, kernel_boot_flag, argv[2]); /* try secondary */
	} else {
		boot_and_update(cmdtp, 2, kernel_boot_flag, argv[2]); /* try secondary */
		boot_and_update(cmdtp, 1, kernel_boot_flag, argv[1]); /* try primary */
	}

	return 0;
}

U_BOOT_CMD(
	bootf,	3,	0, do_bootf_cmd,
	"bootf - boots from either of the selected kernels. depending on mode and settings.\n",
	"\n"
	"<addr1> - first uImage address\n"
	"<addr2> - second uImage address\n"
);
#endif
