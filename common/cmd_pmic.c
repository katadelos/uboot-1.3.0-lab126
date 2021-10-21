/*
 * Copyright 2007
 * Jon Mayo, Lab126, Inc, jonmayo@lab126.com
 */

/* PMIC access via SPI */

#include <common.h>
#include <command.h>
#include <spi.h>

#if defined(CONFIG_CMD_PMIC)

/*
 * Constants
 */

static const struct {
	const char *name;
	char reg, bit_h, bit_l;
} pmic_names[] = {
	{ "vchrg", 48, 2, 0 },
	{ "ichrg", 48, 6, 3 },
	{ "ichrgtr", 48, 9, 7 },
	{ "fetovrd", 48, 10, 10 },
	{ "fetctrl", 48, 11, 11 },
	{ "rvrsmode", 48, 13, 13 },
	{ "ovctrl", 48, 16, 15 },
	{ "uchen", 48, 17 },
	{ "chrgleden", 48, 18, 18},
	{ "chrgrawpden", 48, 19, 19 },
	{ "on1brsten", 15, 1, 1},
};

/*
 * Definitions
 */

#ifndef NR
#   define NR(x) (sizeof(x)/sizeof*(x))
#endif

#ifndef MAX_SPI_BYTES
#   define MAX_SPI_BYTES 32     /* Maximum number of bytes we can handle */
#endif

#define BITLEN 32

enum pmic_act {
	ACT_LIST,
	ACT_WR,
	ACT_RD,
	ACT_WOR,
	ACT_SET,
	ACT_GET
};

/*
 * Globals
 */
extern spi_chipsel_type spi_chipsel[];
extern int spi_chipsel_cnt;
static int device=0; /* PMIC is attached to entry 0 (CSPI2) */

static int read_reg(spi_chipsel_type cs, int reg, unsigned *value) {
	uchar dout[MAX_SPI_BYTES], din[MAX_SPI_BYTES];
	int i;
	unsigned tmp;

	dout[0]=0;
	dout[1]=0;
	dout[2]=0;
	dout[3]=(reg<<1)&0x7e; /* force read only mode */

	if(spi_xfer(cs, BITLEN, dout, din) != 0) {
		printf ("Error with the SPI transaction.\n");
		return 0; /* failure */
	}

	/* load the value into tmp */
	tmp=0;
	i=((BITLEN+7)/8);
	while(i>0) {
		tmp=(tmp<<8)|din[--i];
	}

	*value=tmp;	

	return 1; /* success */
}

static int write_reg(spi_chipsel_type cs, int reg, unsigned value, unsigned *orig) {
	uchar dout[MAX_SPI_BYTES], din[MAX_SPI_BYTES];
	unsigned tmp;
	int i;

	dout[0]=value&255;
	dout[1]=(value>>8)&255;
	dout[2]=(value>>16)&255;
	dout[3]=((value>>24)&255)|128; /* force write bit set */

	if(spi_xfer(cs, BITLEN, dout, din) != 0) {
		printf ("Error with the SPI transaction.\n");
		return 0; /* failure */
	}

	/* load the result into tmp */
	tmp=0;
	i=((BITLEN+7)/8);
	while(i>0) {
		tmp=(tmp<<8)|din[--i];
	}
	
	if(orig) {
		*orig=tmp;
	}
	
	return 1; /* success */
}

static void dump_value(const char *title, uchar din[MAX_SPI_BYTES]) {
	int i;

	printf ("%s: ", title);
	i=(BITLEN+7)/8;
	while(i>0) {
		printf ("%02X", din[--i]);
	}
	printf ("\n");
}

/* 
 * mask - 0 bits set for valid bits of data
 */
static int read_and_modify(spi_chipsel_type cs, unsigned data, unsigned mask) {
	int reg;
	unsigned tmp;

	if((data&mask)!=0) {
		printf ("Input value exceeds bit mask.\n");
		return 0;
	}

	reg=(data>>25)&63; /* just used for printing */
	
	if(!read_reg(cs, reg, &tmp)) {
		printf ("Could not read register.\n");
		return 0;
	}

	tmp=(tmp&mask)|data;
	
	if(!write_reg(cs, reg, tmp, 0)) {
		printf ("Could not write register.\n");
		return 0;
	}

	if(!read_reg(cs, reg, &tmp)) {
		printf ("Could not read back register.\n");
		return 0;
	}

	printf("wrote %06X\n", tmp);

	return 1; /* success */
}

/* generate a command */
static int make_pmic_wr_command(unsigned reg, unsigned bit_l, unsigned bit_h, unsigned value, unsigned *data_out, unsigned *mask_out) {

	if(bit_l>bit_h) {
		printf ("High bit and Low bit are reversed?\n");
		return 0; /* failure */
	}

	/* check value */
	if((value&0xff000000) != 0) {
		printf ("Input value exceeds 24-bits\n");
		return 0; /* failure */
	}

	/* data mask and command mask */
	*mask_out=
		(((~1U)<<(bit_h))|((~0U)>>(31-bit_l)>>1)) /* data mask */
		& 0x01ffffff; /* command mask */

	/* data and write command combined */
	*data_out=
		(((reg<<1)|0x80)<<24) /* command */
		| (value<<bit_l); /* data */

	/* verify the mask and data we generated */
	if((*data_out&*mask_out) != 0) {
		printf ("Input value exceeds bit size. (%u bits)\n", bit_h+1-bit_l);
		printf ("data=%08x mask=%08x\n", *data_out, *mask_out);
		return 0;
	}

	return 1;
}

static int lookup_command_by_name(const char *name, int value, unsigned *data_out, unsigned *mask_out) {
	int i;
	for(i=0;i<NR(pmic_names);i++) {
		if(strcmp(pmic_names[i].name, name) == 0) {
			/* found a matching entry */
			if(!make_pmic_wr_command(
				pmic_names[i].reg, pmic_names[i].bit_l, pmic_names[i].bit_h,
				value, data_out, mask_out)
			) {
				return 0;
			}
			return 1; /* success */
		}
	}
	printf ("Unknown PMIC parameter named '%s'\n", name);
	return 0;
}

/*
 * PMIC read/write
 *
 * Syntax:
 *   pmic wr {register} {value}
 *   	{register} - pmic register id in decimal or hex
 *   	{value} - hex or 0b binary value
 *   pmic rd {register}
 *   	{register} - pmic register id in decimal or hex
 *   	{value} - hex or 0b binary value
 *   pmic wor {register} {bit_high} {big_low} {value}
 *   	{register} - pmic register id in decimal or hex
 *   	{value} - hex or 0b binary value
 *   pmic set {name} {value}
 *   pmic get {name}
 *   pmic list
 *   	list names that we know about
 */

int do_pmic (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int res=1; /* default is failure */
	static enum pmic_act last_cmd=ACT_LIST;
	static uchar dout[MAX_SPI_BYTES];
	uchar din[MAX_SPI_BYTES];
	static unsigned wor_data, wor_mask;

	if ((flag & CMD_FLAG_REPEAT) == 0) {
		/* we're not repeating, parse commands */
		if(strcmp(argv[1], "list") == 0 && argc == 2) {
			last_cmd=ACT_LIST;
		} else if(strcmp(argv[1], "wr") == 0 && argc == 4) {
			unsigned tmp;
			tmp=simple_strtoul(argv[3], NULL, 16);
			dout[0]=tmp&255;
			dout[1]=(tmp>>8)&255;
			dout[2]=(tmp>>16)&255;
			dout[3]=(simple_strtoul(argv[2], NULL, 0)<<1)|0x80;
			last_cmd=ACT_WR;
		} else if(strcmp(argv[1], "rd") == 0 && argc == 3) {
			dout[0]=0;
			dout[1]=0;
			dout[2]=0;
			dout[3]=simple_strtoul(argv[2], NULL, 0)<<1;
			last_cmd=ACT_RD;
		} else if(strcmp(argv[1], "wor") == 0 && argc == 6) {
			unsigned bit_h, bit_l, reg, value;

			reg=simple_strtoul(argv[2], NULL, 0);
			bit_h=simple_strtoul(argv[3], NULL, 0);
			bit_l=simple_strtoul(argv[4], NULL, 0);
			value=simple_strtoul(argv[5], NULL, 16);

			if(!make_pmic_wr_command(reg, bit_l, bit_h, value, &wor_data, &wor_mask)) {
				return 1; /* failure */
			}

			last_cmd=ACT_WOR;
		} else if(strcmp(argv[1], "set") == 0 && argc==4) {
			unsigned value;	
			value=simple_strtoul(argv[3], NULL, 0);
			if(!lookup_command_by_name(argv[2], value, &wor_data, &wor_mask)) {
				return 1; /* failure */
			}
			last_cmd=ACT_SET;
		} else if(strcmp(argv[1], "get") == 0 && argc==3) {
			if(!lookup_command_by_name(argv[2], 0, &wor_data, &wor_mask)) {
				return 1; /* failure */
			}
			wor_mask=0x01ffffff; /* command mask */
			wor_data&=~wor_mask;
			last_cmd=ACT_GET;
		} else {
			printf ("Usage:\n%s\n", cmdtp->usage);
			return 1;
		}
	}

	switch(last_cmd) {
		case ACT_LIST: {
				int i;
				for(i=0;i<NR(pmic_names);i++) {
					printf("%s (reg %d - %u:%u)\n", pmic_names[i].name, pmic_names[i].reg, pmic_names[i].bit_h, pmic_names[i].bit_l);
				}
			}
			break;
		case ACT_WR:
		case ACT_RD:
			if(last_cmd==ACT_RD) {
				printf ("Reading PMIC register %u\n", (dout[3]>>1)&63);
			} else {
				printf ("Writing PMIC register %u : %02X %02X %02X\n", (dout[3]>>1)&63, dout[2], dout[1], dout[0]);
			}
			if(spi_xfer(spi_chipsel[device], BITLEN, dout, din) != 0) {
				printf ("Error with the SPI transaction.\n");
			} else {
				dump_value("Result", din);
				res=0; /* success */
			}
			break;
		case ACT_WOR:
		case ACT_SET:
			read_and_modify(spi_chipsel[device], wor_data, wor_mask);
			break;
		case ACT_GET: {
				unsigned v;
				read_reg(spi_chipsel[device], wor_data>>25, &v);
				/* TODO: shift and mask read value */
				printf("read %06X\n", v);
			}
			break;
	}
/*
	debug ("spi_chipsel[%d] = %08X\n",
		device, (uint)spi_chipsel[device]);

	if(spi_xfer(spi_chipsel[device], 32, dout, din) != 0) {
		printf("Error with the SPI transaction.\n");
	}
*/
 

	return res;
}

/***************************************************/

U_BOOT_CMD(
	pmic,	6,	1,	do_pmic,
	"pmic    - PMIC utility commands\n",
	"[wr|rd|wor|list|set|get] args...\n"
	"wr <reg> <hexvalue>\n"
	"    - write a register\n"
	"rd <reg>\n"
	"    - read a register\n"
	"wor <reg> <bit_high> <bit_low> <hexvalue>\n"
	"    - set some bits in a register (write-OR)\n"
	"list\n"
	"    - lists names\n"
	"set <name> <value>\n"
	"    - set a register by name\n"
	"get <name>\n"
	"    - get a register by name\n"
);

#endif
