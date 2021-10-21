/*
 * Mini SPI driver for iMX31
 *
 * Copyright (C) 2008, Lab126, Inc.
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
#include <asm/arch/iomux.h>
#include <asm/arch/minispi.h>

int cspi_logging; /* should be 0 until serial output is configured */

/****** mini cspi driver ******/
#define CSPI_MAXLOOP 1000 /* max number of attempts to read/write FIFO */

#define CSPI_BASE 0x50010000 /* CSPI2 */
#define CSPI_RXDATA (CSPI_BASE)
#define CSPI_TXDATA (CSPI_BASE+4)
#define CSPI_CONREG (CSPI_BASE+8)
#define CSPI_INTREG (CSPI_BASE+12)
#define CSPI_DMAREG (CSPI_BASE+16)
#define CSPI_STATREG (CSPI_BASE+20)
#define CSPI_PERIODREG (CSPI_BASE+24)

/** CONREG bits **/
#define CSPI_CONREG_SS0				0x00000000	/* chip select SS0 */
#define CSPI_CONREG_SS1				0x01000000	/* chip select SS1 */
#define CSPI_CONREG_SS2				0x02000000  /* chip select SS2 */
#define CSPI_CONREG_SS3				0x03000000  /* chip select SS3 */
#define CSPI_CONREG_BITCOUNT_32		0x00001f00	/* BITCOUNT = 32 */
#define CSPI_CONREG_DATARATE_4		0x00000000	/* data rate = ipg/4 */
#define CSPI_CONREG_DATARATE_16		0x00020000	/* data rate = ipg/16 */
#define CSPI_CONREG_DATARATE_512	0x00070000	/* data rate = ipg/512 */
#define CSPI_CONREG_SSPOL_HIGH		0x00000080	/* active high */
#define CSPI_CONREG_SSCTL_NEGSS		0x00000040	/* negate SS between bursts */
#define CSPI_CONREG_PHA_PHASE1		0x00000020	/* Phase 1 operation */
#define CSPI_CONREG_POL_LOW			0x00000010	/* active low clock polarity */
#define CSPI_CONREG_SMC_IMMEDIATE	0x00000008	/* start when write to TXFIFO */
#define CSPI_CONREG_XCH				0x00000004	/* initiate exchange */
#define CSPI_CONREG_MODE_MASTER		0x00000002	/* master mode */
#define CSPI_CONREG_EN				0x00000001	/* enable bit */

#define CSPI_STATREG_TC				0x00000100	/* transfer complete */
#define CSPI_STATREG_BO				0x00000080	/* bit counter overflow */
#define CSPI_STATREG_RO				0x00000040	/* RXFIFO overflow */
#define CSPI_STATREG_RF				0x00000020	/* RXFIFO full */
#define CSPI_STATREG_RH				0x00000010	/* RXFIFO half full */
#define CSPI_STATREG_RR				0x00000008	/* RXFIFO ready */
#define CSPI_STATREG_TF				0x00000004	/* TXFIFO full */
#define CSPI_STATREG_TH				0x00000002	/* TXFIFO half empty */
#define CSPI_STATREG_TE				0x00000001	/* TXFIFO empty */

static void cspi_delay(void) {
	int i=1000000;
	while(i--) ;
}

/* read all remaining data from rxfifo */
static void cspi_rx_flush(void) {
	int maxloop=CSPI_MAXLOOP;
	u32 junk;
	while(maxloop--) {
		if((__REG(CSPI_STATREG)&CSPI_STATREG_RR)) {
			junk=__REG(CSPI_RXDATA);
			if(cspi_logging) printf("cspi flushed %08x\n", junk);
		}
	}
}

/* loop until the tx is empty */
static void cspi_tx_flush(void) {
	int maxloop=CSPI_MAXLOOP;
	while(maxloop-- && !(__REG(CSPI_STATREG)&CSPI_STATREG_TE)) {
		cspi_delay();
	}	
}

void cspi_init(void) {
	mxc_request_iomux(MX31_PIN_CSPI2_MISO, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSPI2_MOSI, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSPI2_SCLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSPI2_SS0, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

	__REG(CSPI_CONREG) = CSPI_CONREG_EN
		|CSPI_CONREG_MODE_MASTER
		|CSPI_CONREG_SSPOL_HIGH
		|CSPI_CONREG_DATARATE_16
		|CSPI_CONREG_BITCOUNT_32
		|CSPI_CONREG_SS0;
	__REG(CSPI_INTREG) = 0; /* no interrupts */
	__REG(CSPI_DMAREG) = 0; /* no DMA */
	__REG(CSPI_PERIODREG) = 500; /* 500 waits states - SPI clock source */
	cspi_rx_flush();
}

void cspi_shutdown(void) {
	cspi_tx_flush();
	__REG(CSPI_CONREG) = 0; /* disable the interface */
	/* TODO: disable iomux settings */
}

int cspi_write(u32 d) {
	int maxloop=CSPI_MAXLOOP;
	while(maxloop--) {
		/* loop though until space is available in queue */
		if((__REG(CSPI_STATREG)&CSPI_STATREG_TF)==0) {
			__REG(CSPI_TXDATA)=d;
			__REG(CSPI_CONREG)|=CSPI_CONREG_XCH;
			return 1; /* success */
		}
	}
	if(cspi_logging) printf("cspi write failed\n");
	return 0; /* failure */
}

int cspi_read(u32 *d) {
	int maxloop=CSPI_MAXLOOP;
	while(maxloop--) {
		/* loop though until data is in the queue */
		if((__REG(CSPI_STATREG)&CSPI_STATREG_RR)) {
			*d=__REG(CSPI_RXDATA);
			return 1; /* success */
		}
	}
	if(cspi_logging) printf("cspi read failed\n");
	return 0; /* failure */
}

