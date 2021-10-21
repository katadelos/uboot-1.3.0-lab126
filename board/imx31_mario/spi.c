/*
 *
 * (C) 2007. Embedded Alley Solutions, Inc. 
 * 
 * Based on Freescale BSP for Linux.
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
#include "spi.h"
#include "spi_defs.h"
#include <asm/arch/mario.h>

#define FALSE 0
#define TRUE  !FALSE

/* debug print outbound/inbound data */
/* #define SPI_DEBUG 1 */
/* #define SPI_DEBUG 2 */

#define IMX_SPI_CHIPSEL_CNT 	4

static inline void spi_write(u32 reg, u32 value)
{
#if SPI_DEBUG > 3
	printf("SPI WR: [%08x] <= %x\n", reg, value);
#endif
	udelay(100);
	__REG(reg) = value;
}

static inline u32 spi_read(u32 reg)
{
	u32 value = __REG(reg);

#if SPI_DEBUG > 3
	printf("SPI RD: [%08x] => %x\n", reg, value);
#endif
	return value;
}

const int spi_chipsel_cnt = IMX_SPI_CHIPSEL_CNT;

int spi_chipsel[IMX_SPI_CHIPSEL_CNT] = {
	0, 1, 2, 3
};

struct mxc_spi {
	u32 	 base;	/* __iomem */
        unsigned long spi_ipg_clk;
	struct  mxc_spi_unique_def *spi_ver_def;
};

static struct mxc_spi_unique_def spi_ver_0_4 = {
        .intr_bit_shift = MXC_CSPIINT_IRQSHIFT_0_4,
        .cs_shift = MXC_CSPICTRL_CSSHIFT_0_4,
        .bc_shift = MXC_CSPICTRL_BCSHIFT_0_4,
        .bc_mask = MXC_CSPICTRL_BCMASK_0_4,
        .drctrl_shift = MXC_CSPICTRL_DRCTRLSHIFT_0_4,
        .xfer_complete = MXC_CSPISTAT_TC_0_4,
        .bc_overflow = MXC_CSPISTAT_BO_0_4,
};

static struct mxc_spi spi_master_data = {
	.spi_ver_def 	= &spi_ver_0_4,
	.base 		= CSPI2_BASE_ADDR,
	.spi_ipg_clk	= -1,
};

static int spi_max_speed_hz = 4000000;
static const int spi_xfer_len = 32;
static int spi_xfer_bpw;
static u32 spi_mode = SPI_MODE_2 | SPI_CS_HIGH;
static const int cspi_module = 2;
static int spi_inited = FALSE;

static unsigned int spi_find_baudrate(struct mxc_spi *master_data,
                                      unsigned int baud)
{
        unsigned int divisor;
        unsigned int shift = 0;

        /* Calculate required divisor (rounded) */
        divisor = (master_data->spi_ipg_clk + baud / 2) / baud;
        while (divisor >>= 1)
                shift++;
        MXC_CSPICTRL_ADJUST_SHIFT(shift);
        if (shift > MXC_CSPICTRL_MAXDATRATE)
                shift = MXC_CSPICTRL_MAXDATRATE;

        return (shift << MXC_CSPICTRL_DATASHIFT);
}

void spi_irq(unsigned int irqs, int enable)
{
	u32 status = spi_read(spi_master_data.base + MXC_CSPIINT);

	if (enable) {
		status |= irqs;
	} else {
		status &= ~irqs;
	}

        spi_write(spi_master_data.base + MXC_CSPIINT, status);
}

static void spi_cs(int cs, int active)
{
	u32 ctrl, ctrl_mask;
	struct mxc_spi_unique_def *spi_ver_def = spi_master_data.spi_ver_def;

	ctrl = spi_read(spi_master_data.base + MXC_CSPICTRL);
	ctrl_mask = (MXC_CSPICTRL_LOWPOL | MXC_CSPICTRL_PHA | MXC_CSPICTRL_HIGHSSPOL |
             MXC_CSPICTRL_CSMASK << spi_ver_def->cs_shift |
             MXC_CSPICTRL_DATAMASK << MXC_CSPICTRL_DATASHIFT |
             spi_ver_def->bc_mask << spi_ver_def->bc_shift);
	ctrl &= ~ctrl_mask;

        ctrl |=
            (cs & MXC_CSPICTRL_CSMASK) << spi_ver_def->cs_shift;
        ctrl |= spi_find_baudrate(&spi_master_data, spi_max_speed_hz);
        ctrl |=
            (((spi_xfer_len - 1) & spi_ver_def->bc_mask) << spi_ver_def->bc_shift);
        if (spi_mode & SPI_CPHA)
                ctrl |= MXC_CSPICTRL_PHA;
        if (!(spi_mode & SPI_CPOL))
                ctrl |= MXC_CSPICTRL_LOWPOL;
        if (spi_mode & SPI_CS_HIGH)
                ctrl |= MXC_CSPICTRL_HIGHSSPOL;

	spi_write(spi_master_data.base + MXC_CSPICTRL, ctrl);
}

static int spi_isr(u32 tx, u32 *rx, int ignore)
{
	u32 status;
	int r = 0;

        status = spi_read(spi_master_data.base + MXC_CSPISTAT);
#if SPI_DEBUG > 1
	printf("Status = %X\n", status);
#endif
	if (status & MXC_CSPISTAT_RR) {
		u32 rx_tmp = spi_read(spi_master_data.base + MXC_CSPIRXDATA);

		// printf("%s: %x, ignore = %s\n", __FUNCTION__, rx_tmp, ignore ? "Yes" : "No" );
		if (!ignore) {
#if SPI_DEBUG > 0
			printf("%s: rcvd %08X\n", __FUNCTION__, rx_tmp);
#endif
			if (rx)
				*rx = rx_tmp;
			r = !0;
		}
	}

	spi_write(spi_master_data.base + MXC_CSPITXDATA, tx);
	spi_write(spi_master_data.base + MXC_CSPICTRL, spi_read(spi_master_data.base + MXC_CSPICTRL) | MXC_CSPICTRL_XCH);
#if SPI_DEBUG > 0
	printf("%s: sent %08X\n", __FUNCTION__, tx);
#endif
	spi_write(spi_master_data.base + MXC_CSPISTAT, status);
	return r;
}

static u32 get_oword(u8 *ptr, int offset)
{
	return *(u32*)(ptr + offset * spi_xfer_bpw);
}

static u32* get_iword(u8 *ptr, int offset)
{
	return (u32*)(ptr + offset * spi_xfer_bpw);
}
static void gpio_spi_active(int cspi)
{
        switch (cspi)
	{
        case 0:
                /* SPI1 */
                mxc_request_iomux(MX31_PIN_RI_DTE1, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                mxc_request_iomux(MX31_PIN_DCD_DTE1, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                mxc_request_iomux(MX31_PIN_DTR_DCE2, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                mxc_request_iomux(MX31_PIN_RI_DCE1, OUTPUTCONFIG_ALT1,
                                  INPUTCONFIG_ALT1);
                mxc_request_iomux(MX31_PIN_DCD_DCE1, OUTPUTCONFIG_ALT1,
                                  INPUTCONFIG_ALT1);
                mxc_request_iomux(MX31_PIN_DTR_DTE1, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                mxc_request_iomux(MX31_PIN_DSR_DTE1, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                mxc_request_iomux(MX31_PIN_DSR_DCE1, OUTPUTCONFIG_ALT1,
                                  INPUTCONFIG_ALT1);



                break;
        case 1:
                /* SPI2 */
                mxc_request_iomux(MX31_PIN_CSPI2_MISO, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                mxc_request_iomux(MX31_PIN_CSPI2_MOSI, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                mxc_request_iomux(MX31_PIN_CSPI2_SCLK, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                mxc_request_iomux(MX31_PIN_CSPI2_SS0, OUTPUTCONFIG_FUNC,
                                  INPUTCONFIG_FUNC);
                break;
        case 2:
                /* SPI3 */
                /*
                   mxc_request_iomux(MX31_PIN_CSPI2_MISO, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
                   mxc_request_iomux(MX31_PIN_CSPI2_MOSI, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
                   mxc_request_iomux(MX31_PIN_CSPI2_SCLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
                   mxc_request_iomux(MX31_PIN_CSPI2_SPI_RDY, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
                   mxc_request_iomux(MX31_PIN_CSPI2_SS0, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
                   mxc_request_iomux(MX31_PIN_CSPI2_SS1, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
                   mxc_request_iomux(MX31_PIN_CSPI2_SS2, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
               */
                break;
        default:
                break;
        }
}

static void spi_clk_enable(void)
{
	clk_spi_enable();
}

void spi_init(void)
{
	if (spi_inited) {
		return;
	}
#if SPI_DEBUG > 0
	printf("%s, entered\n", __FUNCTION__);
#endif
	spi_master_data.spi_ipg_clk = clk_ipg_rate();
	gpio_spi_active(cspi_module - 1);
	spi_clk_enable();
	spi_write(spi_master_data.base + MXC_CSPICTRL, MXC_CSPICTRL_ENABLE | MXC_CSPICTRL_MASTER);
	spi_xfer_bpw = spi_xfer_len/8;
	spi_inited = TRUE;
}

int spi_xfer(int cs, int data_len, u8* out, u8* in)
{
	int rx_count = 0, tx_count;
	u32 r, t;
	int rcode = 0;
#if SPI_DEBUG > 1
	void *d;
	int i;
#endif
	int first_word;

	spi_init();

#if SPI_DEBUG > 1
	printf("mario spi: chipselect %x, %d bytes\n", cs, data_len);
	for(i = 0, d = out; i < data_len / spi_xfer_bpw; i ++, d = out + i * spi_xfer_bpw) {
		switch(spi_xfer_len)
		{
		case 8 : printf("%02X ", *(u8*)d); break;
		case 16: printf("%04X ", *(u16*)d); break;
		case 32: printf("%08X ", *(u32*)d); break;
		}
	}
	printf("\n");
#endif

	spi_cs(cs, 1);
	spi_irq(MXC_CSPIINT_RREN, TRUE);

	rx_count = tx_count = 0;
	first_word = TRUE;

	while (rx_count < data_len/spi_xfer_bpw || tx_count < data_len/spi_xfer_bpw) {
#if SPI_DEBUG > 1
		printf("mario spi: %d/%d\n", rx_count, tx_count);
#endif
		t = (out && tx_count < data_len/spi_xfer_bpw) ? get_oword(out, tx_count) : 0;
		if (spi_isr(t, &r, first_word)) {
			if (in && rx_count < data_len/spi_xfer_bpw) {
				*(get_iword(in, rx_count)) = r;
			}
			rx_count ++;
		}
		first_word = FALSE;
		tx_count ++;

		/* sanity check */
		if (rx_count + tx_count > data_len/spi_xfer_bpw * 3) /* seems incorrect */ {
			rcode = !0;
			break;
		}
	}

	spi_irq(MXC_CSPIINT_RREN, FALSE);

	return rcode;
}
