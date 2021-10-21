/*
 *
 * (C) 2007. Embedded Alley Solutions, Inc. 
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
#include <asm/arch/mario.h>
#include "mmc.h"

#define IMX_MMC_BASE 0x53f10000

static int mmc_ok = 0;
static struct mxcmci_host {
	u32 base;
} nand = {
	.base = IMX_MMC_BASE,
};

static void __raw_writel(u32 word, u32 reg)
{
	__REG(reg) = word;
}

static u32 __raw_readl(u32 reg)
{
	u32 value = __REG(reg);
	return value;
}

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
	switch (module) 
	{
	case 0:     // moviNAND on Mario
		mxc_request_iomux(MX31_PIN_SD1_CLK, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_CMD, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_DATA0, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_DATA1, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_DATA2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_DATA3, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);

		mxc_iomux_set_pad(MX31_PIN_SD1_CLK,
				  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
		mxc_iomux_set_pad(MX31_PIN_SD1_CMD,
				  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA0,
				  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA1,
				  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA2,
				  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA3,
				  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
		break;

	case 1:     // SD/MMC Card Slot on Mario
		mxc_request_iomux(MX31_PIN_PC_CD2_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_CD1_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_WAIT_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_READY, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_VS1, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_PWRON, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);

#if 0
        mxc_iomux_set_pad(MX31_PIN_PC_CD2_B,
                  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
        mxc_iomux_set_pad(MX31_PIN_PC_CD1_B,
                  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
        mxc_iomux_set_pad(MX31_PIN_PC_WAIT_B,
                  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
        mxc_iomux_set_pad(MX31_PIN_PC_READY,
                  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
        mxc_iomux_set_pad(MX31_PIN_PC_VS1,
                  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
        mxc_iomux_set_pad(MX31_PIN_PC_PWRON,
                  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
#endif
		break;
	default:
		break;
	}
}


/*!
 * This function resets the SDHC host.
 *
 * @param host  Pointer to MMC/SD  host structure
 */
static void mxcmci_softreset(struct mxcmci_host *host)
{
        /* reset sequence */
        __raw_writel(0x8, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x9, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x1, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x1, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x1, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x1, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x1, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x1, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x1, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x1, host->base + MMC_STR_STP_CLK);
        __raw_writel(0x3f, host->base + MMC_CLK_RATE);

        __raw_writel(0xff, host->base + MMC_RES_TO);
        __raw_writel(512, host->base + MMC_BLK_LEN);
        __raw_writel(1, host->base + MMC_NOB);
}

/*
 * Exported stuff
 */
int mmc_init(void)
{
        iomux_config_mux(MX31_PIN_NFCLE, OUTPUTCONFIG_GPIO,
                                 INPUTCONFIG_GPIO);
	spba_take_ownership(SPBA_SDHC1, SPBA_MASTER_A | SPBA_MASTER_C);
	gpio_sdhc_active(0 /* the MoviNAND */);
	pmic_init();
	printf("mario: mmc init\n");
	mxcmci_softreset(&nand);
        if (__raw_readl(nand.base + MMC_REV_NO) != SDHC_REV_NO) {
                printf( "%s: wrong rev.no. 0x%08x. aborting.\n",
                       __FUNCTION__, MMC_REV_NO);
                goto out;
        }
        __raw_writel(READ_TO_VALUE, nand.base + MMC_READ_TO);
        __raw_writel(INT_CNTR_END_CMD_RES, nand.base + MMC_INT_CNTR);

	mmc_ok = !0;
out:
	return mmc_ok;
}

int
mmc_read(ulong src, uchar *dst, int size)
{
	return -1;
}

int
mmc_write(uchar *src, ulong dst, int size)
{
	return -1;
}

int
mmc2info(ulong addr)
{
	if (addr >= IMX_MMC_BASE && addr < IMX_MMC_BASE + 0x1000) {
		return 1;
	}
	return 0;
}

