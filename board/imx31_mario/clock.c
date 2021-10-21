/*
 * 
 * (C) 2007. Embedded Alley Solutions, Inc. 
 * support@embeddedalley.com 
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
#include <div64.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mario.h>

#define AIPS2_BASE_ADDR		   0x53F00000
#define	MXC_CCM_BASE 		   (AIPS2_BASE_ADDR + 0x00080000)
#define MXC_CCM_CGR0               (MXC_CCM_BASE + 0x20)
#define MXC_CCM_CGR1               (MXC_CCM_BASE + 0x24)
#define MXC_CCM_CGR2	   	   (MXC_CCM_BASE + 0x28)
#define MXC_CCM_CGR0_GPT_MASK                   (0x3 << 4)
#define MXC_CCM_CGR2_EMI_MASK                   (0x3 << 8)
#define MXC_CCM_CGR2_IPMUX1_MASK                (0x3 << 14)
#define MXC_CCM_CGR2_IPMUX2_MASK                (0x3 << 16)
#define MXC_CCM_CGR2_CSPI1_OFFSET               2
#define MXC_CCM_CGR2_CSPI2_OFFSET               4
#define MXC_CCM_CGR1_IPU_OFFSET			22
#define MXC_CCM_CGR0_CSPI3_OFFSET               16
#define MXC_CCM_PDR0               (MXC_CCM_BASE + 0x04)
#define MXC_CCM_PDR0_CSI_PODF_OFFSET            26
#define MXC_CCM_PDR0_CSI_PODF_MASK              (0x3F << 26)
#define MXC_CCM_PDR0_CSI_PRDF_OFFSET            23
#define MXC_CCM_PDR0_CSI_PRDF_MASK              (0x7 << 23)
#define MXC_CCM_PDR0_PER_PODF_OFFSET            16
#define MXC_CCM_PDR0_PER_PODF_MASK              (0x1F << 16)
#define MXC_CCM_PDR0_HSP_PODF_OFFSET            11
#define MXC_CCM_PDR0_HSP_PODF_MASK              (0x7 << 11)
#define MXC_CCM_PDR0_NFC_PODF_OFFSET            8
#define MXC_CCM_PDR0_NFC_PODF_MASK              (0x7 << 8)
#define MXC_CCM_PDR0_IPG_PODF_OFFSET            6
#define MXC_CCM_PDR0_IPG_PODF_MASK              (0x3 << 6)
#define MXC_CCM_PDR0_MAX_PODF_OFFSET            3
#define MXC_CCM_PDR0_MAX_PODF_MASK              (0x7 << 3)
#define MXC_CCM_PDR0_MCU_PODF_OFFSET            0
#define MXC_CCM_PDR0_MCU_PODF_MASK              0x7
#define MXC_CCM_CCMR               (MXC_CCM_BASE + 0x00)
#define MXC_CCM_CCMR_WBEN                       (1 << 27)
#define MXC_CCM_CCMR_CSCS                       (1 << 25)
#define MXC_CCM_CCMR_PERCS                      (1 << 24)
#define MXC_CCM_CCMR_SSI1S_OFFSET               18
#define MXC_CCM_CCMR_SSI1S_MASK                 (0x3 << 18)
#define MXC_CCM_CCMR_SSI2S_OFFSET               21
#define MXC_CCM_CCMR_SSI2S_MASK                 (0x3 << 21)
#define MXC_CCM_CCMR_LPM_OFFSET                 14
#define MXC_CCM_CCMR_LPM_MASK                   (0x3 << 14)
#define MXC_CCM_CCMR_FIRS_OFFSET                11
#define MXC_CCM_CCMR_FIRS_MASK                  (0x3 << 11)
#define MXC_CCM_CCMR_UPE                        (1 << 9)
#define MXC_CCM_CCMR_SPE                        (1 << 8)
#define MXC_CCM_CCMR_MDS                        (1 << 7)
#define MXC_CCM_CCMR_SBYCS                      (1 << 4)
#define MXC_CCM_CCMR_MPE                        (1 << 3)
#define MXC_CCM_CCMR_PRCS_OFFSET                1
#define MXC_CCM_CCMR_PRCS_MASK                  (0x3 << 1)
#define MXC_CCM_MPCTL              (MXC_CCM_BASE + 0x10)
#define MXC_CCM_MPCTL              (MXC_CCM_BASE + 0x10)
#define MXC_CCM_UPCTL              (MXC_CCM_BASE + 0x14)
#define MXC_CCM_SRPCTL             (MXC_CCM_BASE + 0x18)
#define MXC_CCM_PCTL_BRM                        0x80000000
#define MXC_CCM_PCTL_PD_OFFSET                  26
#define MXC_CCM_PCTL_PD_MASK                    (0xF << 26)
#define MXC_CCM_PCTL_MFD_OFFSET                 16
#define MXC_CCM_PCTL_MFD_MASK                   (0x3FF << 16)
#define MXC_CCM_PCTL_MFI_OFFSET                 10
#define MXC_CCM_PCTL_MFI_MASK                   (0xF << 10)
#define MXC_CCM_PCTL_MFN_OFFSET                 0
#define MXC_CCM_PCTL_MFN_MASK                   0x3FF
#define CKIH_CLK_FREQ           26000000
#define CKIH_CLK_FREQ_27MHZ     27000000
#define CKIL_CLK_FREQ           32768

#define PDR0(mask, off) ((__REG(MXC_CCM_PDR0) & mask) >> off)
#define PDR1(mask, off) ((__REG(MXC_CCM_PDR1) & mask) >> off)
#define PDR2(mask, off) ((__REG(MXC_CCM_PDR2) & mask) >> off)

int clk_ckih_rate(void)
{
	printf("%s: %d\n", __FUNCTION__, CKIH_CLK_FREQ);
	return CKIH_CLK_FREQ;
}

int clk_mcu_main_rate(void)
{
	u32 ccmr, prcs, reg, pdf, mfd, mfi, mfn, mfn_abs;
	int ref_clk;
	u64 temp;

        ccmr = __REG(MXC_CCM_CCMR);
        prcs = (ccmr & MXC_CCM_CCMR_PRCS_MASK) >> MXC_CCM_CCMR_PRCS_OFFSET;
        if (prcs == 0x1) {
                ref_clk = CKIL_CLK_FREQ * 1024;
        } else {
                ref_clk = clk_ckih_rate();
        }

        if ((ccmr & MXC_CCM_CCMR_MPE) == 0) {
		printf("%s: %d\n", __FUNCTION__, ref_clk);
                return ref_clk;
	}
        if ((ccmr & MXC_CCM_CCMR_MDS) != 0) {
		printf("%s: %d\n", __FUNCTION__, ref_clk);
		return ref_clk;
	}

        reg = __REG(MXC_CCM_MPCTL);

        pdf = (reg & MXC_CCM_PCTL_PD_MASK) >> MXC_CCM_PCTL_PD_OFFSET;
        mfd = (reg & MXC_CCM_PCTL_MFD_MASK) >> MXC_CCM_PCTL_MFD_OFFSET;
        mfi = (reg & MXC_CCM_PCTL_MFI_MASK) >> MXC_CCM_PCTL_MFI_OFFSET;
        mfi = (mfi <= 5) ? 5 : mfi;
        mfn = mfn_abs = reg & MXC_CCM_PCTL_MFN_MASK;

        if (mfn >= 0x200) {
                mfn |= 0xFFFFFE00;
                mfn_abs = -mfn;
        }

        ref_clk *= 2;
        ref_clk /= pdf + 1;

        temp = (u64)ref_clk *mfn_abs;
        do_div(temp, mfd + 1);
        if (mfn < 0)
                temp = -temp;

	return ref_clk * mfi + temp;
}

int clk_ahb_rate(void)
{
        unsigned long max_pdf;
	int r = clk_mcu_main_rate();

        max_pdf = PDR0(MXC_CCM_PDR0_MAX_PODF_MASK,
                       MXC_CCM_PDR0_MAX_PODF_OFFSET);
        return r / (max_pdf + 1);
}

int clk_ipg_rate(void)
{
        unsigned long ipg_pdf;
	int r = clk_ahb_rate();

        ipg_pdf = PDR0(MXC_CCM_PDR0_IPG_PODF_MASK,
                       MXC_CCM_PDR0_IPG_PODF_OFFSET);
	return r/(ipg_pdf + 1);
}

void clk_spi_enable(void)
{
	u32 reg;

#if SPI_DEBUG > 1
	printf("%s, entered\n", __FUNCTION__);
#endif
//	__REG(MXC_CCM_CGR) = MXC_CCM_CGR0_GPT_MASK;
//	__REG(MXC_CCM_CGR0, 0);
	printf("CGR0 = %x\n", __REG(MXC_CCM_CGR0));
	__REG(MXC_CCM_CGR2) =  MXC_CCM_CGR2_EMI_MASK |
                     MXC_CCM_CGR2_IPMUX1_MASK |
                     MXC_CCM_CGR2_IPMUX2_MASK;

	reg = __REG(MXC_CCM_CGR1);
	reg |= (3<<MXC_CCM_CGR1_IPU_OFFSET);
	__REG(MXC_CCM_CGR1) =  reg;

        reg = __REG(MXC_CCM_CGR0);
        reg |= (3 << MXC_CCM_CGR0_CSPI3_OFFSET);
        __REG(MXC_CCM_CGR0) = reg;

        reg = __REG(MXC_CCM_CGR2);
	reg |= ((3 << MXC_CCM_CGR2_CSPI1_OFFSET) |
	        (3 << MXC_CCM_CGR2_CSPI2_OFFSET));
        __REG(MXC_CCM_CGR2) = reg;
}

int spba_take_ownership(int mod, int master)
{
	u32 check;

        __REG(SPBA_CTRL_BASE_ADDR + mod) = master;
	check = __REG(SPBA_CTRL_BASE_ADDR + mod);
	printf("%s: checked %x & %x = %x (== %x)\n", __FUNCTION__,
		check, MXC_SPBA_RAR_MASK, check & MXC_SPBA_RAR_MASK, master);
        return check & MXC_SPBA_RAR_MASK == master;
}

