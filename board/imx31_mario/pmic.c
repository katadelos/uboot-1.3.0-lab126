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
#include <command.h>
#include "spi.h"
#include "spi_defs.h"
#include "pmic_external.h"
#include "pmic.h"

/* #define PMIC_DEBUG */

#define PMIC_SUCCESS 	0
#define PMIC_ERROR	-1

#define EVENT_MASK_0                    0x697fdf
#define EVENT_MASK_1                    0x3efffb
#define MXC_PMIC_FRAME_MASK             0x00FFFFFF
#define MXC_PMIC_MAX_REG_NUM            0x3F
#define MXC_PMIC_REG_NUM_SHIFT          0x19
#define MXC_PMIC_WRITE_BIT_SHIFT        31

typedef enum {
        VMMC1_1_6V = 0,         /*!< 1.60 V */
        VMMC1_1_8V,             /*!< 1.80 V */
        VMMC1_2V,               /*!< 2.00 V */
        VMMC1_2_6V,             /*!< 2.60 V */
        VMMC1_2_7V,             /*!< 2.70 V */
        VMMC1_2_8V,             /*!< 2.80 V */
        VMMC1_2_9V,             /*!< 2.90 V */
        VMMC1_3V,               /*!< 3.00 V */
} t_pmic_regulator_voltage_vmmc1;

typedef enum {
        VMMC2_1_6V = 0,         /*!< 1.60 V */
        VMMC2_1_8V,             /*!< 1.80 V */
        VMMC2_2V,               /*!< 2.00 V */
        VMMC2_2_6V,             /*!< 2.60 V */
        VMMC2_2_7V,             /*!< 2.70 V */
        VMMC2_2_8V,             /*!< 2.80 V */
        VMMC2_2_9V,             /*!< 2.90 V */
        VMMC2_3V,               /*!< 3.00 V */
} t_pmic_regulator_voltage_vmmc2;

/*
 * Bitfield macros that use rely on bitfield width/shift information.
 */
#define BITFMASK(field) (((1U << (field ## _WID)) - 1) << (field ## _LSH))
#define BITFVAL(field, val) ((val) << (field ## _LSH))
#define BITFEXT(var, bit) ((var & BITFMASK(bit)) >> (bit ## _LSH))

#define CHECK_ERROR(code) do { int r; r = (code); /* printf("%s returned %d\n", #code, r); */ } while (0)

#define PMIC_DEBUG
/*!
 * This function is called to write a value to the register on PMIC.
 *
 * @param        reg_num     number of the pmic register to be written
 * @param        reg_val   value to be written
 *
 * @return       Returns 0 on success -1 on failure.
 */
static int pmic_write(int reg_num, const unsigned int reg_val)
{
        u32 frame = 0;

        if (reg_num > MXC_PMIC_MAX_REG_NUM)
                return PMIC_ERROR;

        frame |= (1 << MXC_PMIC_WRITE_BIT_SHIFT);

        frame |= reg_num << MXC_PMIC_REG_NUM_SHIFT;

        frame |= reg_val & MXC_PMIC_FRAME_MASK;

#ifdef PMIC_DEBUG
	printf("%s: sending %X\n", __FUNCTION__, frame);
#endif
        return spi_xfer(0, sizeof(u32), (u8 *) &frame, NULL);
}

/*!
 * This function is called to read a register on PMIC.
 *
 * @param        reg_num     number of the pmic register to be read
 * @param        reg_val   return value of register
 *
 * @return       Returns 0 on success -1 on failure.
 */
static int pmic_read(unsigned int reg_num, unsigned int *reg_val)
{
        u32 frame = 0;
	u32 rcvd;
        int ret = 0;

        if (reg_num > MXC_PMIC_MAX_REG_NUM)
                return PMIC_ERROR;

        frame |= reg_num << MXC_PMIC_REG_NUM_SHIFT;
#ifdef PMIC_DEBUG
	printf("%s: sending %X\n", __FUNCTION__, frame);
#endif
        ret = spi_xfer(0, sizeof(u32), (u8*)&frame, (u8*)&rcvd);
#ifdef PMIC_DEBUG
	printf("%s: received %X\n", __FUNCTION__, rcvd);
#endif

        *reg_val = rcvd & MXC_PMIC_FRAME_MASK;

        return ret;
}

/*!
 * This function initializes the PMIC registers.
 *
 * @return   None
 */
static int pmic_init_registers(void)
{
        CHECK_ERROR(pmic_write(REG_INTERRUPT_MASK_0, MXC_PMIC_FRAME_MASK));
        CHECK_ERROR(pmic_write(REG_INTERRUPT_MASK_1, MXC_PMIC_FRAME_MASK));
        CHECK_ERROR(pmic_write(REG_INTERRUPT_STATUS_0, MXC_PMIC_FRAME_MASK));
        CHECK_ERROR(pmic_write(REG_INTERRUPT_STATUS_1, MXC_PMIC_FRAME_MASK));
        return PMIC_SUCCESS;
}

/*!
 * This function returns the PMIC version in system.
 *
 * @param       ver     pointer to the pmic_version_t structure
 *
 * @return       This function returns PMIC version.
 */

static void pmic_get_revision(u32 * ver)
{
        unsigned rev_id = 0;
        int rev1 = 0;
        int rev2 = 0;
        int finid = 0;
        int icid = 0;

        pmic_read(REG_REVISION, &rev_id);
#ifdef PMIC_DEBUG
	printf("READ: %x\n", rev_id);
#endif

        rev1 = (rev_id & 0x018) >> 3;
        rev2 = (rev_id & 0x007);
        icid = (rev_id & 0x01C0) >> 6;
        finid = (rev_id & 0x01E00) >> 9;

        /* Ver 0.2 is actually 3.2a.  Report as 3.2 */
        if ((rev1 == 0) && (rev2 == 2)) {
                rev1 = 3;
        }

        if (rev1 == 0 || icid != 2) {
				if(ver) 
					*ver = -1;
                printf("mc13783: Not detected.\tAccess failed\t!!!\n");
                return;
        } else {
				if(ver) 
					*ver = ((rev1 * 10) + rev2);
                printf( "mc13783 Rev %d.%d FinVer %x detected\n", rev1,
                       rev2, finid);
        }

        return;
}

/*!
 * This function is called by PMIC clients to write a register on PMIC.
 *
 * @param        reg        number of register
 * @param        reg_value  New value of register
 * @param        reg_mask   Bitmap mask indicating which bits to modify
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
static int pmic_write_reg(int reg, u32 reg_value,
                           u32 reg_mask)
{
        u32 temp = 0;

        if (!pmic_read(reg, &temp))
		return -1;
        temp = (temp & (~reg_mask)) | reg_value;
	return pmic_write(reg, temp);
}

static int pmic_power_regulator_set_voltage_mmc(u32 voltage)
{
	u32 reg, reg_mask, reg_val;
        reg_val = BITFVAL(MC13783_REGSET_VMMC1, voltage);
        reg_mask = BITFMASK(MC13783_REGSET_VMMC1);
        reg = REG_REGULATOR_SETTING_1;
	return pmic_write_reg(reg, reg_val, reg_mask);
}

static int pmic_power_regulator_set_lp_mode_mmc(void)
{
	u32 reg, reg_mask, reg_val;
	u32 l_mode, l_stby;

        l_mode = MC13783_REGTRL_LP_MODE_DISABLE;
        l_stby = MC13783_REGTRL_STBY_MODE_DISABLE;
        reg_val = BITFVAL(MC13783_REGCTRL_VMMC1_MODE, l_mode) |
                    BITFVAL(MC13783_REGCTRL_VMMC1_STBY, l_stby);
        reg_mask = BITFMASK(MC13783_REGCTRL_VMMC1_MODE) |
                    BITFMASK(MC13783_REGCTRL_VMMC1_STBY);
        reg = REG_REGULATOR_MODE_1;
	return pmic_write_reg(reg, reg_val, reg_mask);
}

static int pmic_power_regulator_on_mmc(void)
{
	u32 reg, reg_mask, reg_val;

        reg_val = BITFVAL(MC13783_REGCTRL_VMMC1_EN,
                                 MC13783_REGCTRL_VMMC1_EN_ENABLE);
        reg_mask = BITFMASK(MC13783_REGCTRL_VMMC1_EN);
        reg = REG_REGULATOR_MODE_1;

	return pmic_write_reg(reg, reg_val, reg_mask);
}

int pmic_init(void)
{
	u32 r;

	spi_init();
	pmic_init_registers();
	pmic_get_revision(&r);
	pmic_power_regulator_set_voltage_mmc(VMMC1_2_8V);
	pmic_power_regulator_set_lp_mode_mmc();
	pmic_power_regulator_on_mmc();
	return 0;
}
