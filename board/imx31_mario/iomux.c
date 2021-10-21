/*
 * IOMUX. 
 *
 * (C) 2007. Embedded Alley Solutions, Inc. 
 *
 * based on Freescale BSP for Linux 
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

#ifdef DEBUG
#define dprintf printf
#else
#define dprintf(...) 
#endif

#define PIN_TO_IOMUX_FIELD(pin) 	((pin >> MUX_F) & ((1 << (PAD_I - MUX_F)) - 1))
#define PIN_TO_IOMUX_INDEX(pin) 	((pin >> MUX_I) & ((1 << (MUX_F - MUX_I)) - 1))
#define GET_FIELD_MASK(len, fld)    	(((1 << len) - 1) << (len * fld))
#define MUX_CTL_BIT_LEN         	8
#define MUX_CTL_FIELDS          	4
#define MUX_PAD_BIT_LEN         	10


#define AIPS1_BASE_ADDR		0x43F00000
#define AIPS2_BASE_ADDR         0x53F00000
#define IOMUXC_BASE_ADDR        (AIPS1_BASE_ADDR + 0x000AC000)
#define GPIO3_BASE_ADDR         (AIPS2_BASE_ADDR + 0x000A4000)
#define GPIO1_BASE_ADDR         (AIPS2_BASE_ADDR + 0x000CC000)
#define GPIO2_BASE_ADDR         (AIPS2_BASE_ADDR + 0x000D0000)

#define IOMUXSW_MUX_CTL 	((IOMUXC_BASE_ADDR) + 0x00C)
#define IOMUXSW_PAD_CTL 	((IOMUXC_BASE_ADDR) + 0x154)
#define IOMUXSW_MUX_END 	((IOMUXC_BASE_ADDR) + 0x150)

#define MUX_PIN_NUM_MAX \
        (((u32 *)IOMUXSW_MUX_END - (u32 *)IOMUXSW_MUX_CTL + 1) * MUX_CTL_FIELDS)
#define GPIO_PORT_NUM           3
#define IOMUX_TO_GPIO(pin)      ((((unsigned int)pin >> MUX_IO_P) * GPIO_NUM_PIN) + ((pin >> MUX_IO_I) & ((1 << (MUX_IO_P - MUX_IO_I)) -1)))
#define GPIO_NUM_PIN            32
#define GPIO_TO_PORT(n)         (n / GPIO_NUM_PIN)
#define GPIO_TO_INDEX(n) 	(n % GPIO_NUM_PIN)

static u8 iomux_pin_res_table[MUX_PIN_NUM_MAX];

enum gpio_reg {
        GPIO_DR = 0x00,
        GPIO_GDIR = 0x04,
        GPIO_PSR = 0x08,
        GPIO_ICR1 = 0x0C,
        GPIO_ICR2 = 0x10,
        GPIO_IMR = 0x14,
        GPIO_ISR = 0x18,
};

struct gpio_port {
        u32 num;                /*!< gpio port number */
        u32 base;               /*!< gpio port base VA */
        u32 reserved_map;       /*!< keep track of which pins are in use */
};

static struct gpio_port gpio_port[GPIO_PORT_NUM] = {
        { 0, GPIO1_BASE_ADDR, },
	{ 1, GPIO2_BASE_ADDR, },
	{ 2, GPIO3_BASE_ADDR, },
};

/*
 * Find the pointer to the gpio_port for a given pin.
 * @param gpio          a gpio pin number
 * @return              pointer to \b struc \b gpio_port
 */
static inline struct gpio_port *get_gpio_port(u32 gpio)
{
        return &gpio_port[GPIO_TO_PORT(gpio)];
}


static inline u32 fsl_readl(u32 r)
{
	u32 v = __REG(r);

	dprintf("RD [%08x] => %08x\n", r, v);
	return v;
}

static inline void fsl_writel(u32 v, u32 r)
{
	dprintf("WR [%08x] <= %08x\n", r, v);
	__REG(r) = v;
}

int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
                     iomux_pin_icfg_t in)
{
        u32 reg, l;
        u32 mux_index = PIN_TO_IOMUX_INDEX(pin);
        u32 mux_field = PIN_TO_IOMUX_FIELD(pin);
        u32 mux_mask = GET_FIELD_MASK(MUX_CTL_BIT_LEN, mux_field);
        u8 *rp;

        reg = IOMUXSW_MUX_CTL + (mux_index * 4);

        l = fsl_readl(reg);
        l = (l & (~mux_mask)) |
            (((out << 4) | in) << (mux_field * MUX_CTL_BIT_LEN));
        fsl_writel(l, reg);

        rp = iomux_pin_res_table + mux_index * MUX_CTL_FIELDS + mux_field;
        *rp = (out << 4) | in;

	dprintf("%s done\n", __FUNCTION__);

	return 0;
}

int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
                      iomux_pin_icfg_t in)
{
	dprintf("%s(%x,%x,%x)\n", __FUNCTION__, pin, out, in);
        return  iomux_config_mux(pin, out, in);
}

void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
                    iomux_pin_icfg_t in)
{
        u32 mux_index = PIN_TO_IOMUX_INDEX(pin);
        u32 mux_field = PIN_TO_IOMUX_FIELD(pin);
        u8 *rp = iomux_pin_res_table + mux_index * MUX_CTL_FIELDS + mux_field;

        *rp = 0;
}


void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config)
{
        u32 reg, l;
        u32 pad_index = (pin >> PAD_I) & ((1 << (PAD_F - PAD_I)) - 1);
        u32 pad_field = (pin >> PAD_F) & ((1 << (MUX_IO_I - PAD_F)) - 1);
        u32 pad_mask = GET_FIELD_MASK(MUX_PAD_BIT_LEN, pad_field);

	dprintf("%s(%x,%x)\n", __FUNCTION__, pin, config);
        reg = IOMUXSW_PAD_CTL + (pad_index * 4);
        l = fsl_readl(reg);
        l = (l & (~pad_mask)) | (config << (pad_field * MUX_PAD_BIT_LEN));
        fsl_writel(l, reg);
	dprintf("%s done\n", __FUNCTION__);
}

static void _set_gpio_dataout(struct gpio_port *port, u32 index, u32 data)
{
        u32 reg = port->base + GPIO_DR;
        u32 l = 0;

        l = (fsl_readl(reg) & (~(1 << index))) | (data << index);
        fsl_writel(l, reg);
	dprintf("%s done\n", __FUNCTION__);
}

/*!
 * Exported function to set a GPIO pin's data output
 * @param pin           a name defined by \b iomux_pin_name_t
 * @param data          value to be set (only 0 or 1 is valid)
 */
void mxc_set_gpio_dataout(iomux_pin_name_t pin, u32 data)
{
        u32 gpio = IOMUX_TO_GPIO(pin);
        _set_gpio_dataout(get_gpio_port(gpio), GPIO_TO_INDEX(gpio), (data == 0) ? 0 : 1);
}

/*
 * Set a GPIO pin's direction
 * @param port          pointer to a gpio_port
 * @param index         gpio pin index value (0~31)
 * @param is_input      0 for output; non-zero for input
 */
static void _set_gpio_direction(struct gpio_port *port, u32 index, int is_input)
{
        u32 reg = port->base + GPIO_GDIR;
        u32 l;

        l = fsl_readl(reg);
        if (is_input)
                l &= ~(1 << index);
        else
                l |= 1 << index;
        fsl_writel(l, reg);
}

/*!
 * Exported function to set a GPIO pin's direction
 * @param pin           a name defined by \b iomux_pin_name_t
 * @param is_input      1 (or non-zero) for input; 0 for output
 */
void mxc_set_gpio_direction(iomux_pin_name_t pin, int is_input)
{
        u32 gpio = IOMUX_TO_GPIO(pin);
        _set_gpio_direction(get_gpio_port(gpio), GPIO_TO_INDEX(gpio), is_input);
}


