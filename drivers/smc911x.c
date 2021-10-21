/*
 * smc911x.c
 * This is a driver for SMSC's LAN911{5,6,7,8} single-chip Ethernet devices.
 *
 * Copyright (C) 2005 Sensoria Corp
 *	   Derived from the unified SMC91x driver by Nicolas Pitre
 *	   and the smsc911x.c reference driver by SMSC
 * Copyright (C) 2006 EmbeddedAlley
 *	   Ported to u-boot from Linux kernel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Arguments:
 *	 watchdog  = TX watchdog timeout
 *	 tx_fifo_kb = Size of TX FIFO in KB
 *
 * History:
 *	  04/16/05	Dustin McIntire		 Initial version
 */
#include <config.h>

#ifdef CONFIG_DRIVER_SMC911X

static const char version[] =
	 "smc911x.c: v1.0 04-16-2005 by Dustin McIntire <dustin@sensoria.com>\n";

/* Debugging options */
#define ENABLE_SMC_DEBUG_RX		0
#define ENABLE_SMC_DEBUG_TX		0
#define ENABLE_SMC_DEBUG_DMA		0
#define ENABLE_SMC_DEBUG_PKTS		0
#define ENABLE_SMC_DEBUG_MISC		0
#define ENABLE_SMC_DEBUG_FUNC		0

#define SMC_DEBUG_RX		((ENABLE_SMC_DEBUG_RX	? 1 : 0) << 0)
#define SMC_DEBUG_TX		((ENABLE_SMC_DEBUG_TX	? 1 : 0) << 1)
#define SMC_DEBUG_DMA		((ENABLE_SMC_DEBUG_DMA	? 1 : 0) << 2)
#define SMC_DEBUG_PKTS		((ENABLE_SMC_DEBUG_PKTS ? 1 : 0) << 3)
#define SMC_DEBUG_MISC		((ENABLE_SMC_DEBUG_MISC ? 1 : 0) << 4)
#define SMC_DEBUG_FUNC		((ENABLE_SMC_DEBUG_FUNC ? 1 : 0) << 5)

#ifndef SMC_DEBUG
#define SMC_DEBUG	 ( SMC_DEBUG_RX	  | \
			   SMC_DEBUG_TX	  | \
			   SMC_DEBUG_DMA  | \
			   SMC_DEBUG_PKTS | \
			   SMC_DEBUG_MISC | \
			   SMC_DEBUG_FUNC   \
			 )
#endif

#define SMC_TX_TIMEOUT 30
#define ETH_ZLEN 60

#include <common.h>
#include <command.h>
#include "smc911x.h"
#include <net.h>

/*
 * The internal workings of the driver.  If you are changing anything
 * here with the SMC stuff, you should have the datasheet and know
 * what you are doing.
 */
#define CARDNAME "smc911x"

#ifndef CONFIG_SMC911X_BASE
#error "define base IO addr"
#define CONFIG_SMC911X_BASE 0x20000300
#endif

/*
 * Use power-down feature of the chip
 */
#define POWER_DOWN		 1


#if SMC_DEBUG > 0
#define DBG(n, args...)				 \
	do {					 \
		if (SMC_DEBUG & (n))		 \
			printf(args);		 \
	} while (0)

#define PRINTK(args...)   printf(args)
#else
#define DBG(n, args...)   do { } while (0)
#define PRINTK(args...)   printf(args)
#endif

static void smc911x_tx(void);
int smc911x_get_ethaddr (bd_t * bd);

#if SMC_DEBUG_PKTS > 0
static void PRINT_PKT(u_char *buf, int length)
{
	int i;
	int remainder;
	int lines;

	lines = length / 16;
	remainder = length % 16;

	for (i = 0; i < lines ; i ++) {
		int cur;
		for (cur = 0; cur < 8; cur++) {
			u_char a, b;
			a = *buf++;
			b = *buf++;
			printf("%02x%02x ", a, b);
		}
		printf("\n");
	}
	for (i = 0; i < remainder/2 ; i++) {
		u_char a, b;
		a = *buf++;
		b = *buf++;
		printf("%02x%02x ", a, b);
	}
	printf("\n");
}
#else
#define PRINT_PKT(x...)  do { } while (0)
#endif


/* this enables an interrupt in the interrupt mask register */
#define SMC_ENABLE_INT(x) do {				\
	unsigned int  __mask;				\
	unsigned long __flags;				\
	spin_lock_irqsave(&lp->lock, __flags);		\
	__mask = SMC_GET_INT_EN();			\
	__mask |= (x);					\
	SMC_SET_INT_EN(__mask);				\
	spin_unlock_irqrestore(&lp->lock, __flags);	\
} while (0)

/* this disables an interrupt from the interrupt mask register */
#define SMC_DISABLE_INT(x) do {				\
	unsigned int  __mask;				\
	unsigned long __flags;				\
	spin_lock_irqsave(&lp->lock, __flags);		\
	__mask = SMC_GET_INT_EN();			\
	__mask &= ~(x);					\
	SMC_SET_INT_EN(__mask);				\
	spin_unlock_irqrestore(&lp->lock, __flags);	\
} while (0)

void smc911x_set_mac_addr(const char *addr)
{
	unsigned long ioaddr = CONFIG_SMC911X_BASE;
	DBG(SMC_DEBUG_FUNC, "setting mac address to "
		"%02x:%02x:%02x:%02x:%02x:%02x\n",
		addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	SMC_SET_MAC_ADDR(addr);
}

/*
 * this does a soft reset on the device
 */
static void smc911x_reset(bd_t *bd)
{
	unsigned long ioaddr = CONFIG_SMC911X_BASE;
	unsigned int reg, timeout=0, resets=1;

	/*	 Take out of PM setting first */
	if ((SMC_GET_PMT_CTRL() & PMT_CTRL_READY_) == 0) {
		/* Write to the bytetest will take out of powerdown */
		SMC_SET_BYTE_TEST(0);
		timeout=10;
		do {
			udelay(10);
			reg = SMC_GET_PMT_CTRL() & PMT_CTRL_READY_;
		} while ( timeout-- && !reg);
		if (timeout == 0) {
			return;
		}
	}

	/* Disable all interrupts */
	SMC_SET_INT_EN(0);

	while (resets--) {
		SMC_SET_HW_CFG(HW_CFG_SRST_);
		timeout=10;
		do {
			udelay(10);
			reg = SMC_GET_HW_CFG();
			/* If chip indicates reset timeout then try again */
			if (reg & HW_CFG_SRST_TO_) {
				resets++;
				break;
			}
		} while ( timeout-- && (reg & HW_CFG_SRST_));
	}
	if (timeout == 0) {
		return;
	}

	/* make sure EEPROM has finished loading before setting GPIO_CFG */
	timeout=1000;
	while ( timeout-- && (SMC_GET_E2P_CMD() & E2P_CMD_EPC_BUSY_)) {
		udelay(10);
	}
	if (timeout == 0){
		return;
	}

	/* Initialize interrupts */
	SMC_SET_INT_EN(0);
	SMC_ACK_INT(-1);

	SMC_SET_FLOW(FLOW_FCPT_ | FLOW_FCEN_);

	/* Set to LED outputs */
	SMC_SET_GPIO_CFG(0x70070000);

	udelay(10);

	/* MAC address is lost on reset */
	smc911x_set_mac_addr((char *)bd->bi_enetaddr);
	udelay(10);
}

/*
 * Enable Interrupts, Receive, and Transmit
 */
static void smc911x_enable(void)
{
	unsigned long ioaddr = CONFIG_SMC911X_BASE;
	unsigned cfg, cr;

	DBG(SMC_DEBUG_FUNC, "--> %s\n",  __FUNCTION__);

	/* Enable TX */
	cfg = SMC_GET_HW_CFG();
	DBG(SMC_DEBUG_FUNC, "%s: got cfg 0x%x\n", __FUNCTION__, cfg);
	cfg &= HW_CFG_TX_FIF_SZ_ | 0xFFF;
	cfg |= HW_CFG_SF_;
	SMC_SET_HW_CFG(cfg);
	SMC_SET_FIFO_TDA(1);
	/* Update TX stats on every 64 packets received or every 1 sec */
	SMC_SET_FIFO_TSL(64);
	SMC_GET_MAC_CR(cr);
	cr |= MAC_CR_TXEN_ | MAC_CR_HBDIS_;
	SMC_SET_MAC_CR(cr);
	SMC_SET_TX_CFG(TX_CFG_TX_ON_);

	/* Turn on receiver and enable RX */
	if (cr & MAC_CR_RXEN_)
		DBG(SMC_DEBUG_RX, "%s: Receiver already enabled\n", __FUNCTION__);

	SMC_SET_MAC_CR( cr | MAC_CR_RXEN_ );

	/* Interrupt on every received packet */
	SMC_SET_FIFO_RSA(0x01);
	SMC_SET_FIFO_RSL(0x00);
}

/*
 * this puts the device in an inactive state
 */
static void smc911x_shutdown(void)
{
	unsigned long ioaddr = CONFIG_SMC911X_BASE;
	unsigned cr;

	/* Disable IRQ's */
	SMC_SET_INT_EN(0);

	/* Turn of Rx and TX */
	SMC_GET_MAC_CR(cr);
	cr &= ~(MAC_CR_TXEN_ | MAC_CR_RXEN_ | MAC_CR_HBDIS_);
	SMC_SET_MAC_CR(cr);
	SMC_SET_TX_CFG(TX_CFG_STOP_TX_);
}

static int poll4int (int mask, int timeout)
{
	int tmo = get_timer (0) + timeout * CFG_HZ;
	unsigned long ioaddr = CONFIG_SMC911X_BASE;
	int is_timeout = 0;

	DBG(SMC_DEBUG_FUNC, "Polling for 0x%x, currenty 0x%lx\n",
		mask, SMC_GET_INT());
	while ((SMC_GET_INT() & mask) == 0) {
		if (get_timer (0) >= tmo) {
			is_timeout = 1;
			break;
		}
	}

	DBG(SMC_DEBUG_FUNC, "Poll %s\n", is_timeout ? "timed out" : "successful");

	if (is_timeout)
		return 1;
	else
		return 0;
}


/*
 * This is the procedure to handle the receipt of a packet.
 * It should be called after checking for packet presence in
 * the RX status FIFO.	 It must be called with the spin lock
 * already held.
 */
static int smc911x_receive(void)
{
	unsigned long ioaddr = CONFIG_SMC911X_BASE;
	unsigned int pkt_len = 0, status;

	status = SMC_GET_INT();

	if (!(status & INT_STS_RSFL_))
		return 0;

	status = SMC_GET_RX_STS_FIFO();
	pkt_len = (status & RX_STS_PKT_LEN_) >> 16;
	if (!(status & RX_STS_ES_) && pkt_len) {
		/* Receive a valid packet */
		DBG(SMC_DEBUG_RX, "received valid packet length %d\n", pkt_len);
		SMC_PULL_DATA(NetRxPackets[0], pkt_len+2+3);
		PRINT_PKT(NetRxPackets[0], (pkt_len-4) <= 64 ? (pkt_len-4) : 64);
		NetReceive(NetRxPackets[0], pkt_len-4); /* XXX? */
	}
	if (SMC_GET_INT() & INT_STS_RXSTOP_INT_)
		SMC_ACK_INT(INT_STS_RXSTOP_INT_);
	if (SMC_GET_INT() & INT_STS_RXE_)
		SMC_ACK_INT(INT_STS_RXE_);
	SMC_ACK_INT(INT_STS_RSFL_);
	return pkt_len;
}

/*
 * This is called to actually send a packet to the chip.
 */
static int smc911x_send_packet(volatile void *buf, int length)
{
	unsigned long ioaddr = CONFIG_SMC911X_BASE;
	unsigned int cmdA, cmdB;
	int len = length;

	SMC_SET_TX_CFG(TX_CFG_TX_ON_);
	/* cmdA {25:24] data alignment [20:16] start offset [10:0] buffer length */
	/* cmdB {31:16] pkt tag [10:0] length */
	len = (len + 3 + ((u32)buf & 3)) & ~0x3;
	len = ETH_ZLEN < len ? len : ETH_ZLEN;
	cmdA = (((u32)buf & 0x3) << 16) |
			TX_CMD_A_INT_FIRST_SEG_ | TX_CMD_A_INT_LAST_SEG_ |
			len;
	/* tag is packet length so we can use this in stats update later */
	cmdB = (len  << 16) | (len & 0x7FF);

	SMC_SET_TX_FIFO(cmdA);
	SMC_SET_TX_FIFO(cmdB);

	DBG(SMC_DEBUG_TX, "%s: Transmitted packet\n", __FUNCTION__);
	PRINT_PKT(buf, len <= 64 ? len : 64);

	/* Send pkt via PIO or DMA */
	SMC_PUSH_DATA(buf, len);

	/* poll for int */
	if (poll4int(INT_STS_TDFA_, SMC_TX_TIMEOUT))
		DBG(SMC_DEBUG_TX, "TX timeout\n");
	else
		DBG(SMC_DEBUG_TX, "TX success\n");

	smc911x_tx();
	SMC_ACK_INT(INT_STS_TSFL_);

	return len < length ? len : length;
}

/*
 * This handles a TX status interrupt, which is only called when:
 * - a TX error occurred, or
 * - TX of a packet completed.
 */
static void smc911x_tx(void)
{
	unsigned long ioaddr = CONFIG_SMC911X_BASE;
	unsigned int tx_status;

	/* Collect the TX status */
	while (((SMC_GET_TX_FIFO_INF() & TX_FIFO_INF_TSUSED_) >> 16) != 0) {
		DBG(SMC_DEBUG_TX, "%s: Tx stat FIFO used 0x%04x\n",
			__FUNCTION__,
			(SMC_GET_TX_FIFO_INF() & TX_FIFO_INF_TSUSED_) >> 16);
		tx_status = SMC_GET_TX_STS_FIFO();
		DBG(SMC_DEBUG_TX, "%s: Tx FIFO tag 0x%04x status 0x%04x\n",
			__FUNCTION__, (tx_status & 0xffff0000) >> 16,
			tx_status & 0x0000ffff);
	}
}

/*
 * Open and Initialize the board
 *
 * Set up everything, reset the card, etc..
 */
static int
smc911x_open(bd_t *bd)
{
	/* reset the hardware */
	smc911x_reset(bd);
	udelay(100);

	if (smc911x_get_ethaddr(bd) < 0) {
		PRINTK("%s: error getting ethaddr\n", __FUNCTION__);
		memset (bd->bi_enetaddr, 0, 6);
		return (-1);
	}

	/* Turn on Tx + Rx */
	smc911x_enable();

	return 0;
}

/*
 * smc911x_close
 *
 * this makes the board clean up everything that it can
 * and not talk to the outside world.	 Caused by
 * an 'ifconfig ethX down'
 */
static int smc911x_close(void)
{
	/* clear everything */
	smc911x_shutdown();

	return 0;
}

int eth_init(bd_t *bd)
{
	return (smc911x_open(bd));
}

void eth_halt()
{
	smc911x_close();
}

int eth_rx()
{
	return smc911x_receive();
}

int eth_send(volatile void *packet, int length) {
	return smc911x_send_packet(packet, length);
}

int get_rom_mac (char *v_rom_mac)
{
#ifdef HARDCODE_MAC	/* used for testing or to supress run time warnings */
	char hw_mac_addr[] = { 0x02, 0x80, 0xad, 0x20, 0x31, 0xb8 };

	memcpy (v_rom_mac, hw_mac_addr, 6);
	return (1);
#else
	unsigned long ioaddr = CONFIG_SMC911X_BASE;

	SMC_GET_MAC_ADDR(v_rom_mac);
	DBG(SMC_DEBUG_FUNC, "%s: %02x %02x %02x %02x %02x %02x\n", __FUNCTION__,
		v_rom_mac[0], v_rom_mac[1], v_rom_mac[2], v_rom_mac[3],
		v_rom_mac[4], v_rom_mac[5]);

	return 1;

#endif
}


int smc911x_get_ethaddr (bd_t * bd)
{
	int env_size, rom_valid, env_present = 0, reg;
	char *s = NULL, *e, *v_mac, es[] = "11:22:33:44:55:66";
	char s_env_mac[64], v_env_mac[6], v_rom_mac[6];

	env_size = getenv_r ("ethaddr", s_env_mac, sizeof (s_env_mac));
	if ((env_size > 0) && (env_size < sizeof (es))) {	/* exit if env is bad */
		PRINTK ("\n*** ERROR: ethaddr is not set properly!!\n");
		return (-1);
	}

	if (env_size > 0) {
		env_present = 1;
		s = s_env_mac;
	}

	for (reg = 0; reg < 6; ++reg) { /* turn string into mac value */
		v_env_mac[reg] = s ? simple_strtoul (s, &e, 16) : 0;
		if (s)
			s = (*e) ? e + 1 : e;
	}

	rom_valid = get_rom_mac (v_rom_mac);	/* get ROM mac value if any */

	if (!env_present) {	/* if NO env */
		if (rom_valid) {	/* but ROM is valid */
			v_mac = v_rom_mac;
			sprintf (s_env_mac, "%02X:%02X:%02X:%02X:%02X:%02X",
				 v_mac[0], v_mac[1], v_mac[2], v_mac[3],
				 v_mac[4], v_mac[5]);
			setenv ("ethaddr", s_env_mac);
		} else {	/* no env, bad ROM */
			PRINTK ("\n*** ERROR: ethaddr is NOT set !!\n");
			return (-1);
		}
	} else {		/* good env, don't care ROM */
		v_mac = v_env_mac;	/* always use a good env over a ROM */
	}

	if (env_present && rom_valid) { /* if both env and ROM are good */
		if (memcmp (v_env_mac, v_rom_mac, 6) != 0) {
			PRINTK ("\nWarning: MAC addresses don't match:\n");
			PRINTK ("\tHW MAC address:  "
				"%02X:%02X:%02X:%02X:%02X:%02X\n",
				v_rom_mac[0], v_rom_mac[1],
				v_rom_mac[2], v_rom_mac[3],
				v_rom_mac[4], v_rom_mac[5] );
			PRINTK ("\t\"ethaddr\" value: "
				"%02X:%02X:%02X:%02X:%02X:%02X\n",
				v_env_mac[0], v_env_mac[1],
				v_env_mac[2], v_env_mac[3],
				v_env_mac[4], v_env_mac[5]) ;
			debug ("### Set MAC addr from environment\n");
		}
	}
	memcpy (bd->bi_enetaddr, v_mac, 6);	/* update global address to match env (allows env changing) */
	smc911x_set_mac_addr (v_mac);	/* use old function to update smc default */
	DBG(SMC_DEBUG_FUNC, "Using MAC Address "
		"%02X:%02X:%02X:%02X:%02X:%02X\n", v_mac[0], v_mac[1],
		v_mac[2], v_mac[3], v_mac[4], v_mac[5]);
	return (0);
}

#endif /* CONFIG_DRIVER_SMC911X */

