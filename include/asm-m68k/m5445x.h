/*
 * MCF5445x Internal Memory Map
 *
 * Copyright (C) 2004-2007 Freescale Semiconductor, Inc.
 * TsiChung Liew (Tsi-Chung.Liew@freescale.com)
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

#ifndef __MCF5445X__
#define __MCF5445X__

/*********************************************************************
* Cross-bar switch (XBS)
*********************************************************************/

/* Bit definitions and macros for PRS group */
#define XBS_PRS_M0(x)			(((x)&0x00000007))	/* Core */
#define XBS_PRS_M1(x)			(((x)&0x00000007)<<4)	/* eDMA */
#define XBS_PRS_M2(x)			(((x)&0x00000007)<<8)	/* FEC0 */
#define XBS_PRS_M3(x)			(((x)&0x00000007)<<12)	/* FEC1 */
#define XBS_PRS_M5(x)			(((x)&0x00000007)<<20)	/* PCI controller */
#define XBS_PRS_M6(x)			(((x)&0x00000007)<<24)	/* USB OTG */
#define XBS_PRS_M7(x)			(((x)&0x00000007)<<28)	/* Serial Boot */

/* Bit definitions and macros for CRS group */
#define XBS_CRS_PARK(x)			(((x)&0x00000007))	/* Master parking ctrl */
#define XBS_CRS_PCTL(x)			(((x)&0x00000003)<<4)	/* Parking mode ctrl */
#define XBS_CRS_ARB			(0x00000100)	/* Arbitration Mode */
#define XBS_CRS_RO			(0x80000000)	/* Read Only */

#define XBS_CRS_PCTL_PARK_FIELD		(0)
#define XBS_CRS_PCTL_PARK_ON_LAST	(1)
#define XBS_CRS_PCTL_PARK_NONE		(2)
#define XBS_CRS_PCTL_PARK_CORE		(0)
#define XBS_CRS_PCTL_PARK_EDMA		(1)
#define XBS_CRS_PCTL_PARK_FEC0		(2)
#define XBS_CRS_PCTL_PARK_FEC1		(3)
#define XBS_CRS_PCTL_PARK_PCI		(5)
#define XBS_CRS_PCTL_PARK_USB		(6)
#define XBS_CRS_PCTL_PARK_SBF		(7)

/*********************************************************************
* FlexBus Chip Selects (FBCS)
*********************************************************************/

/* Bit definitions and macros for CSAR group */
#define FBCS_CSAR_BA(x)			((x)&0xFFFF0000)

/* Bit definitions and macros for CSMR group */
#define FBCS_CSMR_V			(0x00000001)	/* Valid bit */
#define FBCS_CSMR_WP			(0x00000100)	/* Write protect */
#define FBCS_CSMR_BAM(x)		(((x)&0x0000FFFF)<<16)	/* Base address mask */
#define FBCS_CSMR_BAM_4G		(0xFFFF0000)
#define FBCS_CSMR_BAM_2G		(0x7FFF0000)
#define FBCS_CSMR_BAM_1G		(0x3FFF0000)
#define FBCS_CSMR_BAM_1024M		(0x3FFF0000)
#define FBCS_CSMR_BAM_512M		(0x1FFF0000)
#define FBCS_CSMR_BAM_256M		(0x0FFF0000)
#define FBCS_CSMR_BAM_128M		(0x07FF0000)
#define FBCS_CSMR_BAM_64M		(0x03FF0000)
#define FBCS_CSMR_BAM_32M		(0x01FF0000)
#define FBCS_CSMR_BAM_16M		(0x00FF0000)
#define FBCS_CSMR_BAM_8M		(0x007F0000)
#define FBCS_CSMR_BAM_4M		(0x003F0000)
#define FBCS_CSMR_BAM_2M		(0x001F0000)
#define FBCS_CSMR_BAM_1M		(0x000F0000)
#define FBCS_CSMR_BAM_1024K		(0x000F0000)
#define FBCS_CSMR_BAM_512K		(0x00070000)
#define FBCS_CSMR_BAM_256K		(0x00030000)
#define FBCS_CSMR_BAM_128K		(0x00010000)
#define FBCS_CSMR_BAM_64K		(0x00000000)

/* Bit definitions and macros for CSCR group */
#define FBCS_CSCR_BSTW			(0x00000008)	/* Burst-write enable */
#define FBCS_CSCR_BSTR			(0x00000010)	/* Burst-read enable */
#define FBCS_CSCR_BEM			(0x00000020)	/* Byte-enable mode */
#define FBCS_CSCR_PS(x)			(((x)&0x00000003)<<6)	/* Port size */
#define FBCS_CSCR_AA			(0x00000100)	/* Auto-acknowledge */
#define FBCS_CSCR_WS(x)			(((x)&0x0000003F)<<10)	/* Wait states */
#define FBCS_CSCR_WRAH(x)		(((x)&0x00000003)<<16)	/* Write address hold or deselect */
#define FBCS_CSCR_RDAH(x)		(((x)&0x00000003)<<18)	/* Read address hold or deselect */
#define FBCS_CSCR_ASET(x)		(((x)&0x00000003)<<20)	/* Address setup */
#define FBCS_CSCR_SWSEN			(0x00800000)	/* Secondary wait state enable */
#define FBCS_CSCR_SWS(x)		(((x)&0x0000003F)<<26)	/* Secondary wait states */

#define FBCS_CSCR_PS_8			(0x00000040)
#define FBCS_CSCR_PS_16			(0x00000080)
#define FBCS_CSCR_PS_32			(0x00000000)

/*********************************************************************
* Interrupt Controller (INTC)
*********************************************************************/
#define INT0_LO_RSVD0			(0)
#define INT0_LO_EPORT1			(1)
#define INT0_LO_EPORT2			(2)
#define INT0_LO_EPORT3			(3)
#define INT0_LO_EPORT4			(4)
#define INT0_LO_EPORT5			(5)
#define INT0_LO_EPORT6			(6)
#define INT0_LO_EPORT7			(7)
#define INT0_LO_EDMA_00			(8)
#define INT0_LO_EDMA_01			(9)
#define INT0_LO_EDMA_02			(10)
#define INT0_LO_EDMA_03			(11)
#define INT0_LO_EDMA_04			(12)
#define INT0_LO_EDMA_05			(13)
#define INT0_LO_EDMA_06			(14)
#define INT0_LO_EDMA_07			(15)
#define INT0_LO_EDMA_08			(16)
#define INT0_LO_EDMA_09			(17)
#define INT0_LO_EDMA_10			(18)
#define INT0_LO_EDMA_11			(19)
#define INT0_LO_EDMA_12			(20)
#define INT0_LO_EDMA_13			(21)
#define INT0_LO_EDMA_14			(22)
#define INT0_LO_EDMA_15			(23)
#define INT0_LO_EDMA_ERR		(24)
#define INT0_LO_SCM			(25)
#define INT0_LO_UART0			(26)
#define INT0_LO_UART1			(27)
#define INT0_LO_UART2			(28)
#define INT0_LO_RSVD1			(29)
#define INT0_LO_I2C			(30)
#define INT0_LO_QSPI			(31)
#define INT0_HI_DTMR0			(32)
#define INT0_HI_DTMR1			(33)
#define INT0_HI_DTMR2			(34)
#define INT0_HI_DTMR3			(35)
#define INT0_HI_FEC0_TXF		(36)
#define INT0_HI_FEC0_TXB		(37)
#define INT0_HI_FEC0_UN			(38)
#define INT0_HI_FEC0_RL			(39)
#define INT0_HI_FEC0_RXF		(40)
#define INT0_HI_FEC0_RXB		(41)
#define INT0_HI_FEC0_MII		(42)
#define INT0_HI_FEC0_LC			(43)
#define INT0_HI_FEC0_HBERR		(44)
#define INT0_HI_FEC0_GRA		(45)
#define INT0_HI_FEC0_EBERR		(46)
#define INT0_HI_FEC0_BABT		(47)
#define INT0_HI_FEC0_BABR		(48)
#define INT0_HI_FEC1_TXF		(49)
#define INT0_HI_FEC1_TXB		(50)
#define INT0_HI_FEC1_UN			(51)
#define INT0_HI_FEC1_RL			(52)
#define INT0_HI_FEC1_RXF		(53)
#define INT0_HI_FEC1_RXB		(54)
#define INT0_HI_FEC1_MII		(55)
#define INT0_HI_FEC1_LC			(56)
#define INT0_HI_FEC1_HBERR		(57)
#define INT0_HI_FEC1_GRA		(58)
#define INT0_HI_FEC1_EBERR		(59)
#define INT0_HI_FEC1_BABT		(60)
#define INT0_HI_FEC1_BABR		(61)
#define INT0_HI_SCMIR			(62)
#define INT0_HI_RTC_ISR			(63)

#define INT1_HI_DSPI_EOQF		(33)
#define INT1_HI_DSPI_TFFF		(34)
#define INT1_HI_DSPI_TCF		(35)
#define INT1_HI_DSPI_TFUF		(36)
#define INT1_HI_DSPI_RFDF		(37)
#define INT1_HI_DSPI_RFOF		(38)
#define INT1_HI_DSPI_RFOF_TFUF		(39)
#define INT1_HI_RNG_EI			(40)
#define INT1_HI_PIT0_PIF		(43)
#define INT1_HI_PIT1_PIF		(44)
#define INT1_HI_PIT2_PIF		(45)
#define INT1_HI_PIT3_PIF		(46)
#define INT1_HI_USBOTG_USBSTS		(47)
#define INT1_HI_SSI_ISR			(49)
#define INT1_HI_CCM_UOCSR		(53)
#define INT1_HI_ATA_ISR			(54)
#define INT1_HI_PCI_SCR			(55)
#define INT1_HI_PCI_ASR			(56)
#define INT1_HI_PLL_LOCKS		(57)

/* Bit definitions and macros for IPRH */
#define INTC_IPRH_INT32			(0x00000001)
#define INTC_IPRH_INT33			(0x00000002)
#define INTC_IPRH_INT34			(0x00000004)
#define INTC_IPRH_INT35			(0x00000008)
#define INTC_IPRH_INT36			(0x00000010)
#define INTC_IPRH_INT37			(0x00000020)
#define INTC_IPRH_INT38			(0x00000040)
#define INTC_IPRH_INT39			(0x00000080)
#define INTC_IPRH_INT40			(0x00000100)
#define INTC_IPRH_INT41			(0x00000200)
#define INTC_IPRH_INT42			(0x00000400)
#define INTC_IPRH_INT43			(0x00000800)
#define INTC_IPRH_INT44			(0x00001000)
#define INTC_IPRH_INT45			(0x00002000)
#define INTC_IPRH_INT46			(0x00004000)
#define INTC_IPRH_INT47			(0x00008000)
#define INTC_IPRH_INT48			(0x00010000)
#define INTC_IPRH_INT49			(0x00020000)
#define INTC_IPRH_INT50			(0x00040000)
#define INTC_IPRH_INT51			(0x00080000)
#define INTC_IPRH_INT52			(0x00100000)
#define INTC_IPRH_INT53			(0x00200000)
#define INTC_IPRH_INT54			(0x00400000)
#define INTC_IPRH_INT55			(0x00800000)
#define INTC_IPRH_INT56			(0x01000000)
#define INTC_IPRH_INT57			(0x02000000)
#define INTC_IPRH_INT58			(0x04000000)
#define INTC_IPRH_INT59			(0x08000000)
#define INTC_IPRH_INT60			(0x10000000)
#define INTC_IPRH_INT61			(0x20000000)
#define INTC_IPRH_INT62			(0x40000000)
#define INTC_IPRH_INT63			(0x80000000)

/* Bit definitions and macros for IPRL */
#define INTC_IPRL_INT0			(0x00000001)
#define INTC_IPRL_INT1			(0x00000002)
#define INTC_IPRL_INT2			(0x00000004)
#define INTC_IPRL_INT3			(0x00000008)
#define INTC_IPRL_INT4			(0x00000010)
#define INTC_IPRL_INT5			(0x00000020)
#define INTC_IPRL_INT6			(0x00000040)
#define INTC_IPRL_INT7			(0x00000080)
#define INTC_IPRL_INT8			(0x00000100)
#define INTC_IPRL_INT9			(0x00000200)
#define INTC_IPRL_INT10			(0x00000400)
#define INTC_IPRL_INT11			(0x00000800)
#define INTC_IPRL_INT12			(0x00001000)
#define INTC_IPRL_INT13			(0x00002000)
#define INTC_IPRL_INT14			(0x00004000)
#define INTC_IPRL_INT15			(0x00008000)
#define INTC_IPRL_INT16			(0x00010000)
#define INTC_IPRL_INT17			(0x00020000)
#define INTC_IPRL_INT18			(0x00040000)
#define INTC_IPRL_INT19			(0x00080000)
#define INTC_IPRL_INT20			(0x00100000)
#define INTC_IPRL_INT21			(0x00200000)
#define INTC_IPRL_INT22			(0x00400000)
#define INTC_IPRL_INT23			(0x00800000)
#define INTC_IPRL_INT24			(0x01000000)
#define INTC_IPRL_INT25			(0x02000000)
#define INTC_IPRL_INT26			(0x04000000)
#define INTC_IPRL_INT27			(0x08000000)
#define INTC_IPRL_INT28			(0x10000000)
#define INTC_IPRL_INT29			(0x20000000)
#define INTC_IPRL_INT30			(0x40000000)
#define INTC_IPRL_INT31			(0x80000000)

/* Bit definitions and macros for IMRH */
#define INTC_IMRH_INT_MASK32		(0x00000001)
#define INTC_IMRH_INT_MASK33		(0x00000002)
#define INTC_IMRH_INT_MASK34		(0x00000004)
#define INTC_IMRH_INT_MASK35		(0x00000008)
#define INTC_IMRH_INT_MASK36		(0x00000010)
#define INTC_IMRH_INT_MASK37		(0x00000020)
#define INTC_IMRH_INT_MASK38		(0x00000040)
#define INTC_IMRH_INT_MASK39		(0x00000080)
#define INTC_IMRH_INT_MASK40		(0x00000100)
#define INTC_IMRH_INT_MASK41		(0x00000200)
#define INTC_IMRH_INT_MASK42		(0x00000400)
#define INTC_IMRH_INT_MASK43		(0x00000800)
#define INTC_IMRH_INT_MASK44		(0x00001000)
#define INTC_IMRH_INT_MASK45		(0x00002000)
#define INTC_IMRH_INT_MASK46		(0x00004000)
#define INTC_IMRH_INT_MASK47		(0x00008000)
#define INTC_IMRH_INT_MASK48		(0x00010000)
#define INTC_IMRH_INT_MASK49		(0x00020000)
#define INTC_IMRH_INT_MASK50		(0x00040000)
#define INTC_IMRH_INT_MASK51		(0x00080000)
#define INTC_IMRH_INT_MASK52		(0x00100000)
#define INTC_IMRH_INT_MASK53		(0x00200000)
#define INTC_IMRH_INT_MASK54		(0x00400000)
#define INTC_IMRH_INT_MASK55		(0x00800000)
#define INTC_IMRH_INT_MASK56		(0x01000000)
#define INTC_IMRH_INT_MASK57		(0x02000000)
#define INTC_IMRH_INT_MASK58		(0x04000000)
#define INTC_IMRH_INT_MASK59		(0x08000000)
#define INTC_IMRH_INT_MASK60		(0x10000000)
#define INTC_IMRH_INT_MASK61		(0x20000000)
#define INTC_IMRH_INT_MASK62		(0x40000000)
#define INTC_IMRH_INT_MASK63		(0x80000000)

/* Bit definitions and macros for IMRL */
#define INTC_IMRL_INT_MASK0		(0x00000001)
#define INTC_IMRL_INT_MASK1		(0x00000002)
#define INTC_IMRL_INT_MASK2		(0x00000004)
#define INTC_IMRL_INT_MASK3		(0x00000008)
#define INTC_IMRL_INT_MASK4		(0x00000010)
#define INTC_IMRL_INT_MASK5		(0x00000020)
#define INTC_IMRL_INT_MASK6		(0x00000040)
#define INTC_IMRL_INT_MASK7		(0x00000080)
#define INTC_IMRL_INT_MASK8		(0x00000100)
#define INTC_IMRL_INT_MASK9		(0x00000200)
#define INTC_IMRL_INT_MASK10		(0x00000400)
#define INTC_IMRL_INT_MASK11		(0x00000800)
#define INTC_IMRL_INT_MASK12		(0x00001000)
#define INTC_IMRL_INT_MASK13		(0x00002000)
#define INTC_IMRL_INT_MASK14		(0x00004000)
#define INTC_IMRL_INT_MASK15		(0x00008000)
#define INTC_IMRL_INT_MASK16		(0x00010000)
#define INTC_IMRL_INT_MASK17		(0x00020000)
#define INTC_IMRL_INT_MASK18		(0x00040000)
#define INTC_IMRL_INT_MASK19		(0x00080000)
#define INTC_IMRL_INT_MASK20		(0x00100000)
#define INTC_IMRL_INT_MASK21		(0x00200000)
#define INTC_IMRL_INT_MASK22		(0x00400000)
#define INTC_IMRL_INT_MASK23		(0x00800000)
#define INTC_IMRL_INT_MASK24		(0x01000000)
#define INTC_IMRL_INT_MASK25		(0x02000000)
#define INTC_IMRL_INT_MASK26		(0x04000000)
#define INTC_IMRL_INT_MASK27		(0x08000000)
#define INTC_IMRL_INT_MASK28		(0x10000000)
#define INTC_IMRL_INT_MASK29		(0x20000000)
#define INTC_IMRL_INT_MASK30		(0x40000000)
#define INTC_IMRL_INT_MASK31		(0x80000000)

/* Bit definitions and macros for INTFRCH */
#define INTC_INTFRCH_INTFRC32		(0x00000001)
#define INTC_INTFRCH_INTFRC33		(0x00000002)
#define INTC_INTFRCH_INTFRC34		(0x00000004)
#define INTC_INTFRCH_INTFRC35		(0x00000008)
#define INTC_INTFRCH_INTFRC36		(0x00000010)
#define INTC_INTFRCH_INTFRC37		(0x00000020)
#define INTC_INTFRCH_INTFRC38		(0x00000040)
#define INTC_INTFRCH_INTFRC39		(0x00000080)
#define INTC_INTFRCH_INTFRC40		(0x00000100)
#define INTC_INTFRCH_INTFRC41		(0x00000200)
#define INTC_INTFRCH_INTFRC42		(0x00000400)
#define INTC_INTFRCH_INTFRC43		(0x00000800)
#define INTC_INTFRCH_INTFRC44		(0x00001000)
#define INTC_INTFRCH_INTFRC45		(0x00002000)
#define INTC_INTFRCH_INTFRC46		(0x00004000)
#define INTC_INTFRCH_INTFRC47		(0x00008000)
#define INTC_INTFRCH_INTFRC48		(0x00010000)
#define INTC_INTFRCH_INTFRC49		(0x00020000)
#define INTC_INTFRCH_INTFRC50		(0x00040000)
#define INTC_INTFRCH_INTFRC51		(0x00080000)
#define INTC_INTFRCH_INTFRC52		(0x00100000)
#define INTC_INTFRCH_INTFRC53		(0x00200000)
#define INTC_INTFRCH_INTFRC54		(0x00400000)
#define INTC_INTFRCH_INTFRC55		(0x00800000)
#define INTC_INTFRCH_INTFRC56		(0x01000000)
#define INTC_INTFRCH_INTFRC57		(0x02000000)
#define INTC_INTFRCH_INTFRC58		(0x04000000)
#define INTC_INTFRCH_INTFRC59		(0x08000000)
#define INTC_INTFRCH_INTFRC60		(0x10000000)
#define INTC_INTFRCH_INTFRC61		(0x20000000)
#define INTC_INTFRCH_INTFRC62		(0x40000000)
#define INTC_INTFRCH_INTFRC63		(0x80000000)

/* Bit definitions and macros for INTFRCL */
#define INTC_INTFRCL_INTFRC0		(0x00000001)
#define INTC_INTFRCL_INTFRC1		(0x00000002)
#define INTC_INTFRCL_INTFRC2		(0x00000004)
#define INTC_INTFRCL_INTFRC3		(0x00000008)
#define INTC_INTFRCL_INTFRC4		(0x00000010)
#define INTC_INTFRCL_INTFRC5		(0x00000020)
#define INTC_INTFRCL_INTFRC6		(0x00000040)
#define INTC_INTFRCL_INTFRC7		(0x00000080)
#define INTC_INTFRCL_INTFRC8		(0x00000100)
#define INTC_INTFRCL_INTFRC9		(0x00000200)
#define INTC_INTFRCL_INTFRC10		(0x00000400)
#define INTC_INTFRCL_INTFRC11		(0x00000800)
#define INTC_INTFRCL_INTFRC12		(0x00001000)
#define INTC_INTFRCL_INTFRC13		(0x00002000)
#define INTC_INTFRCL_INTFRC14		(0x00004000)
#define INTC_INTFRCL_INTFRC15		(0x00008000)
#define INTC_INTFRCL_INTFRC16		(0x00010000)
#define INTC_INTFRCL_INTFRC17		(0x00020000)
#define INTC_INTFRCL_INTFRC18		(0x00040000)
#define INTC_INTFRCL_INTFRC19		(0x00080000)
#define INTC_INTFRCL_INTFRC20		(0x00100000)
#define INTC_INTFRCL_INTFRC21		(0x00200000)
#define INTC_INTFRCL_INTFRC22		(0x00400000)
#define INTC_INTFRCL_INTFRC23		(0x00800000)
#define INTC_INTFRCL_INTFRC24		(0x01000000)
#define INTC_INTFRCL_INTFRC25		(0x02000000)
#define INTC_INTFRCL_INTFRC26		(0x04000000)
#define INTC_INTFRCL_INTFRC27		(0x08000000)
#define INTC_INTFRCL_INTFRC28		(0x10000000)
#define INTC_INTFRCL_INTFRC29		(0x20000000)
#define INTC_INTFRCL_INTFRC30		(0x40000000)
#define INTC_INTFRCL_INTFRC31		(0x80000000)

/* Bit definitions and macros for ICONFIG */
#define INTC_ICONFIG_EMASK		(0x0020)
#define INTC_ICONFIG_ELVLPRI1		(0x0200)
#define INTC_ICONFIG_ELVLPRI2		(0x0400)
#define INTC_ICONFIG_ELVLPRI3		(0x0800)
#define INTC_ICONFIG_ELVLPRI4		(0x1000)
#define INTC_ICONFIG_ELVLPRI5		(0x2000)
#define INTC_ICONFIG_ELVLPRI6		(0x4000)
#define INTC_ICONFIG_ELVLPRI7		(0x8000)

/* Bit definitions and macros for SIMR */
#define INTC_SIMR_SIMR(x)		(((x)&0x7F))

/* Bit definitions and macros for CIMR */
#define INTC_CIMR_CIMR(x)		(((x)&0x7F))

/* Bit definitions and macros for CLMASK */
#define INTC_CLMASK_CLMASK(x)		(((x)&0x0F))

/* Bit definitions and macros for SLMASK */
#define INTC_SLMASK_SLMASK(x)		(((x)&0x0F))

/* Bit definitions and macros for ICR group */
#define INTC_ICR_IL(x)			(((x)&0x07))

/*********************************************************************
* DMA Serial Peripheral Interface (DSPI)
*********************************************************************/

/* Bit definitions and macros for DMCR */
#define DSPI_DMCR_HALT			(0x00000001)
#define DSPI_DMCR_SMPL_PT(x)		(((x)&0x00000003)<<8)
#define DSPI_DMCR_CRXF			(0x00000400)
#define DSPI_DMCR_CTXF			(0x00000800)
#define DSPI_DMCR_DRXF			(0x00001000)
#define DSPI_DMCR_DTXF			(0x00002000)
#define DSPI_DMCR_CSIS0			(0x00010000)
#define DSPI_DMCR_CSIS2			(0x00040000)
#define DSPI_DMCR_CSIS3			(0x00080000)
#define DSPI_DMCR_CSIS5			(0x00200000)
#define DSPI_DMCR_ROOE			(0x01000000)
#define DSPI_DMCR_PCSSE			(0x02000000)
#define DSPI_DMCR_MTFE			(0x04000000)
#define DSPI_DMCR_FRZ			(0x08000000)
#define DSPI_DMCR_DCONF(x)		(((x)&0x00000003)<<28)
#define DSPI_DMCR_CSCK			(0x40000000)
#define DSPI_DMCR_MSTR			(0x80000000)

/* Bit definitions and macros for DTCR */
#define DSPI_DTCR_SPI_TCNT(x)		(((x)&0x0000FFFF)<<16)

/* Bit definitions and macros for DCTAR group */
#define DSPI_DCTAR_BR(x)		(((x)&0x0000000F))
#define DSPI_DCTAR_DT(x)		(((x)&0x0000000F)<<4)
#define DSPI_DCTAR_ASC(x)		(((x)&0x0000000F)<<8)
#define DSPI_DCTAR_CSSCK(x)		(((x)&0x0000000F)<<12)
#define DSPI_DCTAR_PBR(x)		(((x)&0x00000003)<<16)
#define DSPI_DCTAR_PDT(x)		(((x)&0x00000003)<<18)
#define DSPI_DCTAR_PASC(x)		(((x)&0x00000003)<<20)
#define DSPI_DCTAR_PCSSCK(x)		(((x)&0x00000003)<<22)
#define DSPI_DCTAR_LSBFE		(0x01000000)
#define DSPI_DCTAR_CPHA			(0x02000000)
#define DSPI_DCTAR_CPOL			(0x04000000)
#define DSPI_DCTAR_TRSZ(x)		(((x)&0x0000000F)<<27)
#define DSPI_DCTAR_PCSSCK_1CLK		(0x00000000)
#define DSPI_DCTAR_PCSSCK_3CLK		(0x00400000)
#define DSPI_DCTAR_PCSSCK_5CLK		(0x00800000)
#define DSPI_DCTAR_PCSSCK_7CLK		(0x00A00000)
#define DSPI_DCTAR_PASC_1CLK		(0x00000000)
#define DSPI_DCTAR_PASC_3CLK		(0x00100000)
#define DSPI_DCTAR_PASC_5CLK		(0x00200000)
#define DSPI_DCTAR_PASC_7CLK		(0x00300000)
#define DSPI_DCTAR_PDT_1CLK		(0x00000000)
#define DSPI_DCTAR_PDT_3CLK		(0x00040000)
#define DSPI_DCTAR_PDT_5CLK		(0x00080000)
#define DSPI_DCTAR_PDT_7CLK		(0x000A0000)
#define DSPI_DCTAR_PBR_1CLK		(0x00000000)
#define DSPI_DCTAR_PBR_3CLK		(0x00010000)
#define DSPI_DCTAR_PBR_5CLK		(0x00020000)
#define DSPI_DCTAR_PBR_7CLK		(0x00030000)

/* Bit definitions and macros for DSR */
#define DSPI_DSR_RXPTR(x)		(((x)&0x0000000F))
#define DSPI_DSR_RXCTR(x)		(((x)&0x0000000F)<<4)
#define DSPI_DSR_TXPTR(x)		(((x)&0x0000000F)<<8)
#define DSPI_DSR_TXCTR(x)		(((x)&0x0000000F)<<12)
#define DSPI_DSR_RFDF			(0x00020000)
#define DSPI_DSR_RFOF			(0x00080000)
#define DSPI_DSR_TFFF			(0x02000000)
#define DSPI_DSR_TFUF			(0x08000000)
#define DSPI_DSR_EOQF			(0x10000000)
#define DSPI_DSR_TXRXS			(0x40000000)
#define DSPI_DSR_TCF			(0x80000000)

/* Bit definitions and macros for DIRSR */
#define DSPI_DIRSR_RFDFS		(0x00010000)
#define DSPI_DIRSR_RFDFE		(0x00020000)
#define DSPI_DIRSR_RFOFE		(0x00080000)
#define DSPI_DIRSR_TFFFS		(0x01000000)
#define DSPI_DIRSR_TFFFE		(0x02000000)
#define DSPI_DIRSR_TFUFE		(0x08000000)
#define DSPI_DIRSR_EOQFE		(0x10000000)
#define DSPI_DIRSR_TCFE			(0x80000000)

/* Bit definitions and macros for DTFR */
#define DSPI_DTFR_TXDATA(x)		(((x)&0x0000FFFF))
#define DSPI_DTFR_CS0			(0x00010000)
#define DSPI_DTFR_CS2			(0x00040000)
#define DSPI_DTFR_CS3			(0x00080000)
#define DSPI_DTFR_CS5			(0x00200000)
#define DSPI_DTFR_CTCNT			(0x04000000)
#define DSPI_DTFR_EOQ			(0x08000000)
#define DSPI_DTFR_CTAS(x)		(((x)&0x00000007)<<28)
#define DSPI_DTFR_CONT			(0x80000000)

/* Bit definitions and macros for DRFR */
#define DSPI_DRFR_RXDATA(x)		(((x)&0x0000FFFF))

/* Bit definitions and macros for DTFDR group */
#define DSPI_DTFDR_TXDATA(x)		(((x)&0x0000FFFF))
#define DSPI_DTFDR_TXCMD(x)		(((x)&0x0000FFFF)<<16)

/* Bit definitions and macros for DRFDR group */
#define DSPI_DRFDR_RXDATA(x)		(((x)&0x0000FFFF))

/*********************************************************************
* Edge Port Module (EPORT)
*********************************************************************/

/* Bit definitions and macros for EPPAR */
#define EPORT_EPPAR_EPPA1(x)		(((x)&0x0003)<<2)
#define EPORT_EPPAR_EPPA2(x)		(((x)&0x0003)<<4)
#define EPORT_EPPAR_EPPA3(x)		(((x)&0x0003)<<6)
#define EPORT_EPPAR_EPPA4(x)		(((x)&0x0003)<<8)
#define EPORT_EPPAR_EPPA5(x)		(((x)&0x0003)<<10)
#define EPORT_EPPAR_EPPA6(x)		(((x)&0x0003)<<12)
#define EPORT_EPPAR_EPPA7(x)		(((x)&0x0003)<<14)
#define EPORT_EPPAR_LEVEL		(0)
#define EPORT_EPPAR_RISING		(1)
#define EPORT_EPPAR_FALLING		(2)
#define EPORT_EPPAR_BOTH		(3)
#define EPORT_EPPAR_EPPA7_LEVEL		(0x0000)
#define EPORT_EPPAR_EPPA7_RISING	(0x4000)
#define EPORT_EPPAR_EPPA7_FALLING	(0x8000)
#define EPORT_EPPAR_EPPA7_BOTH		(0xC000)
#define EPORT_EPPAR_EPPA6_LEVEL		(0x0000)
#define EPORT_EPPAR_EPPA6_RISING	(0x1000)
#define EPORT_EPPAR_EPPA6_FALLING	(0x2000)
#define EPORT_EPPAR_EPPA6_BOTH		(0x3000)
#define EPORT_EPPAR_EPPA5_LEVEL		(0x0000)
#define EPORT_EPPAR_EPPA5_RISING	(0x0400)
#define EPORT_EPPAR_EPPA5_FALLING	(0x0800)
#define EPORT_EPPAR_EPPA5_BOTH		(0x0C00)
#define EPORT_EPPAR_EPPA4_LEVEL		(0x0000)
#define EPORT_EPPAR_EPPA4_RISING	(0x0100)
#define EPORT_EPPAR_EPPA4_FALLING	(0x0200)
#define EPORT_EPPAR_EPPA4_BOTH		(0x0300)
#define EPORT_EPPAR_EPPA3_LEVEL		(0x0000)
#define EPORT_EPPAR_EPPA3_RISING	(0x0040)
#define EPORT_EPPAR_EPPA3_FALLING	(0x0080)
#define EPORT_EPPAR_EPPA3_BOTH		(0x00C0)
#define EPORT_EPPAR_EPPA2_LEVEL		(0x0000)
#define EPORT_EPPAR_EPPA2_RISING	(0x0010)
#define EPORT_EPPAR_EPPA2_FALLING	(0x0020)
#define EPORT_EPPAR_EPPA2_BOTH		(0x0030)
#define EPORT_EPPAR_EPPA1_LEVEL		(0x0000)
#define EPORT_EPPAR_EPPA1_RISING	(0x0004)
#define EPORT_EPPAR_EPPA1_FALLING	(0x0008)
#define EPORT_EPPAR_EPPA1_BOTH		(0x000C)

/* Bit definitions and macros for EPDDR */
#define EPORT_EPDDR_EPDD1		(0x02)
#define EPORT_EPDDR_EPDD2		(0x04)
#define EPORT_EPDDR_EPDD3		(0x08)
#define EPORT_EPDDR_EPDD4		(0x10)
#define EPORT_EPDDR_EPDD5		(0x20)
#define EPORT_EPDDR_EPDD6		(0x40)
#define EPORT_EPDDR_EPDD7		(0x80)

/* Bit definitions and macros for EPIER */
#define EPORT_EPIER_EPIE1		(0x02)
#define EPORT_EPIER_EPIE2		(0x04)
#define EPORT_EPIER_EPIE3		(0x08)
#define EPORT_EPIER_EPIE4		(0x10)
#define EPORT_EPIER_EPIE5		(0x20)
#define EPORT_EPIER_EPIE6		(0x40)
#define EPORT_EPIER_EPIE7		(0x80)

/* Bit definitions and macros for EPDR */
#define EPORT_EPDR_EPD1			(0x02)
#define EPORT_EPDR_EPD2			(0x04)
#define EPORT_EPDR_EPD3			(0x08)
#define EPORT_EPDR_EPD4			(0x10)
#define EPORT_EPDR_EPD5			(0x20)
#define EPORT_EPDR_EPD6			(0x40)
#define EPORT_EPDR_EPD7			(0x80)

/* Bit definitions and macros for EPPDR */
#define EPORT_EPPDR_EPPD1		(0x02)
#define EPORT_EPPDR_EPPD2		(0x04)
#define EPORT_EPPDR_EPPD3		(0x08)
#define EPORT_EPPDR_EPPD4		(0x10)
#define EPORT_EPPDR_EPPD5		(0x20)
#define EPORT_EPPDR_EPPD6		(0x40)
#define EPORT_EPPDR_EPPD7		(0x80)

/* Bit definitions and macros for EPFR */
#define EPORT_EPFR_EPF1			(0x02)
#define EPORT_EPFR_EPF2			(0x04)
#define EPORT_EPFR_EPF3			(0x08)
#define EPORT_EPFR_EPF4			(0x10)
#define EPORT_EPFR_EPF5			(0x20)
#define EPORT_EPFR_EPF6			(0x40)
#define EPORT_EPFR_EPF7			(0x80)

/*********************************************************************
* Watchdog Timer Modules (WTM)
*********************************************************************/

/* Bit definitions and macros for WCR */
#define WTM_WCR_EN			(0x0001)
#define WTM_WCR_HALTED			(0x0002)
#define WTM_WCR_DOZE			(0x0004)
#define WTM_WCR_WAIT			(0x0008)

/*********************************************************************
* Serial Boot Facility (SBF)
*********************************************************************/

/* Bit definitions and macros for SBFCR */
#define SBF_SBFCR_BLDIV(x)		(((x)&0x000F))	/* Boot loader clock divider */
#define SBF_SBFCR_FR			(0x0010)	/* Fast read */

/*********************************************************************
* Reset Controller Module (RCM)
*********************************************************************/

/* Bit definitions and macros for RCR */
#define RCM_RCR_FRCRSTOUT		(0x40)
#define RCM_RCR_SOFTRST			(0x80)

/* Bit definitions and macros for RSR */
#define RCM_RSR_LOL			(0x01)
#define RCM_RSR_WDR_CORE		(0x02)
#define RCM_RSR_EXT			(0x04)
#define RCM_RSR_POR			(0x08)
#define RCM_RSR_SOFT			(0x20)

/*********************************************************************
* Chip Configuration Module (CCM)
*********************************************************************/

/* Bit definitions and macros for CCR_360 */
#define CCM_CCR_360_PLLMULT2(x)		(((x)&0x0003))	/* 2-Bit PLL clock mode */
#define CCM_CCR_360_PCISLEW		(0x0004)	/* PCI pad slew rate mode */
#define CCM_CCR_360_PCIMODE		(0x0008)	/* PCI host/agent mode */
#define CCM_CCR_360_PLLMODE		(0x0010)	/* PLL Mode */
#define CCM_CCR_360_FBCONFIG(x)		(((x)&0x0007)<<5)	/* Flexbus/PCI port size configuration */
#define CCM_CCR_360_PLLMULT3(x)		(((x)&0x0007))	/* 3-Bit PLL Clock Mode */
#define CCM_CCR_360_OSCMODE		(0x0008)	/* Oscillator Clock Mode */
#define CCM_CCR_360_FBCONFIG_MASK	(0x00E0)
#define CCM_CCR_360_PLLMULT2_MASK	(0x0003)
#define CCM_CCR_360_PLLMULT3_MASK	(0x0007)
#define CCM_CCR_360_FBCONFIG_NM_NP_32	(0x0000)
#define CCM_CCR_360_FBCONFIG_NM_NP_8	(0x0020)
#define CCM_CCR_360_FBCONFIG_NM_NP_16	(0x0040)
#define CCM_CCR_360_FBCONFIG_M_P_16	(0x0060)
#define CCM_CCR_360_FBCONFIG_M_NP_32	(0x0080)
#define CCM_CCR_360_FBCONFIG_M_NP_8	(0x00A0)
#define CCM_CCR_360_FBCONFIG_M_NP_16	(0x00C0)
#define CCM_CCR_360_FBCONFIG_M_P_8	(0x00E0)
#define CCM_CCR_360_PLLMULT2_12X	(0x0000)
#define CCM_CCR_360_PLLMULT2_6X		(0x0001)
#define CCM_CCR_360_PLLMULT2_16X	(0x0002)
#define CCM_CCR_360_PLLMULT2_8X		(0x0003)
#define CCM_CCR_360_PLLMULT3_20X	(0x0000)
#define CCM_CCR_360_PLLMULT3_10X	(0x0001)
#define CCM_CCR_360_PLLMULT3_24X	(0x0002)
#define CCM_CCR_360_PLLMULT3_18X	(0x0003)
#define CCM_CCR_360_PLLMULT3_12X	(0x0004)
#define CCM_CCR_360_PLLMULT3_6X		(0x0005)
#define CCM_CCR_360_PLLMULT3_16X	(0x0006)
#define CCM_CCR_360_PLLMULT3_8X		(0x0007)

/* Bit definitions and macros for CCR_256 */
#define CCM_CCR_256_PLLMULT3(x)		(((x)&0x0007))	/* 3-Bit PLL clock mode */
#define CCM_CCR_256_OSCMODE		(0x0008)	/* Oscillator clock mode */
#define CCM_CCR_256_PLLMODE		(0x0010)	/* PLL Mode */
#define CCM_CCR_256_FBCONFIG(x)		(((x)&0x0007)<<5)	/* Flexbus/PCI port size configuration */
#define CCM_CCR_256_FBCONFIG_MASK	(0x00E0)
#define CCM_CCR_256_FBCONFIG_NM_32	(0x0000)
#define CCM_CCR_256_FBCONFIG_NM_8	(0x0020)
#define CCM_CCR_256_FBCONFIG_NM_16	(0x0040)
#define CCM_CCR_256_FBCONFIG_M_32	(0x0080)
#define CCM_CCR_256_FBCONFIG_M_8	(0x00A0)
#define CCM_CCR_256_FBCONFIG_M_16	(0x00C0)
#define CCM_CCR_256_PLLMULT3_MASK	(0x0007)
#define CCM_CCR_256_PLLMULT3_20X	(0x0000)
#define CCM_CCR_256_PLLMULT3_10X	(0x0001)
#define CCM_CCR_256_PLLMULT3_24X	(0x0002)
#define CCM_CCR_256_PLLMULT3_18X	(0x0003)
#define CCM_CCR_256_PLLMULT3_12X	(0x0004)
#define CCM_CCR_256_PLLMULT3_6X		(0x0005)
#define CCM_CCR_256_PLLMULT3_16X	(0x0006)
#define CCM_CCR_256_PLLMULT3_8X		(0x0007)

/* Bit definitions and macros for RCON_360 */
#define CCM_RCON_360_PLLMULT(x)		(((x)&0x0003))	/* PLL clock mode */
#define CCM_RCON_360_PCISLEW		(0x0004)	/* PCI pad slew rate mode */
#define CCM_RCON_360_PCIMODE		(0x0008)	/* PCI host/agent mode */
#define CCM_RCON_360_PLLMODE		(0x0010)	/* PLL Mode */
#define CCM_RCON_360_FBCONFIG(x)	(((x)&0x0007)<<5)	/* Flexbus/PCI port size configuration */

/* Bit definitions and macros for RCON_256 */
#define CCM_RCON_256_PLLMULT(x)		(((x)&0x0007))	/* PLL clock mode */
#define CCM_RCON_256_OSCMODE		(0x0008)	/* Oscillator clock mode */
#define CCM_RCON_256_PLLMODE		(0x0010)	/* PLL Mode */
#define CCM_RCON_256_FBCONFIG(x)	(((x)&0x0007)<<5)	/* Flexbus/PCI port size configuration */

/* Bit definitions and macros for CIR */
#define CCM_CIR_PRN(x)			(((x)&0x003F))	/* Part revision number */
#define CCM_CIR_PIN(x)			(((x)&0x03FF)<<6)	/* Part identification number */
#define CCM_CIR_PIN_MASK		(0xFFC0)
#define CCM_CIR_PRN_MASK		(0x003F)
#define CCM_CIR_PIN_MCF54450		(0x4F<<6)
#define CCM_CIR_PIN_MCF54451		(0x4D<<6)
#define CCM_CIR_PIN_MCF54452		(0x4B<<6)
#define CCM_CIR_PIN_MCF54453		(0x49<<6)
#define CCM_CIR_PIN_MCF54454		(0x4A<<6)
#define CCM_CIR_PIN_MCF54455		(0x48<<6)

/* Bit definitions and macros for MISCCR */
#define CCM_MISCCR_USBSRC		(0x0001)	/* USB clock source */
#define CCM_MISCCR_USBOC		(0x0002)	/* USB VBUS over-current sense polarity */
#define CCM_MISCCR_USBPUE		(0x0004)	/* USB transceiver pull-up enable */
#define CCM_MISCCR_SSISRC		(0x0010)	/* SSI clock source */
#define CCM_MISCCR_TIMDMA		(0x0020)	/* Timer DMA mux selection */
#define CCM_MISCCR_SSIPUS		(0x0040)	/* SSI RXD/TXD pull select */
#define CCM_MISCCR_SSIPUE		(0x0080)	/* SSI RXD/TXD pull enable */
#define CCM_MISCCR_BMT(x)		(((x)&0x0007)<<8)	/* Bus monitor timing field */
#define CCM_MISCCR_BME			(0x0800)	/* Bus monitor external enable bit */
#define CCM_MISCCR_LIMP			(0x1000)	/* Limp mode enable */
#define CCM_MISCCR_BMT_65536		(0)
#define CCM_MISCCR_BMT_32768		(1)
#define CCM_MISCCR_BMT_16384		(2)
#define CCM_MISCCR_BMT_8192		(3)
#define CCM_MISCCR_BMT_4096		(4)
#define CCM_MISCCR_BMT_2048		(5)
#define CCM_MISCCR_BMT_1024		(6)
#define CCM_MISCCR_BMT_512		(7)
#define CCM_MISCCR_SSIPUS_UP		(1)
#define CCM_MISCCR_SSIPUS_DOWN		(0)
#define CCM_MISCCR_TIMDMA_TIM		(1)
#define CCM_MISCCR_TIMDMA_SSI		(0)
#define CCM_MISCCR_SSISRC_CLKIN		(0)
#define CCM_MISCCR_SSISRC_PLL		(1)
#define CCM_MISCCR_USBOC_ACTHI		(0)
#define CCM_MISCCR_USBOV_ACTLO		(1)
#define CCM_MISCCR_USBSRC_CLKIN		(0)
#define CCM_MISCCR_USBSRC_PLL		(1)

/* Bit definitions and macros for CDR */
#define CCM_CDR_SSIDIV(x)		(((x)&0x00FF))	/* SSI oversampling clock divider */
#define CCM_CDR_LPDIV(x)		(((x)&0x000F)<<8)	/* Low power clock divider */

/* Bit definitions and macros for UOCSR */
#define CCM_UOCSR_XPDE			(0x0001)	/* On-chip transceiver pull-down enable */
#define CCM_UOCSR_UOMIE			(0x0002)	/* USB OTG misc interrupt enable */
#define CCM_UOCSR_WKUP			(0x0004)	/* USB OTG controller wake-up event */
#define CCM_UOCSR_PWRFLT		(0x0008)	/* VBUS power fault */
#define CCM_UOCSR_SEND			(0x0010)	/* Session end */
#define CCM_UOCSR_VVLD			(0x0020)	/* VBUS valid indicator */
#define CCM_UOCSR_BVLD			(0x0040)	/* B-peripheral valid indicator */
#define CCM_UOCSR_AVLD			(0x0080)	/* A-peripheral valid indicator */
#define CCM_UOCSR_DPPU			(0x0100)	/* D+ pull-up for FS enabled (read-only) */
#define CCM_UOCSR_DCR_VBUS		(0x0200)	/* VBUS discharge resistor enabled (read-only) */
#define CCM_UOCSR_CRG_VBUS		(0x0400)	/* VBUS charge resistor enabled (read-only) */
#define CCM_UOCSR_DMPD			(0x1000)	/* D- 15Kohm pull-down (read-only) */
#define CCM_UOCSR_DPPD			(0x2000)	/* D+ 15Kohm pull-down (read-only) */

/*********************************************************************
* General Purpose I/O Module (GPIO)
*********************************************************************/

/* Bit definitions and macros for PAR_FEC */
#define GPIO_PAR_FEC_FEC0(x)		(((x)&0x07))
#define GPIO_PAR_FEC_FEC1(x)		(((x)&0x07)<<4)
#define GPIO_PAR_FEC_FEC1_MASK		(0x8F)
#define GPIO_PAR_FEC_FEC1_MII		(0x70)
#define GPIO_PAR_FEC_FEC1_RMII_GPIO	(0x30)
#define GPIO_PAR_FEC_FEC1_RMII_ATA	(0x20)
#define GPIO_PAR_FEC_FEC1_ATA		(0x10)
#define GPIO_PAR_FEC_FEC1_GPIO		(0x00)
#define GPIO_PAR_FEC_FEC0_MASK		(0xF8)
#define GPIO_PAR_FEC_FEC0_MII		(0x07)
#define GPIO_PAR_FEC_FEC0_RMII_GPIO	(0x03)
#define GPIO_PAR_FEC_FEC0_RMII_ATA	(0x02)
#define GPIO_PAR_FEC_FEC0_ATA		(0x01)
#define GPIO_PAR_FEC_FEC0_GPIO		(0x00)

/* Bit definitions and macros for PAR_DMA */
#define GPIO_PAR_DMA_DREQ0		(0x01)
#define GPIO_PAR_DMA_DACK0(x)		(((x)&0x03)<<2)
#define GPIO_PAR_DMA_DREQ1(x)		(((x)&0x03)<<4)
#define GPIO_PAR_DMA_DACK1(x)		(((x)&0x03)<<6)
#define GPIO_PAR_DMA_DACK1_MASK		(0x3F)
#define GPIO_PAR_DMA_DACK1_DACK1	(0xC0)
#define GPIO_PAR_DMA_DACK1_ULPI_DIR	(0x40)
#define GPIO_PAR_DMA_DACK1_GPIO		(0x00)
#define GPIO_PAR_DMA_DREQ1_MASK		(0xCF)
#define GPIO_PAR_DMA_DREQ1_DREQ1	(0x30)
#define GPIO_PAR_DMA_DREQ1_USB_CLKIN	(0x10)
#define GPIO_PAR_DMA_DREQ1_GPIO		(0x00)
#define GPIO_PAR_DMA_DACK0_MASK		(0xF3)
#define GPIO_PAR_DMA_DACK0_DACK1	(0x0C)
#define GPIO_PAR_DMA_DACK0_ULPI_DIR	(0x04)
#define GPIO_PAR_DMA_DACK0_GPIO		(0x00)
#define GPIO_PAR_DMA_DREQ0_DREQ0	(0x01)
#define GPIO_PAR_DMA_DREQ0_GPIO		(0x00)

/* Bit definitions and macros for PAR_FBCTL */
#define GPIO_PAR_FBCTL_TS(x)		(((x)&0x03)<<3)
#define GPIO_PAR_FBCTL_RW		(0x20)
#define GPIO_PAR_FBCTL_TA		(0x40)
#define GPIO_PAR_FBCTL_OE		(0x80)
#define GPIO_PAR_FBCTL_OE_OE		(0x80)
#define GPIO_PAR_FBCTL_OE_GPIO		(0x00)
#define GPIO_PAR_FBCTL_TA_TA		(0x40)
#define GPIO_PAR_FBCTL_TA_GPIO		(0x00)
#define GPIO_PAR_FBCTL_RW_RW		(0x20)
#define GPIO_PAR_FBCTL_RW_GPIO		(0x00)
#define GPIO_PAR_FBCTL_TS_MASK		(0xE7)
#define GPIO_PAR_FBCTL_TS_TS		(0x18)
#define GPIO_PAR_FBCTL_TS_ALE		(0x10)
#define GPIO_PAR_FBCTL_TS_TBST		(0x08)
#define GPIO_PAR_FBCTL_TS_GPIO		(0x80)

/* Bit definitions and macros for PAR_DSPI */
#define GPIO_PAR_DSPI_SCK		(0x01)
#define GPIO_PAR_DSPI_SOUT		(0x02)
#define GPIO_PAR_DSPI_SIN		(0x04)
#define GPIO_PAR_DSPI_PCS0		(0x08)
#define GPIO_PAR_DSPI_PCS1		(0x10)
#define GPIO_PAR_DSPI_PCS2		(0x20)
#define GPIO_PAR_DSPI_PCS5		(0x40)
#define GPIO_PAR_DSPI_PCS5_PCS5		(0x40)
#define GPIO_PAR_DSPI_PCS5_GPIO		(0x00)
#define GPIO_PAR_DSPI_PCS2_PCS2		(0x20)
#define GPIO_PAR_DSPI_PCS2_GPIO		(0x00)
#define GPIO_PAR_DSPI_PCS1_PCS1		(0x10)
#define GPIO_PAR_DSPI_PCS1_GPIO		(0x00)
#define GPIO_PAR_DSPI_PCS0_PCS0		(0x08)
#define GPIO_PAR_DSPI_PCS0_GPIO		(0x00)
#define GPIO_PAR_DSPI_SIN_SIN		(0x04)
#define GPIO_PAR_DSPI_SIN_GPIO		(0x00)
#define GPIO_PAR_DSPI_SOUT_SOUT		(0x02)
#define GPIO_PAR_DSPI_SOUT_GPIO		(0x00)
#define GPIO_PAR_DSPI_SCK_SCK		(0x01)
#define GPIO_PAR_DSPI_SCK_GPIO		(0x00)

/* Bit definitions and macros for PAR_BE */
#define GPIO_PAR_BE_BS0			(0x01)
#define GPIO_PAR_BE_BS1			(0x04)
#define GPIO_PAR_BE_BS2(x)		(((x)&0x03)<<4)
#define GPIO_PAR_BE_BS3(x)		(((x)&0x03)<<6)
#define GPIO_PAR_BE_BE3_MASK		(0x3F)
#define GPIO_PAR_BE_BE3_BE3		(0xC0)
#define GPIO_PAR_BE_BE3_TSIZ1		(0x80)
#define GPIO_PAR_BE_BE3_GPIO		(0x00)
#define GPIO_PAR_BE_BE2_MASK		(0xCF)
#define GPIO_PAR_BE_BE2_BE2		(0x30)
#define GPIO_PAR_BE_BE2_TSIZ0		(0x20)
#define GPIO_PAR_BE_BE2_GPIO		(0x00)
#define GPIO_PAR_BE_BE1_BE1		(0x04)
#define GPIO_PAR_BE_BE1_GPIO		(0x00)
#define GPIO_PAR_BE_BE0_BE0		(0x01)
#define GPIO_PAR_BE_BE0_GPIO		(0x00)

/* Bit definitions and macros for PAR_CS */
#define GPIO_PAR_CS_CS1			(0x02)
#define GPIO_PAR_CS_CS2			(0x04)
#define GPIO_PAR_CS_CS3			(0x08)
#define GPIO_PAR_CS_CS3_CS3		(0x08)
#define GPIO_PAR_CS_CS3_GPIO		(0x00)
#define GPIO_PAR_CS_CS2_CS2		(0x04)
#define GPIO_PAR_CS_CS2_GPIO		(0x00)
#define GPIO_PAR_CS_CS1_CS1		(0x02)
#define GPIO_PAR_CS_CS1_GPIO		(0x00)

/* Bit definitions and macros for PAR_TIMER */
#define GPIO_PAR_TIMER_T0IN(x)		(((x)&0x03))
#define GPIO_PAR_TIMER_T1IN(x)		(((x)&0x03)<<2)
#define GPIO_PAR_TIMER_T2IN(x)		(((x)&0x03)<<4)
#define GPIO_PAR_TIMER_T3IN(x)		(((x)&0x03)<<6)
#define GPIO_PAR_TIMER_T3IN_MASK	(0x3F)
#define GPIO_PAR_TIMER_T3IN_T3IN	(0xC0)
#define GPIO_PAR_TIMER_T3IN_T3OUT	(0x80)
#define GPIO_PAR_TIMER_T3IN_U2RXD	(0x40)
#define GPIO_PAR_TIMER_T3IN_GPIO	(0x00)
#define GPIO_PAR_TIMER_T2IN_MASK	(0xCF)
#define GPIO_PAR_TIMER_T2IN_T2IN	(0x30)
#define GPIO_PAR_TIMER_T2IN_T2OUT	(0x20)
#define GPIO_PAR_TIMER_T2IN_U2TXD	(0x10)
#define GPIO_PAR_TIMER_T2IN_GPIO	(0x00)
#define GPIO_PAR_TIMER_T1IN_MASK	(0xF3)
#define GPIO_PAR_TIMER_T1IN_T1IN	(0x0C)
#define GPIO_PAR_TIMER_T1IN_T1OUT	(0x08)
#define GPIO_PAR_TIMER_T1IN_U2CTS	(0x04)
#define GPIO_PAR_TIMER_T1IN_GPIO	(0x00)
#define GPIO_PAR_TIMER_T0IN_MASK	(0xFC)
#define GPIO_PAR_TIMER_T0IN_T0IN	(0x03)
#define GPIO_PAR_TIMER_T0IN_T0OUT	(0x02)
#define GPIO_PAR_TIMER_T0IN_U2RTS	(0x01)
#define GPIO_PAR_TIMER_T0IN_GPIO	(0x00)

/* Bit definitions and macros for PAR_USB */
#define GPIO_PAR_USB_VBUSOC(x)		(((x)&0x03))
#define GPIO_PAR_USB_VBUSEN(x)		(((x)&0x03)<<2)
#define GPIO_PAR_USB_VBUSEN_MASK	(0xF3)
#define GPIO_PAR_USB_VBUSEN_VBUSEN	(0x0C)
#define GPIO_PAR_USB_VBUSEN_USBPULLUP	(0x08)
#define GPIO_PAR_USB_VBUSEN_ULPI_NXT	(0x04)
#define GPIO_PAR_USB_VBUSEN_GPIO	(0x00)
#define GPIO_PAR_USB_VBUSOC_MASK	(0xFC)
#define GPIO_PAR_USB_VBUSOC_VBUSOC	(0x03)
#define GPIO_PAR_USB_VBUSOC_ULPI_STP	(0x01)
#define GPIO_PAR_USB_VBUSOC_GPIO	(0x00)

/* Bit definitions and macros for PAR_UART */
#define GPIO_PAR_UART_U0TXD		(0x01)
#define GPIO_PAR_UART_U0RXD		(0x02)
#define GPIO_PAR_UART_U0RTS		(0x04)
#define GPIO_PAR_UART_U0CTS		(0x08)
#define GPIO_PAR_UART_U1TXD		(0x10)
#define GPIO_PAR_UART_U1RXD		(0x20)
#define GPIO_PAR_UART_U1RTS		(0x40)
#define GPIO_PAR_UART_U1CTS		(0x80)
#define GPIO_PAR_UART_U1CTS_U1CTS	(0x80)
#define GPIO_PAR_UART_U1CTS_GPIO	(0x00)
#define GPIO_PAR_UART_U1RTS_U1RTS	(0x40)
#define GPIO_PAR_UART_U1RTS_GPIO	(0x00)
#define GPIO_PAR_UART_U1RXD_U1RXD	(0x20)
#define GPIO_PAR_UART_U1RXD_GPIO	(0x00)
#define GPIO_PAR_UART_U1TXD_U1TXD	(0x10)
#define GPIO_PAR_UART_U1TXD_GPIO	(0x00)
#define GPIO_PAR_UART_U0CTS_U0CTS	(0x08)
#define GPIO_PAR_UART_U0CTS_GPIO	(0x00)
#define GPIO_PAR_UART_U0RTS_U0RTS	(0x04)
#define GPIO_PAR_UART_U0RTS_GPIO	(0x00)
#define GPIO_PAR_UART_U0RXD_U0RXD	(0x02)
#define GPIO_PAR_UART_U0RXD_GPIO	(0x00)
#define GPIO_PAR_UART_U0TXD_U0TXD	(0x01)
#define GPIO_PAR_UART_U0TXD_GPIO	(0x00)

/* Bit definitions and macros for PAR_FECI2C */
#define GPIO_PAR_FECI2C_SDA(x)		(((x)&0x0003))
#define GPIO_PAR_FECI2C_SCL(x)		(((x)&0x0003)<<2)
#define GPIO_PAR_FECI2C_MDIO0		(0x0010)
#define GPIO_PAR_FECI2C_MDC0		(0x0040)
#define GPIO_PAR_FECI2C_MDIO1(x)	(((x)&0x0003)<<8)
#define GPIO_PAR_FECI2C_MDC1(x)		(((x)&0x0003)<<10)
#define GPIO_PAR_FECI2C_MDC1_MASK	(0xF3FF)
#define GPIO_PAR_FECI2C_MDC1_MDC1	(0x0C00)
#define GPIO_PAR_FECI2C_MDC1_ATA_DIOR	(0x0800)
#define GPIO_PAR_FECI2C_MDC1_GPIO	(0x0000)
#define GPIO_PAR_FECI2C_MDIO1_MASK	(0xFCFF)
#define GPIO_PAR_FECI2C_MDIO1_MDIO1	(0x0300)
#define GPIO_PAR_FECI2C_MDIO1_ATA_DIOW	(0x0200)
#define GPIO_PAR_FECI2C_MDIO1_GPIO	(0x0000)
#define GPIO_PAR_FECI2C_MDC0_MDC0	(0x0040)
#define GPIO_PAR_FECI2C_MDC0_GPIO	(0x0000)
#define GPIO_PAR_FECI2C_MDIO0_MDIO0	(0x0010)
#define GPIO_PAR_FECI2C_MDIO0_GPIO	(0x0000)
#define GPIO_PAR_FECI2C_SCL_MASK	(0xFFF3)
#define GPIO_PAR_FECI2C_SCL_SCL		(0x000C)
#define GPIO_PAR_FECI2C_SCL_U2TXD	(0x0004)
#define GPIO_PAR_FECI2C_SCL_GPIO	(0x0000)
#define GPIO_PAR_FECI2C_SDA_MASK	(0xFFFC)
#define GPIO_PAR_FECI2C_SDA_SDA		(0x0003)
#define GPIO_PAR_FECI2C_SDA_U2RXD	(0x0001)
#define GPIO_PAR_FECI2C_SDA_GPIO	(0x0000)

/* Bit definitions and macros for PAR_SSI */
#define GPIO_PAR_SSI_MCLK		(0x0001)
#define GPIO_PAR_SSI_STXD(x)		(((x)&0x0003)<<2)
#define GPIO_PAR_SSI_SRXD(x)		(((x)&0x0003)<<4)
#define GPIO_PAR_SSI_FS(x)		(((x)&0x0003)<<6)
#define GPIO_PAR_SSI_BCLK(x)		(((x)&0x0003)<<8)
#define GPIO_PAR_SSI_BCLK_MASK		(0xFCFF)
#define GPIO_PAR_SSI_BCLK_BCLK		(0x0300)
#define GPIO_PAR_SSI_BCLK_U1CTS		(0x0200)
#define GPIO_PAR_SSI_BCLK_GPIO		(0x0000)
#define GPIO_PAR_SSI_FS_MASK		(0xFF3F)
#define GPIO_PAR_SSI_FS_FS		(0x00C0)
#define GPIO_PAR_SSI_FS_U1RTS		(0x0080)
#define GPIO_PAR_SSI_FS_GPIO		(0x0000)
#define GPIO_PAR_SSI_SRXD_MASK		(0xFFCF)
#define GPIO_PAR_SSI_SRXD_SRXD		(0x0030)
#define GPIO_PAR_SSI_SRXD_U1RXD		(0x0020)
#define GPIO_PAR_SSI_SRXD_GPIO		(0x0000)
#define GPIO_PAR_SSI_STXD_MASK		(0xFFF3)
#define GPIO_PAR_SSI_STXD_STXD		(0x000C)
#define GPIO_PAR_SSI_STXD_U1TXD		(0x0008)
#define GPIO_PAR_SSI_STXD_GPIO		(0x0000)
#define GPIO_PAR_SSI_MCLK_MCLK		(0x0001)
#define GPIO_PAR_SSI_MCLK_GPIO		(0x0000)

/* Bit definitions and macros for PAR_ATA */
#define GPIO_PAR_ATA_IORDY		(0x0001)
#define GPIO_PAR_ATA_DMARQ		(0x0002)
#define GPIO_PAR_ATA_RESET		(0x0004)
#define GPIO_PAR_ATA_DA0		(0x0020)
#define GPIO_PAR_ATA_DA1		(0x0040)
#define GPIO_PAR_ATA_DA2		(0x0080)
#define GPIO_PAR_ATA_CS0		(0x0100)
#define GPIO_PAR_ATA_CS1		(0x0200)
#define GPIO_PAR_ATA_BUFEN		(0x0400)
#define GPIO_PAR_ATA_BUFEN_BUFEN	(0x0400)
#define GPIO_PAR_ATA_BUFEN_GPIO		(0x0000)
#define GPIO_PAR_ATA_CS1_CS1		(0x0200)
#define GPIO_PAR_ATA_CS1_GPIO		(0x0000)
#define GPIO_PAR_ATA_CS0_CS0		(0x0100)
#define GPIO_PAR_ATA_CS0_GPIO		(0x0000)
#define GPIO_PAR_ATA_DA2_DA2		(0x0080)
#define GPIO_PAR_ATA_DA2_GPIO		(0x0000)
#define GPIO_PAR_ATA_DA1_DA1		(0x0040)
#define GPIO_PAR_ATA_DA1_GPIO		(0x0000)
#define GPIO_PAR_ATA_DA0_DA0		(0x0020)
#define GPIO_PAR_ATA_DA0_GPIO		(0x0000)
#define GPIO_PAR_ATA_RESET_RESET	(0x0004)
#define GPIO_PAR_ATA_RESET_GPIO		(0x0000)
#define GPIO_PAR_ATA_DMARQ_DMARQ	(0x0002)
#define GPIO_PAR_ATA_DMARQ_GPIO		(0x0000)
#define GPIO_PAR_ATA_IORDY_IORDY	(0x0001)
#define GPIO_PAR_ATA_IORDY_GPIO		(0x0000)

/* Bit definitions and macros for PAR_IRQ */
#define GPIO_PAR_IRQ_IRQ1		(0x02)
#define GPIO_PAR_IRQ_IRQ4		(0x10)
#define GPIO_PAR_IRQ_IRQ4_IRQ4		(0x10)
#define GPIO_PAR_IRQ_IRQ4_GPIO		(0x00)
#define GPIO_PAR_IRQ_IRQ1_IRQ1		(0x02)
#define GPIO_PAR_IRQ_IRQ1_GPIO		(0x00)

/* Bit definitions and macros for PAR_PCI */
#define GPIO_PAR_PCI_REQ0		(0x0001)
#define GPIO_PAR_PCI_REQ1		(0x0004)
#define GPIO_PAR_PCI_REQ2		(0x0010)
#define GPIO_PAR_PCI_REQ3(x)		(((x)&0x0003)<<6)
#define GPIO_PAR_PCI_GNT0		(0x0100)
#define GPIO_PAR_PCI_GNT1		(0x0400)
#define GPIO_PAR_PCI_GNT2		(0x1000)
#define GPIO_PAR_PCI_GNT3(x)		(((x)&0x0003)<<14)
#define GPIO_PAR_PCI_GNT3_MASK		(0x3FFF)
#define GPIO_PAR_PCI_GNT3_GNT3		(0xC000)
#define GPIO_PAR_PCI_GNT3_ATA_DMACK	(0x8000)
#define GPIO_PAR_PCI_GNT3_GPIO		(0x0000)
#define GPIO_PAR_PCI_GNT2_GNT2		(0x1000)
#define GPIO_PAR_PCI_GNT2_GPIO		(0x0000)
#define GPIO_PAR_PCI_GNT1_GNT1		(0x0400)
#define GPIO_PAR_PCI_GNT1_GPIO		(0x0000)
#define GPIO_PAR_PCI_GNT0_GNT0		(0x0100)
#define GPIO_PAR_PCI_GNT0_GPIO		(0x0000)
#define GPIO_PAR_PCI_REQ3_MASK		(0xFF3F)
#define GPIO_PAR_PCI_REQ3_REQ3		(0x00C0)
#define GPIO_PAR_PCI_REQ3_ATA_INTRQ	(0x0080)
#define GPIO_PAR_PCI_REQ3_GPIO		(0x0000)
#define GPIO_PAR_PCI_REQ2_REQ2		(0x0010)
#define GPIO_PAR_PCI_REQ2_GPIO		(0x0000)
#define GPIO_PAR_PCI_REQ1_REQ1		(0x0040)
#define GPIO_PAR_PCI_REQ1_GPIO		(0x0000)
#define GPIO_PAR_PCI_REQ0_REQ0		(0x0001)
#define GPIO_PAR_PCI_REQ0_GPIO		(0x0000)

/* Bit definitions and macros for MSCR_SDRAM */
#define GPIO_MSCR_SDRAM_SDCTL(x)	(((x)&0x03))
#define GPIO_MSCR_SDRAM_SDCLK(x)	(((x)&0x03)<<2)
#define GPIO_MSCR_SDRAM_SDDQS(x)	(((x)&0x03)<<4)
#define GPIO_MSCR_SDRAM_SDDATA(x)	(((x)&0x03)<<6)
#define GPIO_MSCR_SDRAM_SDDATA_MASK	(0x3F)
#define GPIO_MSCR_SDRAM_SDDATA_DDR1	(0xC0)
#define GPIO_MSCR_SDRAM_SDDATA_DDR2	(0x80)
#define GPIO_MSCR_SDRAM_SDDATA_FS_LPDDR	(0x40)
#define GPIO_MSCR_SDRAM_SDDATA_HS_LPDDR	(0x00)
#define GPIO_MSCR_SDRAM_SDDQS_MASK	(0xCF)
#define GPIO_MSCR_SDRAM_SDDQS_DDR1	(0x30)
#define GPIO_MSCR_SDRAM_SDDQS_DDR2	(0x20)
#define GPIO_MSCR_SDRAM_SDDQS_FS_LPDDR	(0x10)
#define GPIO_MSCR_SDRAM_SDDQS_HS_LPDDR	(0x00)
#define GPIO_MSCR_SDRAM_SDCLK_MASK	(0xF3)
#define GPIO_MSCR_SDRAM_SDCLK_DDR1	(0x0C)
#define GPIO_MSCR_SDRAM_SDCLK_DDR2	(0x08)
#define GPIO_MSCR_SDRAM_SDCLK_FS_LPDDR	(0x04)
#define GPIO_MSCR_SDRAM_SDCLK_HS_LPDDR	(0x00)
#define GPIO_MSCR_SDRAM_SDCTL_MASK	(0xFC)
#define GPIO_MSCR_SDRAM_SDCTL_DDR1	(0x03)
#define GPIO_MSCR_SDRAM_SDCTL_DDR2	(0x02)
#define GPIO_MSCR_SDRAM_SDCTL_FS_LPDDR	(0x01)
#define GPIO_MSCR_SDRAM_SDCTL_HS_LPDDR	(0x00)

/* Bit definitions and macros for MSCR_PCI */
#define GPIO_MSCR_PCI_PCI		(0x01)
#define GPIO_MSCR_PCI_PCI_HI_66MHZ	(0x01)
#define GPIO_MSCR_PCI_PCI_LO_33MHZ	(0x00)

/* Bit definitions and macros for DSCR_I2C */
#define GPIO_DSCR_I2C_I2C(x)		(((x)&0x03))
#define GPIO_DSCR_I2C_I2C_LOAD_50PF	(0x03)
#define GPIO_DSCR_I2C_I2C_LOAD_30PF	(0x02)
#define GPIO_DSCR_I2C_I2C_LOAD_20PF	(0x01)
#define GPIO_DSCR_I2C_I2C_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_FLEXBUS */
#define GPIO_DSCR_FLEXBUS_FBADL(x)		(((x)&0x03))
#define GPIO_DSCR_FLEXBUS_FBADH(x)		(((x)&0x03)<<2)
#define GPIO_DSCR_FLEXBUS_FBCTL(x)		(((x)&0x03)<<4)
#define GPIO_DSCR_FLEXBUS_FBCLK(x)		(((x)&0x03)<<6)
#define GPIO_DSCR_FLEXBUS_FBCLK_LOAD_50PF	(0xC0)
#define GPIO_DSCR_FLEXBUS_FBCLK_LOAD_30PF	(0x80)
#define GPIO_DSCR_FLEXBUS_FBCLK_LOAD_20PF	(0x40)
#define GPIO_DSCR_FLEXBUS_FBCLK_LOAD_10PF	(0x00)
#define GPIO_DSCR_FLEXBUS_FBCTL_LOAD_50PF	(0x30)
#define GPIO_DSCR_FLEXBUS_FBCTL_LOAD_30PF	(0x20)
#define GPIO_DSCR_FLEXBUS_FBCTL_LOAD_20PF	(0x10)
#define GPIO_DSCR_FLEXBUS_FBCTL_LOAD_10PF	(0x00)
#define GPIO_DSCR_FLEXBUS_FBADH_LOAD_50PF	(0x0C)
#define GPIO_DSCR_FLEXBUS_FBADH_LOAD_30PF	(0x08)
#define GPIO_DSCR_FLEXBUS_FBADH_LOAD_20PF	(0x04)
#define GPIO_DSCR_FLEXBUS_FBADH_LOAD_10PF	(0x00)
#define GPIO_DSCR_FLEXBUS_FBADL_LOAD_50PF	(0x03)
#define GPIO_DSCR_FLEXBUS_FBADL_LOAD_30PF	(0x02)
#define GPIO_DSCR_FLEXBUS_FBADL_LOAD_20PF	(0x01)
#define GPIO_DSCR_FLEXBUS_FBADL_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_FEC */
#define GPIO_DSCR_FEC_FEC0(x)		(((x)&0x03))
#define GPIO_DSCR_FEC_FEC1(x)		(((x)&0x03)<<2)
#define GPIO_DSCR_FEC_FEC1_LOAD_50PF	(0x0C)
#define GPIO_DSCR_FEC_FEC1_LOAD_30PF	(0x08)
#define GPIO_DSCR_FEC_FEC1_LOAD_20PF	(0x04)
#define GPIO_DSCR_FEC_FEC1_LOAD_10PF	(0x00)
#define GPIO_DSCR_FEC_FEC0_LOAD_50PF	(0x03)
#define GPIO_DSCR_FEC_FEC0_LOAD_30PF	(0x02)
#define GPIO_DSCR_FEC_FEC0_LOAD_20PF	(0x01)
#define GPIO_DSCR_FEC_FEC0_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_UART */
#define GPIO_DSCR_UART_UART0(x)		(((x)&0x03))
#define GPIO_DSCR_UART_UART1(x)		(((x)&0x03)<<2)
#define GPIO_DSCR_UART_UART1_LOAD_50PF	(0x0C)
#define GPIO_DSCR_UART_UART1_LOAD_30PF	(0x08)
#define GPIO_DSCR_UART_UART1_LOAD_20PF	(0x04)
#define GPIO_DSCR_UART_UART1_LOAD_10PF	(0x00)
#define GPIO_DSCR_UART_UART0_LOAD_50PF	(0x03)
#define GPIO_DSCR_UART_UART0_LOAD_30PF	(0x02)
#define GPIO_DSCR_UART_UART0_LOAD_20PF	(0x01)
#define GPIO_DSCR_UART_UART0_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_DSPI */
#define GPIO_DSCR_DSPI_DSPI(x)		(((x)&0x03))
#define GPIO_DSCR_DSPI_DSPI_LOAD_50PF	(0x03)
#define GPIO_DSCR_DSPI_DSPI_LOAD_30PF	(0x02)
#define GPIO_DSCR_DSPI_DSPI_LOAD_20PF	(0x01)
#define GPIO_DSCR_DSPI_DSPI_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_TIMER */
#define GPIO_DSCR_TIMER_TIMER(x)	(((x)&0x03))
#define GPIO_DSCR_TIMER_TIMER_LOAD_50PF	(0x03)
#define GPIO_DSCR_TIMER_TIMER_LOAD_30PF	(0x02)
#define GPIO_DSCR_TIMER_TIMER_LOAD_20PF	(0x01)
#define GPIO_DSCR_TIMER_TIMER_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_SSI */
#define GPIO_DSCR_SSI_SSI(x)		(((x)&0x03))
#define GPIO_DSCR_SSI_SSI_LOAD_50PF	(0x03)
#define GPIO_DSCR_SSI_SSI_LOAD_30PF	(0x02)
#define GPIO_DSCR_SSI_SSI_LOAD_20PF	(0x01)
#define GPIO_DSCR_SSI_SSI_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_DMA */
#define GPIO_DSCR_DMA_DMA(x)		(((x)&0x03))
#define GPIO_DSCR_DMA_DMA_LOAD_50PF	(0x03)
#define GPIO_DSCR_DMA_DMA_LOAD_30PF	(0x02)
#define GPIO_DSCR_DMA_DMA_LOAD_20PF	(0x01)
#define GPIO_DSCR_DMA_DMA_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_DEBUG */
#define GPIO_DSCR_DEBUG_DEBUG(x)	(((x)&0x03))
#define GPIO_DSCR_DEBUG_DEBUG_LOAD_50PF	(0x03)
#define GPIO_DSCR_DEBUG_DEBUG_LOAD_30PF	(0x02)
#define GPIO_DSCR_DEBUG_DEBUG_LOAD_20PF	(0x01)
#define GPIO_DSCR_DEBUG_DEBUG_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_RESET */
#define GPIO_DSCR_RESET_RESET(x)	(((x)&0x03))
#define GPIO_DSCR_RESET_RESET_LOAD_50PF	(0x03)
#define GPIO_DSCR_RESET_RESET_LOAD_30PF	(0x02)
#define GPIO_DSCR_RESET_RESET_LOAD_20PF	(0x01)
#define GPIO_DSCR_RESET_RESET_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_IRQ */
#define GPIO_DSCR_IRQ_IRQ(x)		(((x)&0x03))
#define GPIO_DSCR_IRQ_IRQ_LOAD_50PF	(0x03)
#define GPIO_DSCR_IRQ_IRQ_LOAD_30PF	(0x02)
#define GPIO_DSCR_IRQ_IRQ_LOAD_20PF	(0x01)
#define GPIO_DSCR_IRQ_IRQ_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_USB */
#define GPIO_DSCR_USB_USB(x)		(((x)&0x03))
#define GPIO_DSCR_USB_USB_LOAD_50PF	(0x03)
#define GPIO_DSCR_USB_USB_LOAD_30PF	(0x02)
#define GPIO_DSCR_USB_USB_LOAD_20PF	(0x01)
#define GPIO_DSCR_USB_USB_LOAD_10PF	(0x00)

/* Bit definitions and macros for DSCR_ATA */
#define GPIO_DSCR_ATA_ATA(x)		(((x)&0x03))
#define GPIO_DSCR_ATA_ATA_LOAD_50PF	(0x03)
#define GPIO_DSCR_ATA_ATA_LOAD_30PF	(0x02)
#define GPIO_DSCR_ATA_ATA_LOAD_20PF	(0x01)
#define GPIO_DSCR_ATA_ATA_LOAD_10PF	(0x00)

/*********************************************************************
* Random Number Generator (RNG)
*********************************************************************/

/* Bit definitions and macros for RNGCR */
#define RNG_RNGCR_GO			(0x00000001)
#define RNG_RNGCR_HA			(0x00000002)
#define RNG_RNGCR_IM			(0x00000004)
#define RNG_RNGCR_CI			(0x00000008)

/* Bit definitions and macros for RNGSR */
#define RNG_RNGSR_SV			(0x00000001)
#define RNG_RNGSR_LRS			(0x00000002)
#define RNG_RNGSR_FUF			(0x00000004)
#define RNG_RNGSR_EI			(0x00000008)
#define RNG_RNGSR_OFL(x)		(((x)&0x000000FF)<<8)
#define RNG_RNGSR_OFS(x)		(((x)&0x000000FF)<<16)

/*********************************************************************
* SDRAM Controller (SDRAMC)
*********************************************************************/

/* Bit definitions and macros for SDMR */
#define SDRAMC_SDMR_DDR2_AD(x)		(((x)&0x00003FFF))	/* Address for DDR2 */
#define SDRAMC_SDMR_CMD			(0x00010000)	/* Command */
#define SDRAMC_SDMR_AD(x)		(((x)&0x00000FFF)<<18)	/* Address */
#define SDRAMC_SDMR_BK(x)		(((x)&0x00000003)<<30)	/* Bank Address */
#define SDRAMC_SDMR_BK_LMR		(0x00000000)
#define SDRAMC_SDMR_BK_LEMR		(0x40000000)

/* Bit definitions and macros for SDCR */
#define SDRAMC_SDCR_DPD			(0x00000001)	/* Deep Power-Down Mode */
#define SDRAMC_SDCR_IPALL		(0x00000002)	/* Initiate Precharge All */
#define SDRAMC_SDCR_IREF		(0x00000004)	/* Initiate Refresh */
#define SDRAMC_SDCR_DQS_OE(x)		(((x)&0x00000003)<<10)	/* DQS Output Enable */
#define SDRAMC_SDCR_MEM_PS		(0x00002000)	/* Data Port Size */
#define SDRAMC_SDCR_REF_CNT(x)		(((x)&0x0000003F)<<16)	/* Periodic Refresh Counter */
#define SDRAMC_SDCR_OE_RULE		(0x00400000)	/* Drive Rule Selection */
#define SDRAMC_SDCR_ADDR_MUX(x)		(((x)&0x00000003)<<24)	/* Internal Address Mux Select */
#define SDRAMC_SDCR_DDR2_MODE		(0x08000000)	/* DDR2 Mode Select */
#define SDRAMC_SDCR_REF_EN		(0x10000000)	/* Refresh Enable */
#define SDRAMC_SDCR_DDR_MODE		(0x20000000)	/* DDR Mode Select */
#define SDRAMC_SDCR_CKE			(0x40000000)	/* Clock Enable */
#define SDRAMC_SDCR_MODE_EN		(0x80000000)	/* SDRAM Mode Register Programming Enable */
#define SDRAMC_SDCR_DQS_OE_BOTH		(0x00000C000)

/* Bit definitions and macros for SDCFG1 */
#define SDRAMC_SDCFG1_WT_LAT(x)		(((x)&0x00000007)<<4)	/* Write Latency */
#define SDRAMC_SDCFG1_REF2ACT(x)	(((x)&0x0000000F)<<8)	/* Refresh to active delay */
#define SDRAMC_SDCFG1_PRE2ACT(x)	(((x)&0x00000007)<<12)	/* Precharge to active delay */
#define SDRAMC_SDCFG1_ACT2RW(x)		(((x)&0x00000007)<<16)	/* Active to read/write delay */
#define SDRAMC_SDCFG1_RD_LAT(x)		(((x)&0x0000000F)<<20)	/* Read CAS Latency */
#define SDRAMC_SDCFG1_SWT2RWP(x)	(((x)&0x00000007)<<24)	/* Single write to read/write/precharge delay */
#define SDRAMC_SDCFG1_SRD2RWP(x)	(((x)&0x0000000F)<<28)	/* Single read to read/write/precharge delay */

/* Bit definitions and macros for SDCFG2 */
#define SDRAMC_SDCFG2_BL(x)		(((x)&0x0000000F)<<16)	/* Burst Length */
#define SDRAMC_SDCFG2_BRD2W(x)		(((x)&0x0000000F)<<20)	/* Burst read to write delay */
#define SDRAMC_SDCFG2_BWT2RWP(x)	(((x)&0x0000000F)<<24)	/* Burst write to read/write/precharge delay */
#define SDRAMC_SDCFG2_BRD2RP(x)		(((x)&0x0000000F)<<28)	/* Burst read to read/precharge delay */

/* Bit definitions and macros for SDCS group */
#define SDRAMC_SDCS_CSSZ(x)		(((x)&0x0000001F))	/* Chip-Select Size */
#define SDRAMC_SDCS_CSBA(x)		(((x)&0x00000FFF)<<20)	/* Chip-Select Base Address */
#define SDRAMC_SDCS_BA(x)		((x)&0xFFF00000)
#define SDRAMC_SDCS_CSSZ_DISABLE	(0x00000000)
#define SDRAMC_SDCS_CSSZ_1MBYTE		(0x00000013)
#define SDRAMC_SDCS_CSSZ_2MBYTE		(0x00000014)
#define SDRAMC_SDCS_CSSZ_4MBYTE		(0x00000015)
#define SDRAMC_SDCS_CSSZ_8MBYTE		(0x00000016)
#define SDRAMC_SDCS_CSSZ_16MBYTE	(0x00000017)
#define SDRAMC_SDCS_CSSZ_32MBYTE	(0x00000018)
#define SDRAMC_SDCS_CSSZ_64MBYTE	(0x00000019)
#define SDRAMC_SDCS_CSSZ_128MBYTE	(0x0000001A)
#define SDRAMC_SDCS_CSSZ_256MBYTE	(0x0000001B)
#define SDRAMC_SDCS_CSSZ_512MBYTE	(0x0000001C)
#define SDRAMC_SDCS_CSSZ_1GBYTE		(0x0000001D)
#define SDRAMC_SDCS_CSSZ_2GBYTE		(0x0000001E)
#define SDRAMC_SDCS_CSSZ_4GBYTE		(0x0000001F)

/*********************************************************************
* Synchronous Serial Interface (SSI)
*********************************************************************/

/* Bit definitions and macros for CR */
#define SSI_CR_SSI_EN			(0x00000001)
#define SSI_CR_TE			(0x00000002)
#define SSI_CR_RE			(0x00000004)
#define SSI_CR_NET			(0x00000008)
#define SSI_CR_SYN			(0x00000010)
#define SSI_CR_I2S(x)			(((x)&0x00000003)<<5)
#define SSI_CR_MCE			(0x00000080)
#define SSI_CR_TCH			(0x00000100)
#define SSI_CR_CIS			(0x00000200)
#define SSI_CR_I2S_NORMAL		(0x00000000)
#define SSI_CR_I2S_MASTER		(0x00000020)
#define SSI_CR_I2S_SLAVE		(0x00000040)

/* Bit definitions and macros for ISR */
#define SSI_ISR_TFE0			(0x00000001)
#define SSI_ISR_TFE1			(0x00000002)
#define SSI_ISR_RFF0			(0x00000004)
#define SSI_ISR_RFF1			(0x00000008)
#define SSI_ISR_RLS			(0x00000010)
#define SSI_ISR_TLS			(0x00000020)
#define SSI_ISR_RFS			(0x00000040)
#define SSI_ISR_TFS			(0x00000080)
#define SSI_ISR_TUE0			(0x00000100)
#define SSI_ISR_TUE1			(0x00000200)
#define SSI_ISR_ROE0			(0x00000400)
#define SSI_ISR_ROE1			(0x00000800)
#define SSI_ISR_TDE0			(0x00001000)
#define SSI_ISR_TDE1			(0x00002000)
#define SSI_ISR_RDR0			(0x00004000)
#define SSI_ISR_RDR1			(0x00008000)
#define SSI_ISR_RXT			(0x00010000)
#define SSI_ISR_CMDDU			(0x00020000)
#define SSI_ISR_CMDAU			(0x00040000)

/* Bit definitions and macros for IER */
#define SSI_IER_TFE0			(0x00000001)
#define SSI_IER_TFE1			(0x00000002)
#define SSI_IER_RFF0			(0x00000004)
#define SSI_IER_RFF1			(0x00000008)
#define SSI_IER_RLS			(0x00000010)
#define SSI_IER_TLS			(0x00000020)
#define SSI_IER_RFS			(0x00000040)
#define SSI_IER_TFS			(0x00000080)
#define SSI_IER_TUE0			(0x00000100)
#define SSI_IER_TUE1			(0x00000200)
#define SSI_IER_ROE0			(0x00000400)
#define SSI_IER_ROE1			(0x00000800)
#define SSI_IER_TDE0			(0x00001000)
#define SSI_IER_TDE1			(0x00002000)
#define SSI_IER_RDR0			(0x00004000)
#define SSI_IER_RDR1			(0x00008000)
#define SSI_IER_RXT			(0x00010000)
#define SSI_IER_CMDU			(0x00020000)
#define SSI_IER_CMDAU			(0x00040000)
#define SSI_IER_TIE			(0x00080000)
#define SSI_IER_TDMAE			(0x00100000)
#define SSI_IER_RIE			(0x00200000)
#define SSI_IER_RDMAE			(0x00400000)

/* Bit definitions and macros for TCR */
#define SSI_TCR_TEFS			(0x00000001)
#define SSI_TCR_TFSL			(0x00000002)
#define SSI_TCR_TFSI			(0x00000004)
#define SSI_TCR_TSCKP			(0x00000008)
#define SSI_TCR_TSHFD			(0x00000010)
#define SSI_TCR_TXDIR			(0x00000020)
#define SSI_TCR_TFDIR			(0x00000040)
#define SSI_TCR_TFEN0			(0x00000080)
#define SSI_TCR_TFEN1			(0x00000100)
#define SSI_TCR_TXBIT0			(0x00000200)

/* Bit definitions and macros for RCR */
#define SSI_RCR_REFS			(0x00000001)
#define SSI_RCR_RFSL			(0x00000002)
#define SSI_RCR_RFSI			(0x00000004)
#define SSI_RCR_RSCKP			(0x00000008)
#define SSI_RCR_RSHFD			(0x00000010)
#define SSI_RCR_RFEN0			(0x00000080)
#define SSI_RCR_RFEN1			(0x00000100)
#define SSI_RCR_RXBIT0			(0x00000200)
#define SSI_RCR_RXEXT			(0x00000400)

/* Bit definitions and macros for CCR */
#define SSI_CCR_PM(x)			(((x)&0x000000FF))
#define SSI_CCR_DC(x)			(((x)&0x0000001F)<<8)
#define SSI_CCR_WL(x)			(((x)&0x0000000F)<<13)
#define SSI_CCR_PSR			(0x00020000)
#define SSI_CCR_DIV2			(0x00040000)

/* Bit definitions and macros for FCSR */
#define SSI_FCSR_TFWM0(x)		(((x)&0x0000000F))
#define SSI_FCSR_RFWM0(x)		(((x)&0x0000000F)<<4)
#define SSI_FCSR_TFCNT0(x)		(((x)&0x0000000F)<<8)
#define SSI_FCSR_RFCNT0(x)		(((x)&0x0000000F)<<12)
#define SSI_FCSR_TFWM1(x)		(((x)&0x0000000F)<<16)
#define SSI_FCSR_RFWM1(x)		(((x)&0x0000000F)<<20)
#define SSI_FCSR_TFCNT1(x)		(((x)&0x0000000F)<<24)
#define SSI_FCSR_RFCNT1(x)		(((x)&0x0000000F)<<28)

/* Bit definitions and macros for ACR */
#define SSI_ACR_AC97EN			(0x00000001)
#define SSI_ACR_FV			(0x00000002)
#define SSI_ACR_TIF			(0x00000004)
#define SSI_ACR_RD			(0x00000008)
#define SSI_ACR_WR			(0x00000010)
#define SSI_ACR_FRDIV(x)		(((x)&0x0000003F)<<5)

/* Bit definitions and macros for ACADD */
#define SSI_ACADD_SSI_ACADD(x)		(((x)&0x0007FFFF))

/* Bit definitions and macros for ACDAT */
#define SSI_ACDAT_SSI_ACDAT(x)		(((x)&0x0007FFFF))

/* Bit definitions and macros for ATAG */
#define SSI_ATAG_DDI_ATAG(x)		(((x)&0x0000FFFF))

/*********************************************************************
* Phase Locked Loop (PLL)
*********************************************************************/

/* Bit definitions and macros for PCR */
#define PLL_PCR_OUTDIV1(x)		(((x)&0x0000000F))	/* Output divider for CPU clock frequency */
#define PLL_PCR_OUTDIV2(x)		(((x)&0x0000000F)<<4)	/* Output divider for internal bus clock frequency */
#define PLL_PCR_OUTDIV3(x)		(((x)&0x0000000F)<<8)	/* Output divider for Flexbus clock frequency */
#define PLL_PCR_OUTDIV4(x)		(((x)&0x0000000F)<<12)	/* Output divider for PCI clock frequency */
#define PLL_PCR_OUTDIV5(x)		(((x)&0x0000000F)<<16)	/* Output divider for USB clock frequency */
#define PLL_PCR_PFDR(x)			(((x)&0x000000FF)<<24)	/* Feedback divider for VCO frequency */
#define PLL_PCR_PFDR_MASK		(0x000F0000)
#define PLL_PCR_OUTDIV5_MASK		(0x000F0000)
#define PLL_PCR_OUTDIV4_MASK		(0x0000F000)
#define PLL_PCR_OUTDIV3_MASK		(0x00000F00)
#define PLL_PCR_OUTDIV2_MASK		(0x000000F0)
#define PLL_PCR_OUTDIV1_MASK		(0x0000000F)

/* Bit definitions and macros for PSR */
#define PLL_PSR_LOCKS			(0x00000001)	/* PLL lost lock - sticky */
#define PLL_PSR_LOCK			(0x00000002)	/* PLL lock status */
#define PLL_PSR_LOLIRQ			(0x00000004)	/* PLL loss-of-lock interrupt enable */
#define PLL_PSR_LOLRE			(0x00000008)	/* PLL loss-of-lock reset enable */

/*********************************************************************
* PCI
*********************************************************************/

/* Bit definitions and macros for SCR */
#define PCI_SCR_PE			(0x80000000)	/* Parity Error detected */
#define PCI_SCR_SE			(0x40000000)	/* System error signalled */
#define PCI_SCR_MA			(0x20000000)	/* Master aboart received */
#define PCI_SCR_TR			(0x10000000)	/* Target abort received */
#define PCI_SCR_TS			(0x08000000)	/* Target abort signalled */
#define PCI_SCR_DT			(0x06000000)	/* PCI_DEVSEL timing */
#define PCI_SCR_DP			(0x01000000)	/* Master data parity err */
#define PCI_SCR_FC			(0x00800000)	/* Fast back-to-back */
#define PCI_SCR_R			(0x00400000)	/* Reserved */
#define PCI_SCR_66M			(0x00200000)	/* 66Mhz */
#define PCI_SCR_C			(0x00100000)	/* Capabilities list */
#define PCI_SCR_F			(0x00000200)	/* Fast back-to-back enable */
#define PCI_SCR_S			(0x00000100)	/* SERR enable */
#define PCI_SCR_ST			(0x00000080)	/* Addr and Data stepping */
#define PCI_SCR_PER			(0x00000040)	/* Parity error response */
#define PCI_SCR_V			(0x00000020)	/* VGA palette snoop enable */
#define PCI_SCR_MW			(0x00000010)	/* Memory write and invalidate enable */
#define PCI_SCR_SP			(0x00000008)	/* Special cycle monitor or ignore */
#define PCI_SCR_B			(0x00000004)	/* Bus master enable */
#define PCI_SCR_M			(0x00000002)	/* Memory access control */
#define PCI_SCR_IO			(0x00000001)	/* I/O access control */

#define PCI_CR1_BIST(x)			((x & 0xFF) << 24)	/* Built in self test */
#define PCI_CR1_HDR(x)			((x & 0xFF) << 16)	/* Header type */
#define PCI_CR1_LTMR(x)			((x & 0xF8) << 8)	/* Latency timer */
#define PCI_CR1_CLS(x)			(x & 0x0F)	/* Cache line size */

#define PCI_BAR_BAR0(x)			(x & 0xFFFC0000)
#define PCI_BAR_BAR1(x)			(x & 0xFFF00000)
#define PCI_BAR_BAR2(x)			(x & 0xFFC00000)
#define PCI_BAR_BAR3(x)			(x & 0xFF000000)
#define PCI_BAR_BAR4(x)			(x & 0xF8000000)
#define PCI_BAR_BAR5(x)			(x & 0xE0000000)
#define PCI_BAR_PREF			(0x00000004)	/* Prefetchable access */
#define PCI_BAR_RANGE			(0x00000002)	/* Fixed to 00 */
#define PCI_BAR_IO_M			(0x00000001)	/* IO / memory space */

#define PCI_CR2_MAXLAT(x)		((x & 0xFF) << 24)	/* Maximum latency */
#define PCI_CR2_MINGNT(x)		((x & 0xFF) << 16)	/* Minimum grant */
#define PCI_CR2_INTPIN(x)		((x & 0xFF) << 8)	/* Interrupt Pin */
#define PCI_CR2_INTLIN(x)		(x & 0xFF)	/* Interrupt Line */

#define PCI_GSCR_DRD			(0x80000000)	/* Delayed read discarded */
#define PCI_GSCR_PE			(0x20000000)	/* PCI_PERR detected */
#define PCI_GSCR_SE			(0x10000000)	/* SERR detected */
#define PCI_GSCR_ER			(0x08000000)	/* Error response detected */
#define PCI_GSCR_DRDE			(0x00008000)	/* Delayed read discarded enable */
#define PCI_GSCR_PEE			(0x00002000)	/* PERR detected interrupt enable */
#define PCI_GSCR_SEE			(0x00001000)	/* SERR detected interrupt enable */
#define PCI_GSCR_PR			(0x00000001)	/* PCI reset */

#define PCI_TCR1_LD			(0x01000000)	/* Latency rule disable */
#define PCI_TCR1_PID			(0x00020000)	/* Prefetch invalidate and disable */
#define PCI_TCR1_P			(0x00010000)	/* Prefetch reads */
#define PCI_TCR1_WCD			(0x00000100)	/* Write combine disable */

#define PCI_TCR1_B5E			(0x00002000)	/*  */
#define PCI_TCR1_B4E			(0x00001000)	/*  */
#define PCI_TCR1_B3E			(0x00000800)	/*  */
#define PCI_TCR1_B2E			(0x00000400)	/*  */
#define PCI_TCR1_B1E			(0x00000200)	/*  */
#define PCI_TCR1_B0E			(0x00000100)	/*  */
#define PCI_TCR1_CR			(0x00000001)	/*  */

#define PCI_TBATR_BAT(x)		((x & 0xFFF) << 20)
#define PCI_TBATR_EN			(0x00000001)	/* Enable */

#define PCI_IWCR_W0C_IO			(0x08000000)	/* Windows Maps to PCI I/O */
#define PCI_IWCR_W0C_PRC_RDMUL		(0x04000000)	/* PCI Memory Read multiple */
#define PCI_IWCR_W0C_PRC_RDLN		(0x02000000)	/* PCI Memory Read line */
#define PCI_IWCR_W0C_PRC_RD		(0x00000000)	/* PCI Memory Read */
#define PCI_IWCR_W0C_EN			(0x01000000)	/* Enable - Register initialize */
#define PCI_IWCR_W1C_IO			(0x00080000)	/* Windows Maps to PCI I/O */
#define PCI_IWCR_W1C_PRC_RDMUL		(0x00040000)	/* PCI Memory Read multiple */
#define PCI_IWCR_W1C_PRC_RDLN		(0x00020000)	/* PCI Memory Read line */
#define PCI_IWCR_W1C_PRC_RD		(0x00000000)	/* PCI Memory Read */
#define PCI_IWCR_W1C_EN			(0x00010000)	/* Enable - Register initialize */
#define PCI_IWCR_W2C_IO			(0x00000800)	/* Windows Maps to PCI I/O */
#define PCI_IWCR_W2C_PRC_RDMUL		(0x00000400)	/* PCI Memory Read multiple */
#define PCI_IWCR_W2C_PRC_RDLN		(0x00000200)	/* PCI Memory Read line */
#define PCI_IWCR_W2C_PRC_RD		(0x00000000)	/* PCI Memory Read */
#define PCI_IWCR_W2C_EN			(0x00000100)	/* Enable - Register initialize */

#define PCI_ICR_REE			(0x04000000)	/* Retry error enable */
#define PCI_ICR_IAE			(0x02000000)	/* Initiator abort enable */
#define PCI_ICR_TAE			(0x01000000)	/* Target abort enable */

#define PCI_IDR_DEVID			(

/********************************************************************/

#endif				/* __MCF5445X__ */
