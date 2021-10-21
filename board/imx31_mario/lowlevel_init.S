/*
 *
 * (C) 2007. Embedded Alley Solutions, Inc. 
 * (c) 2007 Pengutronix, Sascha Hauer <s.hauer@...>
 *
 * Based on imx31lite code.
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

#include <asm/arch/mx31-regs.h>

.macro REG reg, val
 ldr r2, =\reg
 ldr r3, =\val
 str r3, [r2]
.endm

.macro REG8 reg, val
 ldr r2, =\reg
 ldr r3, =\val
 strb r3, [r2]
.endm

.macro DELAY loops
 ldr r2, =\loops
1:
 subs r2, r2, #1
 nop
 bcs 1b
.endm

.globl lowlevel_init
lowlevel_init:
	mov     r2, pc
	lsr     r2, r2, #28
	cmp     r2, #0xa  	// Are we running in RAM ?
	bne     __skip 
/* Run from RAM .. 
   Assume BDI did all init for us.
*/ 

// First set up the clocks
/* TODO: comes up as 670MHz when in FPM/CKIL mode. we need new divisors */
REG 0x53F80000  0x074b0b7b				// CCMR
REG 0x53FC0000  0x040					// IPU_CONF

// 398MHz
REG  0x53F80004 0xFF871650             	// PDR0
REG  0x53F80010 0x00331C23				// MPCTL

// 531MHz
//REG  0x53F80004 0xFF871D58             	// PDR0 : 133-133-66
//REG  0x53F80010 0x000c2803				// MPCTL


//REG  0xb8002000 0x0000CC03             // Start 16 bit NorFlash Initialization on CS0
//REG  0xb8002004 0xa0330D01
//REG  0xb8002008 0x00220800


REG 0xb8002000  0x00006602
REG 0xb8002004  0x00000501
REG 0xb8002008  0x00000000 



#if PHYS_SDRAM_1_SIZE == SZ_128M
// Initialization script for 32 bit DDR on Tortola EVB
// 128M
REG  0xB8001010 0x00000002             	// ESDMISC - reset
REG  0xB8001010 0x00000004             	// ESDMISC - MDDR
REG  0xB8001004 0x007FFF7F				// ESDCFG0 - timings
/* Enhanced SDRAM Configuration Register 0 (ESDCFG0)
 * tXP=3 tWTR=1 tRP=3 tMRD=3 tWR=1 tRAS=7 tRRD=3 tCAS=3 tRCD=7 tRC=15
 *  tXP - 4 clock delay before new COMMAND issued to LPDDR after power down mode exit.
 *  tWTR - 2 clock WRITE to READ Command Delay.
 *  tRP - 4 clock Row Precharge Delay.
 *  tMRD - 4 clock SDRAM Load Mode Register to ACTIVE Command.
 *  tWR - SDRAM to PRECHARGE Command. (LPDDR: 3 clocks; SDRAM: 2 clocks)
 *  tRAS - 8 clock SDRAM ACTIVE to PRECHARGE Command.
 *  tRRD - 4 clock active to active (different banks)
 *  tCAS - 3 clocks CAS Latency.
 *  tRCD - 8 clock SDRAM Row to Column Delay.
 *  tRC - 16 clock SDRAM Row Cycle Delay.
 */

/* delay at least 200uS before precharge */
	ldr r1, =80000
1:
	subs r1, r1, #1
	bne 1b

REG  0xB8001000 0x92100000				// ESDCTL0 - PRECHARGE
REG  0x80000f00 0x12344321
REG  0xB8001000 0xa2100000				// ESDCTL0 - AUTO REFRESH
REG  0x80000000 0x12344321				// write word to SDRAM
REG  0x80000000 0x12344321				// write word to SDRAM
REG  0xB8001000 0xb2100000				// ESDCTL0 - LOAD MODE REGISTER
REG8 0x80000033 0xda					// write byte to SDRAM
REG8 0x81000000 0xff					// write byte to SDRAM
REG  0xB8001000 0x82226080				// ESDCTL0 - Normal mode
/* column address width = 10 (1K)
 * row addresses width = 13
 * memory data width = 32-bit
 * refresh rate = 4096 rows / 64mS @ 32kHz (15.52 uS)
 * power down timer = any time no banks are active (01)
 * full page = not set to full page
 * burst length  = set (8)
 * precharge timer = disabled
 */

REG  0x80000000 0xDEADBEEF				// write byte to SDRAM
#elif PHYS_SDRAM_1_SIZE == SZ_256M
// Micron Mobile DDR SDRAM
// 256M total
// MT46H64M16LF - 16 Meg x 16 x 4 banks
REG  0xB8001010 0x00000002             	// ESDMISC - reset
REG  0xB8001010 0x00000004             	// ESDMISC - mddr
REG  0xB8001004 0x007FFF7F				// ESDCFG0 - timings

/* delay at least 200uS before precharge */
	ldr r1, =80000
1:
	subs r1, r1, #1
	bne 1b

REG  0xB8001000 0x93100000				// ESDCTL0 - PRECHARGE
REG  0x80000f00 0x12344321
REG  0xB8001000 0xa3100000				// ESDCTL0 - AUTO REFRESH
REG  0x80000000 0x12344321				// write word to SDRAM
REG  0x80000000 0x12344321				// write word to SDRAM
REG  0xB8001000 0xb3100000				// ESDCTL0 - LOAD MODE REGISTER
REG8 0x80000033 0xda					// write byte to SDRAM
REG8 0x81000000 0xff					// write byte to SDRAM
REG  0xB8001000 0x83236080				// ESDCTL0 - Normal Mode
/* column address width = 10 (1K)
 * row addresses width = 14
 * memory data width = 32-bit
 * refresh rate = 8192 rows / 64mS @ 32kHz (7.81 uS)
 * power down timer = any time no banks are active (01)
 * full page = not set to full page
 * burst length  = set (8)
 * precharge timer = disabled
 */

REG  0x80000000 0xDEADBEEF				// write byte to SDRAM
#else
#error you must compile with PHYS_SDRAM_1_SIZE set to a supported value
#endif

// Configure AIPS1
REG 0x43F00040 0x0
REG 0x43F00044 0x0
REG 0x43F00048 0x0
REG 0x43F0004C 0x0
REG 0x43F00050 0x0
REG 0x43F00000 0x77777777
REG 0x43F00004 0x77777777

// Configure AIPS2
REG 0x53F00040 0x0
REG 0x53F00044 0x0
REG 0x53F00048 0x0
REG 0x53F0004C 0x0
REG 0x53F00050 0x0
REG 0x53F00000 0x77777777
REG 0x53F00004 0x77777777

__skip:	
 mov pc, lr 
