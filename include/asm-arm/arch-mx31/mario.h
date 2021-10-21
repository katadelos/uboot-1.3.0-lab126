#define SPBA0_BASE_ADDR			0x50000000
#define SPBA_CTRL_BASE_ADDR		(SPBA0_BASE_ADDR + 0x0003C000)

#define MXC_SPBA_RAR_MASK       0x7

/*!
 * Defines three SPBA masters: A - ARM, C - SDMA (no master B for MX31)
 */
enum spba_masters {
        SPBA_MASTER_A = 1,
        SPBA_MASTER_B = 2,
        SPBA_MASTER_C = 4,
};
#define SPBA_SDHC1      0x04

#define CSPI2_BASE_ADDR			(SPBA0_BASE_ADDR + 0x00010000)
#define MMC_SDHC1_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00004000)
