
/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#undef DEBUG

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/arch/crm_regs.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define MPDGCTRL0_PHY0 0x021b083c
#define MPDGCTRL1_PHY0 0x021b0840
#define MPDGCTRL0_PHY1 0x021b483c
#define MPDGCTRL1_PHY1 0x021b4840
#define MPRDDLCTL_PHY0 0x021b0848
#define MPRDDLCTL_PHY1 0x021b4848
#define MPWRDLCTL_PHY0 0x021b0850
#define MPWRDLCTL_PHY1 0x021b4850

#define MDPDC_OFFSET 0x0004
#define MDCFG0_OFFSET 0x000C
#define MDCFG1_OFFSET 0x0010
#define MDCFG2_OFFSET 0x0014
#define MAPSR_OFFSET 0x0404
#define MDREF_OFFSET 0x0020
#define MDASP_OFFSET 0x0040
#define MPZQHWCTRL_OFFSET 0x0800
#define MDSCR_OFFSET 0x001C
#define MPWLGCR_OFFSET 0x0808
#define MPWLDECTRL0_OFFSET 0x080c
#define MPWLDECTRL1_OFFSET 0x0810
#define MDCTL_OFFSET 0x0000
#define MDMISC_OFFSET 0x0018
#define MPPDCMPR1_OFFSET 0x088C
#define MPSWDAR_OFFSET 0x0894
#define MPRDDLCTL_OFFSET 0x0848
#define MPMUR_OFFSET 0x08B8
#define MPDGCTRL0_OFFSET 0x083C
#define MPDGHWST0_OFFSET 0x087C
#define MPDGHWST1_OFFSET 0x0880
#define MPDGHWST2_OFFSET 0x0884
#define MPDGHWST3_OFFSET 0x0888
#define MPDGCTRL1_OFFSET 0x0840
#define MPRDDLHWCTL_OFFSET 0x0860
#define MPWRDLCTL_OFFSET 0x0850
#define MPWRDLHWCTL_OFFSET 0x0864

#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0 0x020E05A8
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1 0x020E05B0
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2 0x020E0524
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3 0x020E051C
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4 0x020E0518
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5 0x020E050C
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6 0x020E05B8
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7 0x020E05C0


#define DISP_LINE_LEN 16


static void reg32_write(unsigned int addr, unsigned int data) {
	writel(data, addr);
}

static unsigned int reg32_read(unsigned int addr) {
	return readl(addr);
}

static void reg32setbit(unsigned int addr, unsigned int bit) {
	u32 data = readl(addr);
	data |= (1 << bit);
	writel(data, addr);
}
static void reg32clrbit(unsigned int addr, unsigned int bit) {
	u32 data = readl(addr);
	data &= ~(1 << bit);
	writel(data, addr);
}


#define BIT(n,x) ( ( (x) >> (n) ) & 1 )

static int do_tune_mww(ulong addr)
{
	ulong  writeval, count, feedback, sum1;
	int	size;

	debug("write starting at %08lx\n", addr);

	/* Get the value to write.  */
	writeval = 0x0;

	count = 0x20000;
	size = 2;
	sum1 = 0;

	while (count-- > 0) {
		feedback = (BIT(14,writeval) == BIT(13,writeval));
		writeval = (writeval<<1) + feedback;
		writeval &= 0x7FFF;
		*((ushort  *)addr) = (ushort )writeval;
		sum1 += (ushort )writeval;
		addr += size;
	}
	debug("checksum: %08lx\n", sum1);
	return 0;
}

static int do_tune_mrr(ulong addr)
{
	ulong  writeval, count, feedback, sum1, sum2, size;

	debug("read starting at %08lx\n", addr);

	/* Get the value to write.  */
	writeval = 0x0;
	size = 2;

	sum1 = 0;
	sum2 = 0;
	count = 0x20000;
	feedback = 0;
	while (count-- > 0) {
		feedback = (BIT(14,writeval) == BIT(13,writeval));
		writeval = (writeval<<1) + feedback;
		writeval &= 0x7FFF;
		sum1 += (ushort )writeval;
		sum2 += *((ushort  *)addr);
		addr += size;
	}
	debug("computed: %08lx, readback: %08lx\n", sum1, sum2);

	return sum1 == sum2 ? 0 : 1;
}


/*
 * Perform a memory test. A more complete alternative test can be
 * configured using CONFIG_SYS_ALT_MEMTEST. The complete test loops until
 * interrupted by ctrl-c or by a failure of one of the sub-tests.
 */
int mx6_do_tune_mtest(int iteration_limit)
{
	vu_long	*addr, *start, *end;
	ulong	val;
	ulong	readback;
	ulong	errs = 0;
	int iterations = 1;

	ulong	incr;
	ulong	pattern;

	start = (ulong *)CONFIG_SYS_MEMTEST_START;
	end = (ulong *)(CONFIG_SYS_MEMTEST_END);
	pattern = 0;

	incr = 1;
	for (;;) {
		if (ctrlc()) {
			putc('\n');
			return 1;
		}

		if (iteration_limit && iterations > iteration_limit) {
			printf("Tested %d iteration(s) with %lu errors.\n",
				iterations-1, errs);
			return errs != 0;
		}
		++iterations;

		printf("\rPattern %08lX  Writing..."
			"%12s"
			"\b\b\b\b\b\b\b\b\b\b",
			pattern, "");

		for (addr=start,val=pattern; addr<end; addr++) {
			*addr = val;
			val  += incr;
		}

		puts("Reading...");

		for (addr=start,val=pattern; addr<end; addr++) {
			readback = *addr;
			if (readback != val) {
				printf ("\nMem error @ 0x%08X: "
					"found %08lX, expected %08lX\n",
					(uint)(uintptr_t)addr, readback, val);
				errs++;
				if (ctrlc()) {
					putc ('\n');
					return 1;
				}
			}
			val += incr;
		}

		/*
		 * Flip the pattern each time to make lots of zeros and
		 * then, the next time, lots of ones.  We decrement
		 * the "negative" patterns and increment the "positive"
		 * patterns to preserve this feature.
		 */
		if(pattern & 0x80000000)
			pattern = -pattern;	/* complement & increment */
		else
			pattern = ~pattern;
		incr = -incr;
	}
	return 0;	/* not reached */
}

static int do_tune_wcal(void)
{
	int temp1, temp2;
	int errorcount = 0;
	int ddr_mr1 = 0x04;
	int withprint = 1;
	int ldectrl[4];

	ddr_mr1 = 0x04;

	printf("MMDC_MPWLDECTRL0 before write level cal: 0x%08X\n",
		reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET));
	printf("MMDC_MPWLDECTRL1 before write level cal: 0x%08X\n",
		reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET));
	printf("MMDC_MPWLDECTRL0 before write level cal: 0x%08X\n",
		reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET));
	printf("MMDC_MPWLDECTRL1 before write level cal: 0x%08X\n",
		reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET));
	/*
	 * Stash old values in case calibration fails,
	 * we need to restore them
	 */
	ldectrl[0] = reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET);
	ldectrl[1] = reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET);
	ldectrl[2] = reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET);
	ldectrl[3] = reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET);

	/* disable DDR logic power down timer */
	reg32_write((MMDC_P0_BASE_ADDR + MDPDC_OFFSET),
		reg32_read((MMDC_P0_BASE_ADDR + MDPDC_OFFSET)) & 0xffff00ff);
	/* disable Adopt power down timer */
	reg32_write((MMDC_P0_BASE_ADDR + MAPSR_OFFSET),
		reg32_read((MMDC_P0_BASE_ADDR + MAPSR_OFFSET)) | 0x1);

	printf("%s: Start write leveling calibration\n", __func__);
	/*
	 * 2. disable auto refresh and ZQ calibration
	 * before proceeding with Write Leveling calibration
	 */
	temp1 = reg32_read(MMDC_P0_BASE_ADDR + MDREF_OFFSET);
	reg32_write((MMDC_P0_BASE_ADDR + MDREF_OFFSET), 0x0000C000);
	temp2 = reg32_read(MMDC_P0_BASE_ADDR + MPZQHWCTRL_OFFSET);
	printf("%s: MPZQHWCTRL = 0x%08x\n", __func__, temp2);
	reg32_write((MMDC_P0_BASE_ADDR + MPZQHWCTRL_OFFSET), temp2 & ~(0x3));

	/* 3. increase walat and ralat to maximum */
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 6); //set RALAT to max
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 7);
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 8);
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 16); //set WALAT to max
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 17);

	reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 6); //set RALAT to max
	reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 7);
	reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 8);
	reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 16); //set WALAT to max
	reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 17);

	/*
	 * 4 & 5. Configure the external DDR device to enter write-leveling
	 * mode through Load Mode Register command.
	 * Register setting:
	 * Bits[31:16] MR1 value (0x0080 write leveling enable)
	 * Bit[9] set WL_EN to enable MMDC DQS output
	 * Bits[6:4] set CMD bits for Load Mode Register programming
	 * Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
	 */
	reg32_write(MMDC_P0_BASE_ADDR + MDSCR_OFFSET, 0x00808231);

	/* 6. Activate automatic calibration by setting MPWLGCR[HW_WL_EN] */
	reg32_write(MMDC_P0_BASE_ADDR + MPWLGCR_OFFSET, 0x00000001);

	/*
	 * 7. Upon completion of this process the MMDC de-asserts
	 * the MPWLGCR[HW_WL_EN]
	 */
	while (reg32_read(MMDC_P0_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000001)
		if (withprint)
			printf(".");

	/*
	 * 8. check for any errors: check both PHYs for x64 configuration,
	 * if x32, check only PHY0
	 */
	if ((reg32_read(MMDC_P0_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000F00) ||
	    (reg32_read(MMDC_P1_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000F00))
		errorcount++;
	printf("Write leveling calibration completed, errcount: %d\n",
			errorcount);

	/* check to see if cal failed */
	if ((reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET)) == 0x001F001F
	&& (reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET)) == 0x001F001F
	&& (reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET)) == 0x001F001F
	&& (reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET)) == 0x001F001F) {
		printf("Cal seems to have soft-failed due to memory "
			"not supporting write leveling on all channels. "
			"Restoring original write leveling values.\n");
		reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET, ldectrl[0]);
		reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET, ldectrl[1]);
		reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET, ldectrl[2]);
		reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET, ldectrl[3]);
		errorcount++;
	}

	/*
	 * User should issue MRS command to exit write leveling mode
	 * through Load Mode Register command
	 * Register setting:
	 * Bits[31:16] MR1 value "ddr_mr1" value from initialization
	 * Bit[9] clear WL_EN to disable MMDC DQS output
	 * Bits[6:4] set CMD bits for Load Mode Register programming
	 * Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
	 */
	reg32_write( MMDC_P0_BASE_ADDR + MDSCR_OFFSET,((ddr_mr1 << 16)+0x8031));
	/* re-enable to auto refresh and zq cal */
	reg32_write((MMDC_P0_BASE_ADDR + MDREF_OFFSET), temp1);
	reg32_write((MMDC_P0_BASE_ADDR + MPZQHWCTRL_OFFSET), temp2);
	udelay(5000);
	printf("MMDC_MPWLDECTRL0 after write level cal: 0x%08X\n",
		reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET));
	printf("MMDC_MPWLDECTRL1 after write level cal: 0x%08X\n",
		reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET));
	printf("MMDC_MPWLDECTRL0 after write level cal: 0x%08X\n",
		reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET));
	printf("MMDC_MPWLDECTRL1 after write level cal: 0x%08X\n",
		reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET));
	/* enable DDR logic power down timer: */
	reg32_write((MMDC_P0_BASE_ADDR + MDPDC_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MDPDC_OFFSET)) | 0x00005500);
	/* enable Adopt power down timer: */
	reg32_write((MMDC_P0_BASE_ADDR + MAPSR_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MAPSR_OFFSET)) & 0xfffffff7);
	reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET),0); //clear CON_REQ
	return errorcount;
}


static int modify_dg_result(int reg_st0, int reg_st1, int reg_ctrl)
{
	/*
	 * DQS gating absolute offset should be modified from reflecting
	 * (HW_DG_LOWx + HW_DG_UPx)/2 to reflecting (HW_DG_UPx - 0x80)
	 */
	int dg_tmp_val0,dg_tmp_val1, dg_tmp_val2;
	int dg_dl_abs_offset0, dg_dl_abs_offset1;
	int dg_hc_del0, dg_hc_del1;
	dg_tmp_val0 = ((reg32_read(reg_st0) & 0x07ff0000) >>16) - 0xc0;
	dg_tmp_val1 = ((reg32_read(reg_st1) & 0x07ff0000) >>16) - 0xc0;
	dg_dl_abs_offset0 = dg_tmp_val0 & 0x7f;
	dg_hc_del0 = (dg_tmp_val0 & 0x780) << 1;
	dg_dl_abs_offset1 = dg_tmp_val1 & 0x7f;
	dg_hc_del1 = (dg_tmp_val1 & 0x780) << 1;
	dg_tmp_val2 = dg_dl_abs_offset0 + dg_hc_del0 + ((dg_dl_abs_offset1 +
						   dg_hc_del1) << 16);
	reg32_write((reg_ctrl),
		reg32_read((reg_ctrl)) & 0xf0000000);
	reg32_write((reg_ctrl),
		reg32_read((reg_ctrl)) & 0xf0000000);
	reg32_write((reg_ctrl),
		reg32_read((reg_ctrl)) | dg_tmp_val2);
	return 0;
}

static int do_tune_delays(void)
{
	int temp1;
	int data_bus_size;
	int temp_ref;
	int cs0_enable = 0;
	int cs1_enable = 0;
	int cs0_enable_initial = 0;
	int cs1_enable_initial = 0;

	int PDDWord = 0x00FFFF00; /* best so far, place into MPPDCMPR1 */
	int errorcount = 0;
	int withprint = 1;
	unsigned int initdelay = 0x40404040;

	/* check to see which chip selects are enabled */
	cs0_enable_initial = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET)
			& 0x80000000) >> 31;
	cs1_enable_initial = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET)
			& 0x40000000) >> 30;
	debug("init cs0: %d cs1: %d\n", cs0_enable_initial, cs1_enable_initial);
	/* disable DDR logic power down timer: */
	reg32_write((MMDC_P0_BASE_ADDR + MDPDC_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MDPDC_OFFSET)) & 0xffff00ff);

	/* disable Adopt power down timer: */
	reg32_write((MMDC_P0_BASE_ADDR + MAPSR_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MAPSR_OFFSET)) | 0x1);

	/* set DQS pull ups */
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0,
		reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1,
		reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2,
		reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3,
		reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4,
		reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5,
		reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6,
		reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6) | 0x7000);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7,
		reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7) | 0x7000);
	/* set RALAT to max */
	temp1 = reg32_read(MMDC_P0_BASE_ADDR + MDMISC_OFFSET);
		reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 6);
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 7);
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 8);
	/* set WALAT to max */
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 16);
	reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 17);

	/* disable auto refresh before proceeding with calibration */
	temp_ref = reg32_read(MMDC_P0_BASE_ADDR + MDREF_OFFSET);
	reg32_write((MMDC_P0_BASE_ADDR + MDREF_OFFSET), 0x0000C000);
	/*
	 * Per the ref manual, issue one refresh cycle MDSCR[CMD]= 0x2,
	 * this also sets the CON_REQ bit.
	 */
	if (cs0_enable_initial == 1)
		reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x00008020);
	if (cs1_enable_initial == 1)
		reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x00008028);
	/* poll to make sure the con_ack bit was asserted */
	while (!(reg32_read((MMDC_P0_BASE_ADDR + MDSCR_OFFSET)) & 0x00004000))
		if (withprint)
			debug(".");
	/*
	 * Check MDMISC register CALIB_PER_CS to see which CS calibration
	 * is targeted to (under normal cases, it should be cleared
	 * as this is the default value, indicating calibration is directed
	 * to CS0).
	 * Disable the other chip select not being target for calibration
	 * to avoid any potential issues.  This will get re-enabled at end
	 * of calibration.
	 */
	if ((reg32_read(MMDC_P0_BASE_ADDR + MDMISC_OFFSET) & 0x00100000) == 0)
		/* clear SDE_1 */
		reg32clrbit((MMDC_P0_BASE_ADDR + MDCTL_OFFSET), 30);
	else
		/* clear SDE_0 */
		reg32clrbit((MMDC_P0_BASE_ADDR + MDCTL_OFFSET), 31);
	/*
	 * Check to see which chip selects are now enabled for
	 * the remainder of the calibration.
	 */
	cs0_enable = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET)
			& 0x80000000) >> 31;
	cs1_enable = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET)
			& 0x40000000) >> 30;
	debug("cal cs0: %d cs1: %d\n", cs0_enable, cs1_enable);
	/* check to see what is the data bus size: */
	data_bus_size = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET)
			& 0x30000) >> 16;
	debug("db size: %d\n", data_bus_size);
	/*
	 * Issue the Precharge-All command to the DDR device for both
	 * chip selects.  Note, CON_REQ bit should also remain set.
	 * If only using one chip select, then precharge only the desired
	 * chip select.
	 */
	if (cs0_enable == 1)
		/* CS0 */
		reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008050);
	if (cs1_enable == 1)
		/* CS1 */
		reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008058);

	/* Write the pre-defined value into MPPDCMPR1 */
	reg32_write((MMDC_P0_BASE_ADDR + MPPDCMPR1_OFFSET), PDDWord);
	/*
	 * Issue a write access to the external DDR device by setting
	 * the bit SW_DUMMY_WR (bit 0) in the MPSWDAR0 and then poll
	 * this bit until it clears to indicate completion of the write access.
	 */
	reg32setbit((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET), 0);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET)) & 0x00000001)
		if (withprint)
			debug(".");

	/* Set the RD_DL_ABS_OFFSET# bits to their default values
	 * (will be calibrated later in the read delay-line calibration).
	 * Both PHYs for x64 configuration, if x32, do only PHY0.
	 */
	reg32_write((MMDC_P0_BASE_ADDR + MPRDDLCTL_OFFSET), 0x40404040);
	if (data_bus_size == 0x2)
		reg32_write((MMDC_P1_BASE_ADDR + MPRDDLCTL_OFFSET), 0x40404040);
	/* Force a measurment, for previous delay setup to take effect. */
	reg32_write((MMDC_P0_BASE_ADDR + MPMUR_OFFSET), 0x800);
	if (data_bus_size == 0x2)
		reg32_write((MMDC_P1_BASE_ADDR + MPMUR_OFFSET), 0x800);

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Read DQS Gating calibration
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 */
	printf("Starting DQS gating calibration...\n");
	/*
	 * Reset the read data FIFOs (two resets); only need to issue reset
	 * to PHY0 since in x64 mode, the reset will also go to PHY1.
	 */

	 /* Read data FIFOs reset1.  */
	reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000)
		if (withprint)
			debug(".");

	/* Read data FIFOs reset2 */
	reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000)
		if (withprint)
			debug(".");

	/*
	 * Start the automatic read DQS gating calibration process by
	 * asserting MPDGCTRL0[HW_DG_EN] and MPDGCTRL0[DG_CMP_CYC]
	 * and then poll MPDGCTRL0[HW_DG_EN]] until this bit clears
	 * to indicate completion.
	 * Also, ensure that MPDGCTRL0[HW_DG_ERR] is clear to indicate
	 * no errors were seen during calibration.
	 */

	/*
	 * Set bit 30: chooses option to wait 32 cycles instead of
	 * 16 before comparing read data.
	 */
	reg32setbit((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET), 30);
	/* Set bit 28 to start automatic read DQS gating calibration */
	reg32setbit((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET), 28);
	/* Poll for completion.  MPDGCTRL0[HW_DG_EN] should be 0 */
	while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET))
			& 0x10000000) {
		if( withprint )
			debug( "." );
	}

	/*
	 * Check to see if any errors were encountered during calibration
	 * (check MPDGCTRL0[HW_DG_ERR]).
	 * Check both PHYs for x64 configuration, if x32, check only PHY0.
	 */
	if (data_bus_size == 0x2) {
		if ((reg32_read(MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)
					& 0x00001000) ||
			(reg32_read(MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET)
					& 0x00001000)) {
			errorcount++;
		}
	} else {
		if (reg32_read(MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)
				& 0x00001000) {
			errorcount++;
		}
	}
	debug("errorcount: %d\n", errorcount);
	/*
	 * DQS gating absolute offset should be modified from
	 * reflecting (HW_DG_LOWx + HW_DG_UPx)/2 to
	 * reflecting (HW_DG_UPx - 0x80)
	 */
	modify_dg_result(MMDC_P0_BASE_ADDR + MPDGHWST0_OFFSET,
		MMDC_P0_BASE_ADDR + MPDGHWST1_OFFSET,
		MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET);
	modify_dg_result(MMDC_P0_BASE_ADDR + MPDGHWST2_OFFSET,
		MMDC_P0_BASE_ADDR + MPDGHWST3_OFFSET,
		MMDC_P0_BASE_ADDR + MPDGCTRL1_OFFSET);
	if (data_bus_size == 0x2) {
		modify_dg_result((MMDC_P1_BASE_ADDR + MPDGHWST0_OFFSET),
			(MMDC_P1_BASE_ADDR + MPDGHWST1_OFFSET),
			(MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET));
		modify_dg_result((MMDC_P1_BASE_ADDR + MPDGHWST2_OFFSET),
			(MMDC_P1_BASE_ADDR + MPDGHWST3_OFFSET),
			(MMDC_P1_BASE_ADDR + MPDGCTRL1_OFFSET));
	}
	printf("DQS gating calibration completed, continuing...        \n");



	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Read delay Calibration
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 */
	printf("Starting read calibration...\n");
	/*
	 * Reset the read data FIFOs (two resets); only need to issue reset
	 * to PHY0 since in x64 mode, the reset will also go to PHY1.
	 */

	/* Read data FIFOs reset1 */
	reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
		reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET))
		| 0x80000000);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000)
		if (withprint)
			debug(".");
	
	/* Read data FIFOs reset2 */
	reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000)
		if(withprint)
			debug(".");
	
	/*
	 * 4. Issue the Precharge-All command to the DDR device for both
	 * chip selects.  If only using one chip select, then precharge
	 * only the desired chip select.
	 */
	if (cs0_enable == 1)
		/* CS0 */
		reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008050);
	while (!(reg32_read(MMDC_P0_BASE_ADDR + MDSCR_OFFSET) & 0x4000))
		debug( "x" );
	if (cs1_enable == 1)
		/* CS1 */
		reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008058);
	while (!(reg32_read(MMDC_P0_BASE_ADDR + MDSCR_OFFSET) & 0x4000))
		printf( "x" );

	/* *********** 5. 6. 7. set the pre-defined word ************ */
	reg32_write((MMDC_P0_BASE_ADDR + MPPDCMPR1_OFFSET), PDDWord);
	/*
	 * Issue a write access to the external DDR device by setting
	 * the bit SW_DUMMY_WR (bit 0) in the MPSWDAR0 and then poll
	 * this bit until it clears to indicate completion of the write access.
	 */
	reg32setbit((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET), 0);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET)) & 0x00000001) {
		if( withprint )
			debug( "." );
	}

	/* 8. set initial delays to center up dq in clock */
	reg32_write((MMDC_P0_BASE_ADDR + MPRDDLCTL_OFFSET), initdelay);
	if (data_bus_size == 0x2)
		reg32_write((MMDC_P1_BASE_ADDR + MPRDDLCTL_OFFSET), initdelay);
	debug("intdel0: %08x / intdel1: %08x\n",
		reg32_read(MMDC_P0_BASE_ADDR + MPRDDLCTL_OFFSET),
		reg32_read(MMDC_P1_BASE_ADDR + MPRDDLCTL_OFFSET));

	/*
	 * 9. Read delay-line calibration
	 * Start the automatic read calibration process by asserting
	 * MPRDDLHWCTL[HW_RD_DL_EN].
	 */
	reg32_write((MMDC_P0_BASE_ADDR + MPRDDLHWCTL_OFFSET), 0x00000030);

	/*
	 * 10. poll for completion
	 * MMDC indicates that the write data calibration had finished by
	 * setting MPRDDLHWCTL[HW_RD_DL_EN] = 0.   Also, ensure that
	 * no error bits were set.
	 */
	while (reg32_read((MMDC_P0_BASE_ADDR + MPRDDLHWCTL_OFFSET))
			& 0x00000010) {
		if( withprint )
			debug( "." );
	}

	/* check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (data_bus_size == 0x2) {
		if ((reg32_read(MMDC_P0_BASE_ADDR + MPRDDLHWCTL_OFFSET)
					& 0x0000000f) ||
			(reg32_read(MMDC_P1_BASE_ADDR + MPRDDLHWCTL_OFFSET)
					& 0x0000000f)) {
			errorcount++;
		}
	} else {
		if (reg32_read(MMDC_P0_BASE_ADDR + MPRDDLHWCTL_OFFSET)
				& 0x0000000f) {
			errorcount++;
		}
	}
	printf("errorcount: %d\n", errorcount);
	printf("Read calibration completed, continuing...        \n");


	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Write delay Calibration
	 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 */
	printf("Starting write calibration...\n");
	/*
	 * 3. Reset the read data FIFOs (two resets); only need to issue
	 * reset to PHY0 since in x64 mode, the reset will also go to PHY1.
	 */

	/* read data FIFOs reset1 */
	reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000)
		if (withprint)
			debug(".");

	/* read data FIFOs reset2 */
	reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000)
		if (withprint)
			debug(".");

	/*
	 * 4. Issue the Precharge-All command to the DDR device for both
	 * chip selects. If only using one chip select, then precharge
	 * only the desired chip select.
	 */
	if (cs0_enable == 1)
		/* CS0 */
		reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008050);
	if (cs1_enable == 1)
		/* CS1 */
		reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008058);

	/* *********** 5. 6. 7. set the pre-defined word ************ */
	reg32_write((MMDC_P0_BASE_ADDR + MPPDCMPR1_OFFSET), PDDWord);
	/*
	 * Issue a write access to the external DDR device by setting
	 * the bit SW_DUMMY_WR (bit 0) in the MPSWDAR0 and then poll this bit
	 * until it clears to indicate completion of the write access.
	 */
	reg32setbit((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET), 0);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET)) & 0x00000001)
		if (withprint)
			debug(".");

	/*
	 * 8. Set the WR_DL_ABS_OFFSET# bits to their default values.
	 * Both PHYs for x64 configuration, if x32, do only PHY0.
	 */
	reg32_write((MMDC_P0_BASE_ADDR + MPWRDLCTL_OFFSET), initdelay);
	if (data_bus_size == 0x2)
		reg32_write((MMDC_P1_BASE_ADDR + MPWRDLCTL_OFFSET), initdelay);
	debug("intdel0: %08x / intdel1: %08x\n",
		reg32_read(MMDC_P0_BASE_ADDR + MPWRDLCTL_OFFSET),
		reg32_read(MMDC_P1_BASE_ADDR + MPWRDLCTL_OFFSET));

	/*
	 * XXX This isn't in the manual. Force a measurment,
	 * for previous delay setup to effect.
	 */
	reg32_write((MMDC_P0_BASE_ADDR + MPMUR_OFFSET), 0x800);
	if (data_bus_size == 0x2)
		reg32_write((MMDC_P1_BASE_ADDR + MPMUR_OFFSET), 0x800);

	/*
	 * 9. 10. Start the automatic write calibration process
	 * by asserting MPWRDLHWCTL0[HW_WR_DL_EN].
	 */
	reg32_write((MMDC_P0_BASE_ADDR + MPWRDLHWCTL_OFFSET), 0x00000030);

	/*
	 * Poll for completion.
	 * MMDC indicates that the write data calibration had finished
	 * by setting MPWRDLHWCTL[HW_WR_DL_EN] = 0.
	 * Also, ensure that no error bits were set.
	 */
	while (reg32_read((MMDC_P0_BASE_ADDR + MPWRDLHWCTL_OFFSET))
			& 0x00000010)
		if(withprint)
			debug(".");
	/* Check both PHYs for x64 configuration, if x32, check only PHY0 */
	if (data_bus_size == 0x2) {
		if ((reg32_read(MMDC_P0_BASE_ADDR + MPWRDLHWCTL_OFFSET)
					& 0x0000000f) ||
			(reg32_read(MMDC_P1_BASE_ADDR + MPWRDLHWCTL_OFFSET)
					& 0x0000000f)) {
			errorcount++;
		}
	} else {
		if (reg32_read(MMDC_P0_BASE_ADDR + MPWRDLHWCTL_OFFSET)
				& 0x0000000f) {
			errorcount++;
		}
	}
	printf("errorcount: %d\n", errorcount);
	printf("Write calibration completed, continuing...        \n");

	/*
	 * Reset the read data FIFOs (two resets); only need to issue
	 * reset to PHY0 since in x64 mode, the reset will also go to PHY1.
	 */

	/* read data FIFOs reset1 */
	reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
	while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000)
		if (withprint)
			debug(".");

	/* read data FIFOs reset2 */
	reg32_write((MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET),
	reg32_read((MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
	while (reg32_read((MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000)
		if (withprint)
			debug(".");

	/* Enable DDR logic power down timer: */
	reg32_write((MMDC_P0_BASE_ADDR + MDPDC_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MDPDC_OFFSET)) | 0x00005500);
	/* Enable Adopt power down timer: */
	reg32_write((MMDC_P0_BASE_ADDR + MAPSR_OFFSET),
	reg32_read((MMDC_P0_BASE_ADDR + MAPSR_OFFSET)) & 0xfffffff7);
	/* Restore MDMISC value (RALAT, WALAT) */
	reg32_write((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), temp1);
	/* Clear DQS pull ups */
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0,
	reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1,
	reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2,
	reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3,
	reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4,
	reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5,
	reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6,
	reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6) & 0xffff0fff);
	reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7,
	reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7) & 0xffff0fff);

	/* Re-enable SDE (chip selects) if they were set initially */
	if (cs1_enable_initial == 1)
		/* Set SDE_1 */
		reg32setbit((MMDC_P0_BASE_ADDR + MDCTL_OFFSET), 30);
	if (cs0_enable_initial == 1)
		/* Set SDE_0 */
		reg32setbit((MMDC_P0_BASE_ADDR + MDCTL_OFFSET), 31);
	/* Re-enable to auto refresh */
	reg32_write((MMDC_P0_BASE_ADDR + MDREF_OFFSET), temp_ref);
	/* Clear the MDSCR (including the con_req bit) */
	reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x0); // CS0
	/* Poll to make sure the con_ack bit is clear */
	while ((reg32_read(MMDC_P0_BASE_ADDR + MDSCR_OFFSET) & 0x00004000))
		if (withprint)
			debug(".");

	/*
	 * Print out the registers that were updated as a result
	 * of the calibration process.
	 */
	printf("MMDC registers updated from calibration \n");
	printf("\nRead DQS Gating calibration\n");
	printf("MPDGCTRL0 PHY0 (0x021b083c) = 0x%08X\n",
			reg32_read(MPDGCTRL0_PHY0));
	printf("MPDGCTRL1 PHY0 (0x021b0840) = 0x%08X\n",
			reg32_read(MPDGCTRL1_PHY0));
	printf("MPDGCTRL0 PHY1 (0x021b483c) = 0x%08X\n",
			reg32_read(MPDGCTRL0_PHY1));
	printf("MPDGCTRL1 PHY1 (0x021b4840) = 0x%08X\n",
			reg32_read(MPDGCTRL1_PHY1));
	printf("\nRead calibration\n");
	printf("MPRDDLCTL PHY0 (0x021b0848) = 0x%08X\n",
			reg32_read(MPRDDLCTL_PHY0));
	printf("MPRDDLCTL PHY1 (0x021b4848) = 0x%08X\n",
			reg32_read(MPRDDLCTL_PHY1));
	printf("\nWrite calibration\n");
	printf("MPWRDLCTL PHY0 (0x021b0850) = 0x%08X\n",
			reg32_read(MPWRDLCTL_PHY0));
	printf("MPWRDLCTL PHY1 (0x021b4850) = 0x%08X\n",
			reg32_read(MPWRDLCTL_PHY1));
	printf("\n");
	/*
	 * Registers below are for printfging purposes.  These print out
	 * the upper and lower boundaries captured during
	 * read DQS gating calibration.
	 */

	debug("Status registers, upper and lower bounds, "
			"for read DQS gating.\n");
	debug("MPDGHWST0 PHY0 (0x021b087c) = 0x%08X\n", reg32_read(0x021b087c));
	debug("MPDGHWST1 PHY0 (0x021b0880) = 0x%08X\n", reg32_read(0x021b0880));
	debug("MPDGHWST2 PHY0 (0x021b0884) = 0x%08X\n", reg32_read(0x021b0884));
	debug("MPDGHWST3 PHY0 (0x021b0888) = 0x%08X\n", reg32_read(0x021b0888));
	debug("MPDGHWST0 PHY1 (0x021b487c) = 0x%08X\n", reg32_read(0x021b487c));
	debug("MPDGHWST1 PHY1 (0x021b4880) = 0x%08X\n", reg32_read(0x021b4880));
	debug("MPDGHWST2 PHY1 (0x021b4884) = 0x%08X\n", reg32_read(0x021b4884));
	debug("MPDGHWST3 PHY1 (0x021b4888) = 0x%08X\n", reg32_read(0x021b4888));

	printf("errorcount: %d\n", errorcount);

	return 0;
}


/* init ddr3, do calibrations */
int mx6_ddr_init(ulong addr)
{
	unsigned int cfgval = 0;
	int errorcount = 0;


	puts("\nReprogramming DDR timings...\n" );

	cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET);
	debug("Original CTL: %08x\n", cfgval);

	cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDASP_OFFSET);
	debug("Original ASP: %08x\n", cfgval);

	cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDCFG0_OFFSET);
	debug("Original CFG0: %08x\n", cfgval);

	cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDCFG1_OFFSET);
	debug("Original CFG1: %08x\n", cfgval);

	cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDCFG2_OFFSET);
	debug("Original CFG2: %08x\n", cfgval);
	/*
	 * Write and read back some dummy data to demonstrate 
	 * that ddr3 is not broken
	 */
#ifdef CONFIG_MX6_DDRTUNE_WR_LEVEL
	puts("\nReference read/write test prior to tuning\n");
	do_tune_mww(addr);
	errorcount = do_tune_mrr(addr);
	if (errorcount)
		printf("%s: Initial testing shows %d errors\n", __func__, errorcount);

	/* do write (fly-by) calibration */
	puts("\nFly-by calibration\n");
	errorcount = do_tune_wcal();
	udelay(100000);
	/* let it settle in...seems it's necessary */
	if (errorcount != 0) {
		puts("Fly-by calibration seems to have failed\n");
	}
#endif
	/* Tune DQS delays. For some reason, has to be run twice. */
	puts("\nDQS delay calibration\n");
	//do_tune_delays(NULL, 0, 1, NULL);
	errorcount = do_tune_delays();
	if (errorcount != 0) {
		puts("DQS delay calibration has failed. Guessing values "
				"for delay cal based on rank...\n");
		reg32_write(MPDGCTRL0_PHY0, 0x456B057A);
		reg32_write(MPDGCTRL1_PHY0, 0x057F0607);
		reg32_write(MPDGCTRL0_PHY1, 0x46470645);
		reg32_write(MPDGCTRL1_PHY1, 0x0651061D);
		reg32_write(MPRDDLCTL_PHY0, 0x48444249);
		reg32_write(MPRDDLCTL_PHY1, 0x4D4D424E);
		reg32_write(MPWRDLCTL_PHY0, 0x322D4132);
		reg32_write(MPWRDLCTL_PHY1, 0x3D2E3F37);

	}
	/* 
	 * Confirm that the memory is working by read/write demo.
	 * Confirmation currently read out on terminal.
	 */
	puts("\nReference read/write test post-tuning\n");
	do_tune_mww(addr);
	errorcount = do_tune_mrr(addr);

	return errorcount;
}

void mx6_ddr_debug()
{
	u32 temp2;
	temp2 = reg32_read(MMDC_P0_BASE_ADDR + MPZQHWCTRL_OFFSET);
	printf("%s: MPZQHWCTRL = 0x%08x\n", __func__, temp2);

}
