/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <asm/imx-common/sata.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
//#include <asm/arch/mx6-ddr.h>

#include <usb.h>
#include <pwm.h>
#include <version.h>
#include <watchdog.h>
#include <video.h>

DECLARE_GLOBAL_DATA_PTR;

#include "../lm-common/lm_common_defs.h"
#include "sperre_pins.h"
#include "sperre_gpio.h"

#define ENET_PHY_RESET	ENET_CRS_DV

struct fsl_esdhc_cfg usdhc_cfg = USDHC4_BASE_ADDR;

#ifndef CONFIG_SPL_BUILD
static const char* hw_string[8] = {
	"REVA",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
};
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	if(cfg->esdhc_base == USDHC4_BASE_ADDR)
		ret = 1;

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;

		imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg);
		if (ret)
			return ret;

	return 0;
#else
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1		!!! using only SD4
	 * 0x2                  SD2		!!! using only SD4
	 * 0x3                  SD4
	 */

	imx_iomux_v3_setup_multiple_pads(
		usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
	usdhc_cfg.esdhc_base = USDHC4_BASE_ADDR;
	usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
	gd->arch.sdhc_clk = usdhc_cfg.sdhc_clk;


	return fsl_esdhc_initialize(bis, &usdhc_cfg);
#endif
}

/*
 *
 * TODO: Display to be setup here
 *
 */

#ifndef CONFIG_SPL_BUILD

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	/* TODO: Reset AR8033 PHY | Chip will be changed */
	gpio_direction_output(ENET_PHY_RESET , 0);

	udelay(500);
	gpio_set_value(ENET_PHY_RESET, 1);
}


int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();
	return cpu_eth_init(bis);
}
#endif
#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)



int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 0)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif

int board_early_init_f(void)
{
	/* TODO board version */

	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads, ARRAY_SIZE(usb_otg_pads));
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));
	imx_iomux_v3_setup_multiple_pads(other_pads, ARRAY_SIZE(other_pads));
	imx_iomux_v3_setup_multiple_pads(i2c_pads, ARRAY_SIZE(i2c_pads));

	gpio_direction_output(LCD_PPEN, 1);
	gpio_direction_output(BL_EN, 1);
	gpio_direction_output(GPIO_Debug_LED, 0);
	gpio_direction_output(RTC_IRQ, 0);
	gpio_direction_input(IO_INT);
	gpio_direction_output(MMC_RST, 0);
	gpio_direction_output(IO_RESET, 0);
	gpio_direction_output(TOUCH_RES, 0);
	gpio_direction_output(TOUCH_IRQ, 0);
	gpio_direction_output(PMIC_INT_B, 0);
	gpio_direction_output(RS485_PW, 0);
	gpio_direction_output(ADS1248_RESET, 0);
	gpio_direction_output(ADS1248_START, 0);
	gpio_direction_output(SPI_NOR_WP, 0);

	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(3, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	return 0;
}
