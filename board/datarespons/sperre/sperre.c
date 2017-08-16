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

#define BL_PWM 0

#include "../lm-common/lm_common_defs.h"
#include "sperre_pins.h"
#include "sperre_gpio.h"

#define GPIO_ENET_PHY_RESET	GPIO_ENET_RST

struct fsl_esdhc_cfg usdhc_cfg[1] = {
		{USDHC4_BASE_ADDR},
	};

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
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
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
	usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
	gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;


	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_VIDEO_IPUV3)

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

/* Enable LVDS1 for LCD, 3 Differential lines - 18BIT */
static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,	// has 3 differential lines, alternatively IPU_PIX_FMT_LVDS666
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Sperre-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 15380,	// Period = 15.38ns, Freq = 65.019 Mhz | 10^12/65019505.851756 Hz = 15380
		.left_margin    = 80,
		.right_margin   = 40,
		.upper_margin   = 10,
		.lower_margin   = 10,
		.hsync_len      = 20,
		.vsync_len      = 10,
		.sync           = 0,	// alternatively FB_SYNC_EXT
		.vmode          = FB_VMODE_NONINTERLACED,
		.flag			= FB_MODE_IS_DETAILED
} },  };
size_t display_count = ARRAY_SIZE(displays);

/* Screen is located on LVDS1 */
static void setup_display(void)
{
	volatile struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	volatile struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	//int reg;

	enable_ipu_clock();

	/* Turn on LDB0, LDB1, IPU, IPU DI0 clocks */
	setbits_le32(&mxc_ccm->CCGR3,
			MXC_CCM_CCGR3_LDB_DI0_MASK |
			MXC_CCM_CCGR3_LDB_DI1_MASK);

	/* set LDB0, LDB1 clk select to 011/011 */
	clrsetbits_le32(&mxc_ccm->cs2cdr,
			MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK |
			MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK,
			(3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET) |
			(3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET));

	setbits_le32(&mxc_ccm->cscmr2,
			MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV |
			MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV);

	setbits_le32(&mxc_ccm->chsccdr,
			(CHSCCDR_CLK_SEL_LDB_DI0 << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET) |
			(CHSCCDR_CLK_SEL_LDB_DI0 << MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET));

	setbits_le32(&iomux->gpr[2],
			IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES |
			IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW |
			IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW |
			IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG |
			IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT |
			IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG |
		 	IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
		 	IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED |
		 	IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0);

	clrsetbits_le32(&iomux->gpr[3],
			IOMUXC_GPR3_LVDS1_MUX_CTL_MASK,	// | IOMUXC_GPR3_HDMI_MUX_CTL_MASK,
			IOMUXC_GPR3_MUX_SRC_IPU1_DI0 << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
}
#ifdef CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
int overwrite_console(void)
{
	return 1;
}
#endif

/* Called after screen is blanked, so we can now turn on video */
static int show_splash(void *image_at)
{
	pwm_config(BL_PWM, 100, 20000);
	pwm_enable(BL_PWM);
	gpio_set_value(GPIO_LCD_PPEN, 1);
	video_display_bitmap((ulong)image_at, 0, 0);
	gpio_set_value(GPIO_BL_EN, 1);
	mdelay(200);
	pwm_config(BL_PWM, 10000, 20000);
	pwm_enable(BL_PWM);
	return 0;
}

#endif

#ifndef CONFIG_SPL_BUILD

static void setup_iomux_enet(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* Reconfigure enet muxing while LAN8710 PHY is in reset */
	gpio_direction_output(GPIO_ENET_PHY_RESET, 0);
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	mdelay(10);
	gpio_set_value(GPIO_ENET_PHY_RESET, 1);
	udelay(100);

	/* set GPIO_16 as ENET_REF_CLK_OUT */
	setbits_le32(&iomux->gpr[1], IOMUXC_GPR1_ENET_CLK_SEL_MASK);

	enable_fec_anatop_clock(0, ENET_50MHZ);
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

	if (port > 1)
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
		if (on) {
			gpio_direction_output(GPIO_USB_OTG_PWR_EN, 1);
			mdelay(10);
		}
		else
			gpio_direction_output(GPIO_USB_OTG_PWR_EN, 0);
	case 1:
		if (on) {
			gpio_direction_output(GPIO_USB_H1_PWR_EN, 1);
			mdelay(10);
		}
		else
			gpio_direction_output(GPIO_USB_H1_PWR_EN, 0);
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
	imx_iomux_v3_setup_multiple_pads(usb_h1_pads, ARRAY_SIZE(usb_h1_pads));
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));
	imx_iomux_v3_setup_multiple_pads(other_pads, ARRAY_SIZE(other_pads));
	imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
	imx_iomux_v3_setup_multiple_pads(i2c_pads, ARRAY_SIZE(i2c_pads));

	gpio_direction_output(GPIO_LCD_PPEN, 1);
	gpio_direction_output(GPIO_BL_EN, 0);
	gpio_direction_output(GPIO_BL_PWM, 0);
	gpio_set_value(GPIO_BL_PWM, 1);		// setting it on

	gpio_direction_output(GPIO_Debug_LED, 0);
	gpio_direction_output(GPIO_RTC_IRQ, 1);
	gpio_direction_input(GPIO_IO_INT);
	gpio_direction_output(GPIO_MMC_RST, 1);
	gpio_direction_output(GPIO_IO_RESET, 0);
	gpio_direction_output(GPIO_TOUCH_RES, 1);
	gpio_direction_output(GPIO_TOUCH_IRQ, 1);
	gpio_direction_output(GPIO_PMIC_INT_B, 0);
	gpio_direction_output(GPIO_RS485_PW, 0);
	//gpio_direction_output(GPIO_ADS1248_RESET, 0);
	gpio_direction_output(GPIO_ADS1248_START, 0);
	gpio_direction_output(GPIO_SPI_NOR_WP, 0);

	/* SPI_NOR_CS GPIO */
	gpio_direction_output(SPI_CS_GPIO, 1);	// leave high so it is not selected

	/* USB */
	gpio_direction_output(GPIO_USB_OTG_PWR_EN, 0);
	gpio_direction_output(GPIO_USB_H1_PWR_EN, 0);

	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(3, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	return 0;
}

#ifndef CONFIG_SPL_BUILD
static void setup_usb(void)
{
	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
#ifdef CONFIG_MX6Q
	imx_iomux_set_gpr_register(1, 13, 1, 0);
#endif
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	gpio_set_value(GPIO_TOUCH_RES, 1);
	udelay(100);
	gpio_set_value(GPIO_TOUCH_IRQ, 1);
	mdelay(5);
	gpio_set_value(GPIO_TOUCH_IRQ, 0);
	gpio_direction_input(GPIO_TOUCH_IRQ);

#ifdef CONFIG_VIDEO_IPUV3
	setup_display();
#endif
	setup_usb();
	return 0;
}
#endif	/* CONFIG_SPL_BUILD */

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	switch (bus)
	{
	case 0:
		return cs == 1 ? SPI_CS_GPIO : -1;
		break;
	default:
		return -1;
	}
}

#ifndef CONFIG_SPL_BUILD
static char * const usbcmd[] = {"usb", "start"};
static char * const reset_env_cmd[] = {"env", "default", "-a"};
static char * const save_env_cmd[] = {"env", "save"};
static char * const splash_load[] = { "load", "mmc", "0:1", "0x19000000", "/boot/Sperre_800x480.bmp" };

int board_late_init(void)
{
	int rep;
	ulong ticks;
	enum command_ret_t ret;

	printf("Sperre version 1.0 %s\n", hw_string[0]);

	ret = cmd_process(0, 5, splash_load, &rep, &ticks);
	if (ret == CMD_RET_SUCCESS)
		show_splash((void*)0x19000000);
	else
		printf("Unable to load logo BMP file\n");


	cmd_process(0, 2, usbcmd, &rep, &ticks);

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}
#endif

int checkboard(void)
{
	puts("Board: Sperre\n");
	return 0;
}

