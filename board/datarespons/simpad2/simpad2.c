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
//#include "../common/pfuze.h"

#include <usb.h>
#include <pwm.h>
#include <version.h>
#include <watchdog.h>

DECLARE_GLOBAL_DATA_PTR;

#define USE_PWM_FOR_BL
#define BL_PWM 1

#include "../lm-common/lm_common_defs.h"
#ifdef CONFIG_EMU_SABRESD
#include "sabresd_pins.h"
#include "sabresd_gpio.h"

#else
#include "simpad2_pins.h"
#include "simpad2_gpio.h"
#endif

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};
static int enable_display=0;

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}


int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
#ifdef CONFIG_EMU_SABRESD
	case USDHC3_BASE_ADDR:

		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
#else
	case USDHC3_BASE_ADDR:
	case USDHC4_BASE_ADDR:
		ret = 1;
		break;
#endif
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;

		imx_iomux_v3_setup_multiple_pads(usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
#ifdef CONFIG_EMU_SABRESD
		gpio_direction_input(USDHC3_CD_GPIO);
#endif
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
		if (ret)
			return ret;
		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[1]);
		if (ret)
			return ret;

	return 0;
#else
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x2                  SD2
	 * 0x3                  SD4
	 */

	switch (reg & 0x3) {
	case 0x2:
		imx_iomux_v3_setup_multiple_pads(
			usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x3:
		imx_iomux_v3_setup_multiple_pads(
			usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}


#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_VIDEO_IPUV3) && defined(CONFIG_EMU_SABRESD)
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT;
	writel(reg, &iomux->gpr[2]);
}


struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
			.name           = "simpad2-XGA",
			.refresh        = 60,
			.xres           = 1024,
			.yres           = 768,
			.pixclock       = 15385,
			.left_margin    = 220,
			.right_margin   = 40,
			.upper_margin   = 21,
			.lower_margin   = 7,
			.hsync_len      = 60,
			.vsync_len      = 10,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
			.name           = "HDMI",
			.refresh        = 60,
			.xres           = 1280,
			.yres           = 720,
			.pixclock       = 15686,
			.left_margin    = 48,
			.right_margin   = 32,
			.upper_margin   = 3,
			.lower_margin   = 5,
			.hsync_len      = 80,
			.vsync_len      = 13,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
} },  };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	imx_iomux_v3_setup_multiple_pads(di0_pads, ARRAY_SIZE(di0_pads));

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_VIDEO_IPUV3) && !defined(CONFIG_EMU_SABRESD)

static void enable_rgb(struct display_info_t const *dev)
{
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
			.name           = "simpad2-XGA",
			.refresh        = 60,
			.xres           = 640,
			.yres           = 480,
			.pixclock       = 41500,
			.left_margin    = 220,
			.right_margin   = 40,
			.upper_margin   = 21,
			.lower_margin   = 7,
			.hsync_len      = 60,
			.vsync_len      = 10,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
} },  };
size_t display_count = ARRAY_SIZE(displays);
static void setup_display(void)
{

}
#endif

#ifdef CONFIG_VIDEO
int overwrite_console(void)
{
	return 0;
}
#endif


#ifndef CONFIG_SPL_BUILD
#define MAX_I2C_DATA_LEN 10
static char egalax_fw[6];
static int egalax_firmware_version(void)
{
	int to=100000;
	static const uint8_t cmd[MAX_I2C_DATA_LEN] = { 0x03, 0x03, 0xa, 0x01, 'D', 0, 0, 0, 0, 0 };
	uint8_t rcv_buf[MAX_I2C_DATA_LEN+1];

	int ret;
	i2c_set_bus_num(2);
	i2c_set_bus_speed(CONFIG_SYS_I2C_SPEED);

	ret = i2c_probe(0x04);
	if (ret)
	{
		printf("%s: NO touch controller\n", __func__);
		return -ENODEV;
	}
	if (i2c_write(0x04, 0, 0, (uint8_t*)cmd, MAX_I2C_DATA_LEN))
	{
		printf("%s: Error requesting FW version\n", __func__);
		return -EIO;
	}
	else
	{
		while (gpio_get_value(GPIO_TOUCH_IRQ) && to > 0)
		{
			udelay(1000);
			to--;
		}
		if (to <= 0)
		{
			printf("%s: timeout waiting for touch irq\n", __func__);
			return -EIO;
		}
		ret = i2c_read(0x04, 0, 0, rcv_buf, MAX_I2C_DATA_LEN);
		if (ret)
		{
			printf("%s: Error receiving FW version\n", __func__);
			return -EIO;
		}
		if (rcv_buf[0] == 0x03 && rcv_buf[4] == 'D')
		{
			rcv_buf[MAX_I2C_DATA_LEN] = '\0';
			strcpy(egalax_fw, (char *)&rcv_buf[5]);
			printf("Touch FW version [%s]\n", egalax_fw);
			i2c_read(0x04, 0, 0, rcv_buf, MAX_I2C_DATA_LEN);	/* Drain the rest */
		}
		else
		{
			printf("%s: Got touch message %x.%x.%x\n", __func__, rcv_buf[0], rcv_buf[1], rcv_buf[2]);
		}
	}

#ifndef CONFIG_EMU_SABRESD
	gpio_set_value(GPIO_CAP_TOUCH_RST, 0);
	udelay(500);
	gpio_set_value(GPIO_CAP_TOUCH_RST, 1);
#endif
	return 0;
}

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);

	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
}


int simpad2_eeprom_init (unsigned dev_addr);
int eeprom_get_mac_addr(void);
int eeprom_addr;

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
		break;
#ifdef CONFIG_EMU_SABRESD
	case 1:
		if (on) {
			gpio_direction_output(GPIO_USB_H1_EN, 1);
			mdelay(10);
		}
		else {
			gpio_direction_output(GPIO_USB_H1_EN, 0);
		}
		break;
#endif
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif

int board_early_init_f(void)
{

	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads, ARRAY_SIZE(usb_otg_pads));
	imx_iomux_v3_setup_multiple_pads(other_pads, ARRAY_SIZE(other_pads));
	imx_iomux_v3_setup_multiple_pads(i2c_pads, ARRAY_SIZE(i2c_pads));

	gpio_direction_input(GPIO_TOUCH_IRQ);
	gpio_direction_input(KEY_FUNCTION);
	gpio_direction_output(GPIO_LCD_EN, 0);
	gpio_direction_output(GPIO_BL_EN, 0);

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#ifndef CONFIG_EMU_SABRESD
	setup_i2c(3, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);
	gpio_direction_output(GPIO_CAP_TOUCH_RST, 0);
	gpio_direction_output(GPIO_AUX_5V_EN, 1);	/* Turn on power */
	gpio_direction_output(GPIO_CHARGER_NCE, 0);	/* Turn on charger */
	gpio_direction_input(GPIO_ADAPTER_N);

	gpio_direction_output(GPIO_WL_REG_ON, 0);
	gpio_direction_output(GPIO_BT_REG_ON, 0);
	gpio_direction_input(GPIO_RECOVERY_SWITCH);
	gpio_direction_output(GPIO_SPI_NOR_WP, 1);
	gpio_direction_output(GPIO_CHARGER_ISET, 0);
	/* Turn on main power and IO for WiFI module - no action in kernel */
	gpio_direction_output(GPIO_WL_BAT_PWR_EN, 1);
	gpio_direction_output(GPIO_WL_VDDIO_EN, 1);

	gpio_direction_input(GPIO_HW_SETTING0);
	gpio_direction_input(GPIO_HW_SETTING1);
	gpio_direction_input(GPIO_PWR_BTN);
	gpio_direction_input(GPIO_DDR_SETTING);
	gpio_direction_output(GPIO_LED_R, 0);
	gpio_direction_output(GPIO_LED_G, 1);
	gpio_direction_output(GPIO_LED_B, 1);
	gpio_direction_output(GPIO_LCD_LR, 0);
	gpio_direction_output(GPIO_LCD_UD, 0);
	gpio_direction_output(GPIO_PMU_RST_N, 1);

#endif

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_VIDEO)
	setup_display();
#endif
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

#ifndef CONFIG_EMU_SABRESD
	gpio_set_value(GPIO_CAP_TOUCH_RST, 1);
	udelay(100);
#endif
	/* Check if the display should present u-boot info */
	enable_display = gpio_get_value(KEY_FUNCTION);	/* TODO: invert */

	if (enable_display)
	{
		if (pwm_config(BL_PWM, 4000, 5000))
			printf("%s: pwm_config for backlight ERROR\n", __func__);

		pwm_enable(BL_PWM);
		gpio_set_value(GPIO_LCD_EN, 1);
		gpio_set_value(GPIO_BL_EN, 1);
	}

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
#ifndef CONFIG_EMU_SABRESD
	case 1:
		return cs == 1 ? GPIO_PMU_SPI_CS : -1;
		break;
#endif
	default:
		return -1;
	}
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

#ifndef CONFIG_SPL_BUILD
static char * const usbcmd[] = {"usb", "start"};
static char * const reset_env_cmd[] = {"env", "default", "-a"};
static char * const save_env_cmd[] = {"env", "save"};

int board_late_init(void)
{
	int rep;
	ulong ticks;

#ifndef CONFIG_EMU_SABRESD
	printf("SIMPAD2 HW version: %d\n",
			(gpio_get_value(GPIO_HW_SETTING1) << 1) | gpio_get_value(GPIO_HW_SETTING0)
			);
#endif
	//egalax_firmware_version();
#if 0
	printf("Simpad2 U-BOOT version [%s]\n", U_BOOT_VERSION);
	if (egalax_firmware_version() == 0)
		vpd_update(egalax_fw, GPIO_SPI_NOR_WP);
	else
		vpd_update(0, GPIO_SPI_NOR_WP);
#endif
	cmd_process(0, 2, usbcmd, &rep, &ticks);
	//eeprom_get_mac_addr();

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}
#endif

int checkboard(void)
{
	puts("Board: Simpad2\n");
	return 0;
}

int lm_ram64(void)
{
#ifndef CONFIG_EMU_SABRESD
	return gpio_get_value(GPIO_DDR_SETTING);
#else
	return 1;
#endif
}


