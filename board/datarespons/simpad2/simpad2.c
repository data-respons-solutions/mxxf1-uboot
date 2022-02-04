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
#include <errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <asm/mach-imx/sata.h>
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


#include <usb.h>
#include <pwm.h>
#include <version.h>
#include <watchdog.h>
#include <video.h>

DECLARE_GLOBAL_DATA_PTR;

#define FB_SYNC_DATA_INVERT	0x20000000
#define FB_SYNC_CLK_LAT_FALL	0x40000000
#define USE_PWM_FOR_BL
#define BL_PWM 0
#define VIBRA_PWM 1
#define CONFIG_PMIC_I2C_BUS 2

#include "../lm-common/lm_common_defs.h"
#include "simpad2_pins.h"
#include "simpad2_gpio.h"

static const char* hw_string[8] = {
	"REVB",
	"REVC",
	"REVD",
	"REVE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
};

static int get_version(void)
{
	return ((gpio_get_value(GPIO_HW_SETTING2) << 2) |
			(gpio_get_value(GPIO_HW_SETTING1) << 1) |
			gpio_get_value(GPIO_HW_SETTING0)) & 7;
}


int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}


#if defined(CONFIG_FSL_ESDHC)
int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};


int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {

	case USDHC3_BASE_ADDR:
	case USDHC4_BASE_ADDR:
		ret = 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;
	imx_iomux_v3_setup_multiple_pads(usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
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
		imx_iomux_v3_setup_multiple_pads(usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x3:
		imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

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

int overwrite_console(void)
{
	return 1;
}

#if defined(CONFIG_VIDEO_IPUV3)

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= NULL,
	.mode	= {
			.name           = "simpad2-VGA",
			.refresh        = 60,
			.xres           = 640,
			.yres           = 480,
			.pixclock       = 42105,
			.left_margin    = 16,
			.right_margin   = 80,
			.upper_margin   = 13,
			.lower_margin   = 33,
			.hsync_len      = 64,
			.vsync_len      = 4,
			.sync           = 0x40000000,
			.vmode          = FB_VMODE_NONINTERLACED
} },  };

size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	volatile struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	volatile struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));
	enable_ipu_clock();
	setbits_le32(&mxc_ccm->CCGR3, MXC_CCM_CCGR3_IPU1_IPU_MASK);
	//enable_vpll();
	clrbits_le32(&mxc_ccm->CCGR3, MXC_CCM_CCGR3_IPU1_IPU_DI0_MASK);
	clrsetbits_le32(&mxc_ccm->chsccdr, MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK,
			CHSCCDR_IPU_PRE_CLK_540M_PFD << MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);

	clrsetbits_le32(&mxc_ccm->chsccdr, MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK,
			CHSCCDR_PODF_DIVIDE_BY_3 << MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET);

	clrsetbits_le32(&mxc_ccm->chsccdr, MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK,
					0 << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);

	/* Turn on IPU DI0 clocks */
	setbits_le32(&mxc_ccm->CCGR3, MXC_CCM_CCGR3_IPU1_IPU_DI0_MASK);

	//setbits_le32(&mxc_ccm->CCGR3, 0x300);
	/* Set VSYNC Polarity */
	setbits_le32(&iomux->gpr[2], IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW);

}


/* Called after screen is blanked, so we can now turn on video */
static int show_splash(void *image_at)
{
	pwm_config(BL_PWM, 100, 20000);
	pwm_enable(BL_PWM);
	gpio_set_value(GPIO_LCD_EN, 0);
	video_display_bitmap((ulong)image_at, 0, 0);
	gpio_set_value(GPIO_BL_EN, 1);
	mdelay(200);
	pwm_config(BL_PWM, 10000, 20000);
	pwm_enable(BL_PWM);
	return 0;
}

#endif


static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);

	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
}


int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();
	return cpu_eth_init(bis);
}
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
	printf("%s: Port %d power %s\n", __func__, port, on ? "on" : "off");
	switch (port) {
	case 0:
		if (on) {
			gpio_set_value(GPIO_USB_PWR, 1);
			mdelay(600);	/* Allow time for USB stick to power */
		} else {
			gpio_set_value(GPIO_USB_PWR, 0);
		}
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
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads, ARRAY_SIZE(usb_otg_pads));
	imx_iomux_v3_setup_multiple_pads(revc_pads, ARRAY_SIZE(revc_pads));
	imx_iomux_v3_setup_multiple_pads(hw_settings_pads, ARRAY_SIZE(hw_settings_pads));
	imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);
	imx_iomux_v3_setup_multiple_pads(other_pads, ARRAY_SIZE(other_pads));

	gpio_direction_input(GPIO_HW_SETTING0);
	gpio_direction_input(GPIO_HW_SETTING1);
	gpio_direction_input(GPIO_HW_SETTING2);

	gpio_direction_output(GPIO_LED_R, 0);
	gpio_direction_output(GPIO_LED_G, 0);
	gpio_direction_output(GPIO_LED_B, 1);
	gpio_direction_output(GPIO_AUX_5V_EN, 1);

	gpio_direction_output(GPIO_WL_BAT_PWR_EN, 1);
	gpio_direction_output(GPIO_WL_VDDIO_EN, 1);
	gpio_direction_output(GPIO_USB_PWR, 0);
	return 0;
}

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

	printf("Board Init\n");
	gpio_direction_output(GPIO_TOUCH_IRQ, 1);
	gpio_direction_input(KEY_FUNCTION);
	gpio_direction_output(GPIO_LCD_EN, 1);

	gpio_direction_output(GPIO_CAP_TOUCH_RST, 1);
	gpio_direction_input(GPIO_ADAPTER_N);

	gpio_direction_input(GPIO_RECOVERY_SWITCH);
	gpio_direction_output(GPIO_SPI_NOR_WP, 1);

	gpio_direction_input(GPIO_PWR_BTN);
	gpio_direction_input(GPIO_DDR_SETTING);
	gpio_direction_output(GPIO_LCD_LR, 0);
	gpio_direction_output(GPIO_LCD_UD, 1);

	gpio_direction_output(GPIO_CHARGER_NCE, 0);
	gpio_direction_input(GPIO_PCU_START_ADAPTER);
	gpio_direction_input(GPIO_PCU_START_KEY);

	gpio_direction_output(GPIO_CHARGER_ISET, 1);
	gpio_direction_output(GPIO_VIBRA, 0);
	/* WIFI */
	imx_iomux_v3_setup_multiple_pads(wifi_pads, ARRAY_SIZE(wifi_pads));
	gpio_direction_output(GPIO_WL_REG_ON, 0);
	gpio_direction_output(GPIO_BT_REG_ON, 0);

	gpio_direction_output(GPIO_BL_EN, 0);
	gpio_set_value(GPIO_CAP_TOUCH_RST, 1);
	udelay(100);
	gpio_set_value(GPIO_TOUCH_IRQ, 0);
	mdelay(5);
	gpio_set_value(GPIO_TOUCH_IRQ, 1);
	gpio_direction_input(GPIO_TOUCH_IRQ);
#ifdef CONFIG_MXC_SPI
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
#endif
#ifdef CONFIG_VIDEO_IPUV3
	setup_display();
#endif
#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif
	return 0;
}

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

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

static char * const usbcmd[] = {"usb", "start"};
static char *const splash_load[] = { "ext4load", "mmc", "0:1", "0x19000000", "/boot/Logo.bmp" };

int board_late_init(void)
{
	int rep;
	ulong ticks;
	enum command_ret_t ret;
	int version = get_version();
	switch (version)
	{
	case 1:
	case 2:
	case 3:
		env_set("fdt_file", "/boot/simpad2-revC.dtb");
		break;

	default:
		env_set("fdt_file", "/boot/simpad2-revE.dtb");
		break;
	}

	printf("SIMPAD2 HW version: %s\n", hw_string[version]);

	ret = cmd_process(0, 5, splash_load, &rep, &ticks);
#ifdef CONFIG_VIDEO
	if (ret == CMD_RET_SUCCESS)
		show_splash((void*)0x19000000);
	else
		printf("Unable to load logo BMP file\n");
#endif
	cmd_process(0, 2, usbcmd, &rep, &ticks);
	mdelay(500);

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: Simpad2\n");
	return 0;
}

int lm_ram64(void)
{
	return 1;
}

#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-ddr.h>

const struct mx6sdl_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_sdclk_0 =  0x00020030,
	.dram_sdclk_1 =  0x00020030,
	.dram_cas =  0x00020030,
	.dram_ras =  0x00020030,
	.dram_reset =  0x00020030,
	.dram_sdcke0 =  0x00003000,
	.dram_sdcke1 =  0x00003000,
	.dram_sdba2 =  0x00000000,
	.dram_sdodt0 =  0x00003030,
	.dram_sdodt1 =  0x00003030,
	.dram_sdqs0 =  0x00000030,
	.dram_sdqs1 =  0x00000030,
	.dram_sdqs2 =  0x00000030,
	.dram_sdqs3 =  0x00000030,
	.dram_sdqs4 =  0x00000030,
	.dram_sdqs5 =  0x00000030,
	.dram_sdqs6 =  0x00000030,
	.dram_sdqs7 =  0x00000030,
	.dram_dqm0 =  0x00020030,
	.dram_dqm1 =  0x00020030,
	.dram_dqm2 =  0x00020030,
	.dram_dqm3 =  0x00020030,
	.dram_dqm4 =  0x00020030,
	.dram_dqm5 =  0x00020030,
	.dram_dqm6 =  0x00020030,
	.dram_dqm7 =  0x00020030,
};

const struct mx6sdl_iomux_grp_regs mx6_grp_ioregs = {
	.grp_ddr_type =  0x000C0000,
	.grp_ddrmode_ctl =  0x00020000,
	.grp_ddrpke =  0x00000000,
	.grp_addds =  0x00000030,
	.grp_ctlds =  0x00000030,
	.grp_ddrmode =  0x00020000,
	.grp_b0ds =  0x00000030,
	.grp_b1ds =  0x00000030,
	.grp_b2ds =  0x00000030,
	.grp_b3ds =  0x00000030,
	.grp_b4ds =  0x00000030,
	.grp_b5ds =  0x00000030,
	.grp_b6ds =  0x00000030,
	.grp_b7ds =  0x00000030,
};

const struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 =  0x0046004E,
	.p0_mpwldectrl1 =  0x003D0048,
	.p1_mpwldectrl0 =  0x002B0035,
	.p1_mpwldectrl1 =  0x002F0049,
	.p0_mpdgctrl0 =  0x42400240,
	.p0_mpdgctrl1 =  0x022C022C,
	.p1_mpdgctrl0 =  0x42200228,
	.p1_mpdgctrl1 =  0x021C0220,
	.p0_mprddlctl =  0x44464848,
	.p1_mprddlctl =  0x44464642,
	.p0_mpwrdlctl =  0x342E282C,
	.p1_mpwrdlctl =  0x362E342A,
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 1600,
	.density = 4,
	.width = 64,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FF03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

int pmic_setup(void)
{
	int ret;
	i2c_set_bus_num(CONFIG_PMIC_I2C_BUS);
	ret = i2c_probe(CONFIG_POWER_PFUZE100_I2C_ADDR);
	if (ret)
	{
		printf("%s: no pmic\n", __func__);
		return -ENODEV;
	}
	return 0;
}


static int pmic_set(pf100_regs reg, int mV)
{
	u8 values[2];
	i2c_set_bus_num(CONFIG_PMIC_I2C_BUS);
	switch (reg) {

	case SW1AB:
		if (mV > 1425) {
			printf("%s: SW1AB max is 1425 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		printf("Setting PMIC register SW1AB to %d mV\n", mV);
		values[0] = (mV - 300) / 25;
		i2c_write(0x08, PFUZE100_SW1ABVOL, 1, values, 1);
		break;

	case SW1C:
		if (mV > 1425) {
			printf("%s: SW1AB max is 1425 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		printf("Setting PMIC register SW1C to %d mV\n", mV);
		values[0] = (mV - 300) / 25;
		i2c_write(0x08, PFUZE100_SW1CVOL, 1, values, 1);
		break;

	case SW3AB:
		if (mV > 1500) {
			printf("%s: SW3AB max is 1500 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		printf("Setting PMIC register SW3AB to %d mV\n", mV);
		values[0] = (mV - 400) / 25;
		i2c_write(0x08, PFUZE100_SW3AVOL, 1, values, 1);
		i2c_write(0x08, PFUZE100_SW3BVOL, 1, values, 1);
		break;

	case VGEN4:
		printf("Setting PMIC register VGEN4 to %d mV\n", mV);
		values[0] = ((mV - 1800) * 15) / 1500;
		i2c_write(0x08, PFUZE100_VGEN4VOL, 1, values, 1);
		break;
	}
	return 0;
}

/*
 * This section requires the differentiation between iMX6 Sabre boards, but
 * for now, it will configure only for the mx6q variant.
 */
static void spl_dram_init(void)
{
#ifdef CONFIG_MX6_DDRCAL
	int err;
#endif
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus:0=16,1=32,2=64 */
		.dsize = mem_ddr.width/32,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32, /* 32Gb per CS */
		/* single chip select */
		.ncs = 1,
		.cs1_mirror = 0,
		.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
#ifdef RTT_NOM_120OHM
		.rtt_nom = 2 /*DDR3_RTT_120_OHM*/,	/* RTT_Nom = RZQ/2 */
#else
		.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
#endif
		.walat = 1,	/* Write additional latency */
		.ralat = 5,	/* Read additional latency */
		.mif3_mode = 3,	/* Command prediction working mode */
		.bi_on = 1,	/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
		.ddr_type = DDR_TYPE_DDR3,
		.refsel = 1,	/* Refresh cycles at 32KHz */
		.refr = 3,	/* 4 refresh commands per refresh cycle */
	};

	mx6sdl_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
#ifdef CONFIG_MX6_DDRCAL
	err = mmdc_do_write_level_calibration(&sysinfo);
	if (err) {
		printf("error %d from write level calibration\n", err);
	} else {
		err = mmdc_do_dqs_calibration(&sysinfo);
		if (err) {
			printf("error %d from dqs calibration\n", err);
		} else {
			printf("completed successfully\n");
			mmdc_read_calibration(&sysinfo, &mx6_mmcd_calib);
			display_calibration(&sysinfo, &mx6_mmcd_calib);
		}
	}
#endif
}

void board_init_f(ulong dummy)
{
	int err;
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

#ifdef CONFIG_SPL_WATCHDOG_SUPPORT
	hw_watchdog_init();
#endif
	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();
	spl_dram_init();
	printf("SPL started\n");

	err = pmic_setup();
	if (err == 0) {
		pmic_set(SW1AB, 1425);
		pmic_set(SW1C, 1425);
		pmic_set(SW3AB, 1350);
		udelay(10000);
	}
	/* DDR initialization */
	printf("Memory width %d\n", mem_ddr.width);
}

#ifndef CONFIG_SPL_WATCHDOG_SUPPORT
void reset_cpu(ulong addr)
{
}
#endif
#endif /* SPL */
