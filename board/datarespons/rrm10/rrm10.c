/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
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
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/imx-common/mxc_i2c.h>
#include <i2c.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <watchdog.h>
#include <usb.h>
#include <asm/imx_pwm.h>

#define ANATOP_MISC1_CLK1_OBEN (1 << 10)
#define ANATOP_MISC1_CLK1_IBEN (1 << 12)

void hw_watchdog_init(void);

DECLARE_GLOBAL_DATA_PTR;

static int panel_version=0;
static int eeprom_addr;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |               \
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL_UP  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define ENET_PAD_CTRL_DN  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define SLOWOUT_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define OUT_LOW_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define REGINP_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_DISABLE  | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define SPI_IN_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_HIGH |               \
	PAD_CTL_DSE_DISABLE   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)


#define GPIO_AUX_5V IMX_GPIO_NR(6, 10)
#define GPIO_PCIE_RST_N IMX_GPIO_NR(7, 12)
#define GPIO_EEPROM_WP	IMX_GPIO_NR(1, 5)
#define GPIO_SPI_NOR_WP	IMX_GPIO_NR(6, 11)
#define GPIO_LAN2_EE_WP	IMX_GPIO_NR(6, 14)
#define GPIO_CAP_TOUCH_RST	IMX_GPIO_NR(6, 16)
#define GPIO_CAP_TOUCH_PWR	IMX_GPIO_NR(1, 4)
#define GPIO_LCD_EN	IMX_GPIO_NR(6, 15)
#define GPIO_BL_EN	IMX_GPIO_NR(1, 2)
/* #define GPIO_BL_PWM	IMX_GPIO_NR(1, 9) */
#define GPIO_FAN_EN IMX_GPIO_NR(1, 18)
#define GPIO_BUZ_INT_EN IMX_GPIO_NR(1, 17)
#define GPIO_BUZ_EXT_EN IMX_GPIO_NR(1, 19)

#define GPIO_USB_H1_EN	IMX_GPIO_NR(1, 29)
#define GPIO_USBP0_EN	IMX_GPIO_NR(3, 28)
#define GPIO_USBP1_EN	IMX_GPIO_NR(2, 3)
#define GPIO_USBP2_EN	IMX_GPIO_NR(2, 7)
#define GPIO_USBP3_EN	IMX_GPIO_NR(6, 9)
#define GPIO_LVDS_ROTATE IMX_GPIO_NR(7, 11)

#define GPIO_DIMM_DN IMX_GPIO_NR(4, 6)
#define GPIO_DIMM_UP IMX_GPIO_NR(4, 7)


int rrm10_eeprom_read (unsigned dev_addr, unsigned offset, uchar *buffer, unsigned cnt);
static void setup_display(void);


static unsigned char eeprom_content[2048];

int dram_init(void)
{

	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);
	return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const extra_nandf_pads[] = {
	MX6_PAD_NANDF_CS0__GPIO_6_11		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* SPI_NOR_WP	*/
	MX6_PAD_NANDF_CS1__GPIO_6_14		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* LAN2_EEWP	*/
	MX6_PAD_NANDF_CS2__GPIO_6_15		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* LCD_EN		*/
	MX6_PAD_NANDF_CS3__GPIO_6_16		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* CAP_TCH_RST	*/

	MX6_PAD_NANDF_ALE__GPIO_6_8			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* CAP_TCH_INT	*/
	MX6_PAD_NANDF_CLE__GPIO_6_7			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USBP3_OC		*/
	MX6_PAD_NANDF_WP_B__GPIO_6_9		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* USBP3_EN		*/
	MX6_PAD_NANDF_RB0__GPIO_6_10		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* AUX_5V_EN	*/

	MX6_PAD_NANDF_D2__GPIO_2_2			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USBP1_OC		*/
	MX6_PAD_NANDF_D3__GPIO_2_3			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* USBP1_EN		*/

	MX6_PAD_NANDF_D4__GPIO_2_4			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* WDOG_PWR_EN	*/
	MX6_PAD_NANDF_D5__GPIO_2_5			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* EMMC_RST#	*/
	MX6_PAD_NANDF_D6__GPIO_2_6			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USBP2_OC		*/
	MX6_PAD_NANDF_D7__GPIO_2_7			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* USBP2_EN		*/
};

iomux_v3_cfg_t const extra_nvcc_gpio_pads[] = {
	MX6_PAD_KEY_COL0__GPIO_4_6			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* DIMM-	*/
	MX6_PAD_KEY_ROW0__GPIO_4_7			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* DIMM+	*/
	MX6_PAD_KEY_COL1__GPIO_4_8			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* TEMP_OS	*/

	MX6_PAD_KEY_COL2__GPIO_4_10			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* CODEC_PWR_EN	*/
	MX6_PAD_KEY_ROW2__GPIO_4_11			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* HDMI_CEC_IN	*/

	MX6_PAD_KEY_COL4__CAN2_TXCAN		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* CAN2_TX	*/
	MX6_PAD_KEY_ROW4__CAN2_RXCAN		| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* CAN2_RX	*/

	MX6_PAD_GPIO_0__CCM_CLKO			| MUX_PAD_CTRL(PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_HIGH),	/* AUD_MCLK	*/
	MX6_PAD_GPIO_1__WDOG2_WDOG_B		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* WDOG_B		*/
	MX6_PAD_GPIO_2__GPIO_1_2			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* BLON_3V_H	*/

	MX6_PAD_GPIO_4__GPIO_1_4			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* TCH_PWRON	*/
	MX6_PAD_GPIO_5__GPIO_1_5			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* EEPROM_WP	*/
	MX6_PAD_GPIO_7__CAN1_TXCAN			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* CAN1_TX	*/

	MX6_PAD_GPIO_8__CAN1_RXCAN			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* CAN1_RX		*/
	MX6_PAD_GPIO_9__PWM1_PWMO			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* BL_PWM		*/
	MX6_PAD_GPIO_17__GPIO_7_12			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* PCIE_RST	*/
	MX6_PAD_GPIO_18__GPIO_7_13			| MUX_PAD_CTRL(PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_100K_DOWN),	/* PMIC_INT_B	*/

};

iomux_v3_cfg_t const extra_pads[] = {
	MX6_PAD_ENET_RX_ER__GPIO_1_24		| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USB_OTG_ID	*/
	MX6_PAD_ENET_RXD0__GPIO_1_27		| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* UOK_B		*/
	MX6_PAD_ENET_TXD1__GPIO_1_29		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* USB_H1_PWR_EN	*/
	MX6_PAD_SD1_CMD__GPIO_1_18			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* FAN_EN		*/
	MX6_PAD_SD1_DAT1__PWM3_PWMO			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* BUZ_INT_EN	*/
	MX6_PAD_SD1_DAT2__GPIO_1_19			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* BUZ_EXT_EN	*/

	MX6_PAD_CSI0_DATA_EN__GPIO_5_20		| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* PCIE_WAKE_B	*/

	MX6_PAD_EIM_D21__GPIO_3_21			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USB_OTG_OC		*/
	MX6_PAD_EIM_D22__GPIO_3_22			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* USB_ORG_PWR_EN	*/
	MX6_PAD_EIM_D29__GPIO_3_29			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* PWR_BTN_SNS		*/
	MX6_PAD_EIM_D30__GPIO_3_30			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USB_H1_OC		*/
	MX6_PAD_EIM_D31__GPIO_3_31			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USB_P0_OC		*/
	MX6_PAD_EIM_D28__GPIO_3_28			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* USBP0_EN			*/

	MX6_PAD_EIM_D23__GPIO_3_23			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* LAN2_DEV_OFF#	*/
	MX6_PAD_EIM_D24__GPIO_3_24			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* LAN2_SMB_ALRT#	*/
	MX6_PAD_GPIO_16__GPIO_7_11			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL),	/* LVDS_ROTATE		*/

};

iomux_v3_cfg_t const audio_pads[] = {
	MX6_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC	| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* AUD3_TXC		*/
	MX6_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD	| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* AUD3_TXD		*/
	MX6_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS	| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* AUD3_TXFS	*/
	MX6_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD	| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* AUD3_RXD		*/
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL_UP),
	MX6_PAD_ENET_MDC__ENET_MDC			| MUX_PAD_CTRL(ENET_PAD_CTRL_UP),
	MX6_PAD_RGMII_TXC__ENET_RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_TD0__ENET_RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_TD1__ENET_RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_TD2__ENET_RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_TD3__ENET_RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_RXC__ENET_RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_RD0__ENET_RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_RD1__ENET_RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_RD2__ENET_RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_RD3__ENET_RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	/* AR8031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO_1_25		| MUX_PAD_CTRL(ENET_PAD_CTRL_UP),
	MX6_PAD_ENET_TX_EN__GPIO_1_28		| MUX_PAD_CTRL(ENET_PAD_CTRL_UP), 	/* ENET_WOL_INT (low)	*/
	MX6_PAD_ENET_RXD1__GPIO_1_26		| MUX_PAD_CTRL(ENET_PAD_CTRL_UP), 	/* RGMII_INT (low)	*/
};

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO_5_27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO_5_26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 26)
	}
};

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO_4_12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO_4_13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO_3__GPIO_1_3 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO_6__GPIO_1_6 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 6)
	}
};

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS1 */
	MX6_PAD_EIM_D19__GPIO_3_19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

void setup_spi(void)
{
	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);

}
#endif

static void setup_iomux_enet(void)
{


	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	gpio_direction_output(GPIO_CAP_TOUCH_RST, 0);

	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
	gpio_set_value(GPIO_CAP_TOUCH_RST, 1);
}


iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__USDHC3_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__USDHC3_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__USDHC3_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__USDHC3_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D0__GPIO_2_0    | MUX_PAD_CTRL(REGINP_PAD_CTRL), 		/* CD 				*/
	MX6_PAD_NANDF_D1__GPIO_2_1    | MUX_PAD_CTRL(REGINP_PAD_CTRL),		/* WP 				*/
	MX6_PAD_SD3_RST__GPIO_7_8	  | MUX_PAD_CTRL(REGINP_PAD_CTRL),		/* HEADPHONE_DET	*/
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__USDHC4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__USDHC4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__USDHC4_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__USDHC4_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__USDHC4_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__USDHC4_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__USDHC4_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__USDHC4_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__USDHC4_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__USDHC4_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D5__GPIO_2_5    | MUX_PAD_CTRL(USDHC_PAD_CTRL),	/* eMMC reset */
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}


int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) then supported by the board (%d)\n",
				index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}

	return status;
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
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


static int get_mac_addr(void)
{
	int ret=0;
	char *base = (char*)&eeprom_content[4];
	char *at = base;
	char *at_end = base + sizeof(eeprom_content);
	int len;

	ret = rrm10_eeprom_read(eeprom_addr, 0, eeprom_content, sizeof(eeprom_content) );
	if (ret < 0)
	{
		printf("%s: eeprom read failure\n", __func__);
		return -ENODEV;
	}
	while ( at < at_end)
	{
		char *p = strstr(at, "FEC_MAC_ADDR=");
		if (p)
		{
			printf("%s: found MAC info %s\n", __func__, p);
			p += 13;
			if ((len = strnlen(p, 20)) == 17)
			{
				setenv("ethaddr", p);
			}
			else
				printf("%s: Illegal length %d for mac address\n", __func__, len);
			break;
		}
		else
		{
			while (at < at_end && *at != '\0')
				at++;
			if (at >= at_end)
			{
				ret = -EINVAL;
				break;
			}
			else
				at++;
		}

	}
	if (ret)
		printf("%s: FEC_MAC_ADDR not present in eeprom\n", __func__);

	return ret;
}

int board_eth_init(bd_t *bis)
{
	int ret;
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	struct eth_device *fec;
	uchar mac_addr[6];

	setup_iomux_enet();


#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf << 1), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("%s: Using phy[%d]\n", __func__, phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("%s: fec_probe failed\n", __func__);
		free(phydev);
		free(bus);
		return ret;
	}
	fec = eth_get_dev_by_name("FEC");
	if (!fec) {
		printf("%s: Could not get FEC\n", __func__);
		return -EINVAL;
	}
	get_mac_addr();
	ret = eth_getenv_enetaddr("ethaddr", mac_addr);


	if (!ret) {
		printf("%s: No environment MAC for FEC\n", __func__);
		return -EINVAL;
	}

	printf("%s: Using MAC %pM for FEC\n", __func__, mac_addr);
	memcpy(fec->enetaddr, mac_addr, 6);
	ret = fec->write_hwaddr(fec);
	return ret;


#else
	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return ret;
#endif
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	return 0;
}
#endif

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	imx_iomux_v3_setup_multiple_pads(extra_pads, ARRAY_SIZE(extra_pads));
	imx_iomux_v3_setup_multiple_pads(extra_nandf_pads, ARRAY_SIZE(extra_nandf_pads));
	imx_iomux_v3_setup_multiple_pads(extra_nvcc_gpio_pads, ARRAY_SIZE(extra_nvcc_gpio_pads));
	setup_iomux_uart();
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	/* Bring up basic power for serial debug etc	*/

	gpio_direction_output(GPIO_PCIE_RST_N, 1);
	gpio_direction_output(GPIO_EEPROM_WP, 0);
	gpio_direction_output(GPIO_LAN2_EE_WP, 0);
	gpio_direction_output(GPIO_SPI_NOR_WP, 0);
	gpio_direction_output(GPIO_CAP_TOUCH_PWR, 1);
	gpio_direction_output(GPIO_LCD_EN, 0);
	gpio_direction_output(GPIO_BL_EN, 0);
	/* gpio_direction_output(GPIO_BL_PWM, 0); */
	gpio_direction_output(GPIO_FAN_EN, 0);
#ifndef CONFIG_SPL_BUILD
	gpio_direction_output(GPIO_BUZ_INT_EN, 0);
#else
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#endif
	gpio_direction_output(GPIO_BUZ_EXT_EN, 0);
	gpio_direction_output(GPIO_LVDS_ROTATE, 0);
	gpio_direction_output(GPIO_AUX_5V, 1);
	gpio_direction_input(GPIO_DIMM_DN);
	gpio_direction_input(GPIO_DIMM_UP);

#ifdef CONFIG_MXC_SPI
	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
#endif


	return 0;
}



#ifdef CONFIG_CMD_SATA

int setup_sata(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;

	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
			IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
			|IOMUXC_GPR13_SATA_PHY_7_SATA2M
			|IOMUXC_GPR13_SATA_SPEED_3G
			|(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			|IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			|IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			|IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			|IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			|IOMUXC_GPR13_SATA_PHY_1_SLOW);

	return 0;
}
#endif

static int check_version(void)
{
	int ret;
	eeprom_addr = CONFIG_EEPROM_ADDR2;
	i2c_set_bus_num(CONFIG_EEPROM_ON_BUS);
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	i2c_set_bus_speed(CONFIG_SYS_I2C_SPEED);
	ret = i2c_probe(eeprom_addr);
	if (ret == 0)
	{
		printf("%s: I2C EEPROM at 0x%x - DIN RAIL version\n", __func__, eeprom_addr);

	}
	else
	{
		eeprom_addr = CONFIG_EEPROM_ADDR;
		ret = i2c_probe(eeprom_addr);
		if (ret == 0)
		{
			printf("%s: I2C EEPROM at 0x%x - PANEL version\n", __func__, eeprom_addr);

			panel_version = 1;
		}
	}
	if (ret)
	{
		printf("%s: No I2C EEPROM\n", __func__);
		return ret;
	}

	return 0;
}

static int update_env(int is_panel)
{
	if (is_panel)
	{

		imx_pwm_config(0, 2500000, 5000000);
		gpio_direction_output(GPIO_LCD_EN, 1);
		gpio_direction_output(GPIO_BL_EN, 1);
		mdelay(2);
		imx_pwm_enable(0);
		setenv("ftd_file", "/boot/rrm10.dtb");
	/* gpio_direction_output(GPIO_BL_PWM, 1); */
	}
	else
	{
		setenv("fdt_file", "/boot/rrm10-hdmi.dtb");
		setenv("panel", "HDMI");
	}

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	int ret;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#ifdef CONFIG_CMD_I2C
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

#endif

#ifdef CONFIG_CMD_SATA
	ret = setup_sata();
	if (ret)
	{
		return ret;
	}
#endif
	gpio_direction_output(GPIO_USB_H1_EN, 1);
	mdelay(10);
	gpio_direction_output(GPIO_USBP0_EN, 1);
	mdelay(1);
	gpio_direction_output(GPIO_USBP1_EN, 1);
	mdelay(1);
	gpio_direction_output(GPIO_USBP2_EN, 1);
	mdelay(1);
	gpio_direction_output(GPIO_USBP3_EN, 1);
	mdelay(100);

	writel(ANATOP_MISC1_CLK1_IBEN, &anatop->ana_misc1_clr);
	writel(ANATOP_MISC1_CLK1_OBEN, &anatop->ana_misc1_set);
	check_version();
	if (panel_version)
		setenv("panel", "RRM10-XGA");
	else
		setenv("panel", "HDMI");

	setup_display();
	return 0;
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
int board_late_init(void)
{
	int rep;
	ulong ticks;
	char *kbd;

	cmd_process(0, 2, usbcmd, &rep, &ticks);
	update_env(panel_version);

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
#ifdef CONFIG_IMX_WATCHDOG
	hw_watchdog_init();
#endif

	kbd=getenv("hasusbkbd");
	if (kbd && (strcmp(kbd, "yes") == 0))
	{
		setenv("stdout", "vga");
		setenv("stderr", "vga");
	}
	return 0;
}

int checkboard(void)
{
	printf("Board: MX6Q-RRM10\n");

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	char *allow_usb = getenv("allow_usbkbd");
	if (allow_usb && (strcmp(allow_usb, "yes") == 0))
		return 0;
	else
		return 1;
}

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi	= (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
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
static struct display_info_t const displays[] = {{
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
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "RRM10-XGA",
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
} } };

int board_video_skip(void)
{
	int ret;
	ret = ipuv3_fb_init(&displays[panel_version].mode, 0,
				displays[panel_version].pixfmt);
	if (!ret) {
		displays[panel_version].enable(displays+panel_version);
		printf("Display: %s (%ux%u)\n",
			   displays[panel_version].mode.name,
			   displays[panel_version].mode.xres,
			   displays[panel_version].mode.yres);
	} else
		printf("LCD %s cannot be configured: %d\n",
			   displays[panel_version].mode.name, ret);
	return 0;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
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
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT
	     | IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

}
#endif /* CONFIG_VIDEO_IPUV3 */





