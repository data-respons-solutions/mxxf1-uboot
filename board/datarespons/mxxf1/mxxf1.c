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
#include <asm/mach-imx/sata.h>
#include <errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/mach-imx/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
//#include "../common/pfuze.h"
#include <usb.h>
#include <pwm.h>
#include <version.h>
#include <watchdog.h>


#define CMD_CRC32

#define USE_PWM_FOR_BL

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define SLOWOUT_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define OUT_LOW_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define REGINP_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_DISABLE  | PAD_CTL_HYS)

#define ENET_PAD_CTRL_UP  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define ENET_PAD_CTRL_DN  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#include "mxxf1_gpio.h"

typedef enum  { SW1AB, SW1C, SW3AB } pf100_regs;
typedef enum {VER_PANEL, VER_DIN, VER_UNKNOWN} PanelVersion;
#ifndef CONFIG_SPL_BUILD
static PanelVersion board_version = VER_UNKNOWN;
#endif
int vpd_update_eeprom(char *touch_fw_ver);

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

iomux_v3_cfg_t const extra_nandf_pads[] = {
	IOMUX_PADS(PAD_NANDF_CS0__GPIO6_IO11		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* SPI_NOR_WP	*/
	IOMUX_PADS(PAD_NANDF_CS1__GPIO6_IO14		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* LAN2_EEWP	*/
	IOMUX_PADS(PAD_NANDF_CS2__GPIO6_IO15		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* LCD_EN		*/
	IOMUX_PADS(PAD_NANDF_CS3__GPIO6_IO16		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* CAP_TCH_RST	*/

	IOMUX_PADS(PAD_NANDF_ALE__GPIO6_IO08		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* CAP_TCH_INT	*/
	IOMUX_PADS(PAD_NANDF_CLE__GPIO6_IO07		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* USBP3_OC		*/
	IOMUX_PADS(PAD_NANDF_WP_B__GPIO6_IO09		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* USBP3_EN		*/
	IOMUX_PADS(PAD_NANDF_RB0__GPIO6_IO10		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* AUX_5V_EN	*/

	IOMUX_PADS(PAD_NANDF_D2__GPIO2_IO02		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* USBP1_OC		*/
	IOMUX_PADS(PAD_NANDF_D3__GPIO2_IO03		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* USBP1_EN		*/

	IOMUX_PADS(PAD_NANDF_D4__GPIO2_IO04		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* WDOG_PWR_EN	*/
	IOMUX_PADS(PAD_NANDF_D5__GPIO2_IO05		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* EMMC_RST#	*/
	IOMUX_PADS(PAD_NANDF_D6__GPIO2_IO06		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* USBP2_OC		*/
	IOMUX_PADS(PAD_NANDF_D7__GPIO2_IO07		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* USBP2_EN		*/
};

iomux_v3_cfg_t const extra_nvcc_gpio_pads[] = {
	IOMUX_PADS(PAD_KEY_COL0__GPIO4_IO06		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* DIMM-	*/
	IOMUX_PADS(PAD_KEY_ROW0__GPIO4_IO07		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* DIMM+	*/
	IOMUX_PADS(PAD_KEY_COL1__GPIO4_IO08		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* TEMP_OS	*/

	IOMUX_PADS(PAD_KEY_COL2__GPIO4_IO10		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* CODEC_PWR_EN	*/
	IOMUX_PADS(PAD_KEY_ROW2__GPIO4_IO11		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* HDMI_CEC_IN	*/

	IOMUX_PADS(PAD_KEY_COL4__FLEXCAN2_TX		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* CAN2_TX	*/
	IOMUX_PADS(PAD_KEY_ROW4__FLEXCAN2_RX		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* CAN2_RX	*/

	IOMUX_PADS(PAD_GPIO_0__CCM_CLKO1			| MUX_PAD_CTRL(PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_HIGH)),	/* AUD_MCLK	*/
	IOMUX_PADS(PAD_GPIO_1__WDOG2_B				| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* WDOG_B		*/
	IOMUX_PADS(PAD_GPIO_2__GPIO1_IO02			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* BLON_3V_H	*/

	IOMUX_PADS(PAD_GPIO_4__GPIO1_IO04			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* TCH_PWRON	*/
	IOMUX_PADS(PAD_GPIO_5__GPIO1_IO05			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* EEPROM_WP	*/
	IOMUX_PADS(PAD_GPIO_7__FLEXCAN1_TX			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* CAN1_TX	*/

	IOMUX_PADS(PAD_GPIO_8__FLEXCAN1_RX			| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* CAN1_RX		*/
#ifdef USE_PWM_FOR_BL
	IOMUX_PADS(PAD_GPIO_9__PWM1_OUT			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/*  BL_PWM		*/
#else
	IOMUX_PADS(PAD_GPIO_9__GPIO1_IO09			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),
#endif
	IOMUX_PADS(PAD_GPIO_17__GPIO7_IO12			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* PCIE_RST	*/
	IOMUX_PADS(PAD_GPIO_18__GPIO7_IO13			| MUX_PAD_CTRL(PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_100K_DOWN)),	/* PMIC_INT_B	*/

};

iomux_v3_cfg_t const extra_pads[] = {
	IOMUX_PADS(PAD_ENET_RX_ER__GPIO1_IO24		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* USB_OTG_ID	*/
	IOMUX_PADS(PAD_ENET_RXD0__GPIO1_IO27		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* UOK_B		*/
	IOMUX_PADS(PAD_ENET_TXD1__GPIO1_IO29		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* USB_H1_PWR_EN	*/
	IOMUX_PADS(PAD_SD1_CMD__GPIO1_IO18			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* FAN_EN		*/
	IOMUX_PADS(PAD_SD1_DAT1__PWM3_OUT			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* BUZ_INT_EN	*/
	IOMUX_PADS(PAD_SD1_DAT2__GPIO1_IO19		| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* BUZ_EXT_EN	*/

	IOMUX_PADS(PAD_CSI0_DATA_EN__GPIO5_IO20	| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* PCIE_WAKE_B	*/

	IOMUX_PADS(PAD_EIM_D21__GPIO3_IO21			| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* USB_OTG_OC		*/
	IOMUX_PADS(PAD_EIM_D22__GPIO3_IO22			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* USB_ORG_PWR_EN	*/
	IOMUX_PADS(PAD_EIM_D29__GPIO3_IO29			| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* PWR_BTN_SNS		*/
	IOMUX_PADS(PAD_EIM_D30__GPIO3_IO30			| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* USB_H1_OC		*/
	IOMUX_PADS(PAD_EIM_D31__GPIO3_IO31			| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* USB_P0_OC		*/
	IOMUX_PADS(PAD_EIM_D28__GPIO3_IO28			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* USBP0_EN			*/

	IOMUX_PADS(PAD_EIM_D23__GPIO3_IO23			| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* LAN2_DEV_OFF#	*/
	IOMUX_PADS(PAD_EIM_D24__GPIO3_IO24			| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* LAN2_SMB_ALRT#	*/
	IOMUX_PADS(PAD_GPIO_16__GPIO7_IO11			| MUX_PAD_CTRL(OUT_LOW_PAD_CTRL)),	/* LVDS_ROTATE		*/

};

iomux_v3_cfg_t const audio_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT4__AUD3_TXC		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* AUD3_TXC		*/
	IOMUX_PADS(PAD_CSI0_DAT5__AUD3_TXD		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* AUD3_TXD		*/
	IOMUX_PADS(PAD_CSI0_DAT6__AUD3_TXFS	| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL)),	/* AUD3_TXFS	*/
	IOMUX_PADS(PAD_CSI0_DAT7__AUD3_RXD		| MUX_PAD_CTRL(REGINP_PAD_CTRL)),	/* AUD3_RXD		*/
};

iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL_UP)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC			| MUX_PAD_CTRL(ENET_PAD_CTRL_UP)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3		| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL_DN)),
	/* AR8031 PHY Reset */
	IOMUX_PADS(PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(ENET_PAD_CTRL_UP)),
	IOMUX_PADS(PAD_ENET_TX_EN__GPIO1_IO28		| MUX_PAD_CTRL(ENET_PAD_CTRL_UP)), 	/* ENET_WOL_INT (low)	*/
	IOMUX_PADS(PAD_ENET_RXD1__GPIO1_IO26		| MUX_PAD_CTRL(ENET_PAD_CTRL_UP)), 	/* RGMII_INT (low)	*/
};


iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D0__GPIO2_IO00    | MUX_PAD_CTRL(REGINP_PAD_CTRL)), 		/* CD 				*/
	IOMUX_PADS(PAD_NANDF_D1__GPIO2_IO01    | MUX_PAD_CTRL(REGINP_PAD_CTRL)),		/* WP 				*/
	IOMUX_PADS(PAD_SD3_RST__GPIO7_IO08	  | MUX_PAD_CTRL(REGINP_PAD_CTRL)),		/* HEADPHONE_DET	*/
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D5__GPIO2_IO05    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),	/* eMMC reset */
};

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 26)
	}
};

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_GPIO_3__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_GPIO_3__GPIO1_IO03 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_GPIO_6__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_GPIO_6__GPIO1_IO06 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 6)
	}
};

iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS1 */
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19  | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
};

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
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
#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    SD3
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			SETUP_IOMUX_PADS(usdhc3_pads);
			gpio_direction_input(USDHC3_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 1:
			SETUP_IOMUX_PADS(usdhc4_pads);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

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
		SETUP_IOMUX_PADS(usdhc3_pads);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x3:
		SETUP_IOMUX_PADS(usdhc4_pads);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

int overwrite_console(void)
{
	return 0;
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

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_VIDEO_IPUV3)
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
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
			.name           = "MXXF1-XGA",
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
#else
	void setup_display(void) {}
#endif /* CONFIG_VIDEO_IPUV3 */


#ifndef CONFIG_SPL_BUILD
#define MAX_I2C_DATA_LEN 10
static char egalax_fw[32];
static int egalax_firmware_version(void)
{
	int to=100000;
	static const uint8_t cmd[MAX_I2C_DATA_LEN] = { 0x03, 0x03, 0xa, 0x01, 'D', 0, 0, 0, 0, 0 };
	uint8_t rcv_buf[MAX_I2C_DATA_LEN+1];

	int ret;
	gpio_set_value(GPIO_CAP_TOUCH_RST, 0);
	udelay(500);
	gpio_set_value(GPIO_CAP_TOUCH_RST, 1);
	udelay(500);
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
			printf("%s: FW version [%s]\n", __func__, egalax_fw);
		}
		else
		{
			printf("%s: Got touch message %x.%x.%x\n", __func__, rcv_buf[0], rcv_buf[1], rcv_buf[2]);
			return -EINVAL;
		}
	}

	gpio_set_value(GPIO_CAP_TOUCH_RST, 0);
	udelay(500);
	gpio_set_value(GPIO_CAP_TOUCH_RST, 1);
	return 0;
}

static void setup_iomux_enet(void)
{

	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	gpio_direction_output(GPIO_CAP_TOUCH_RST, 0);

	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
	gpio_set_value(GPIO_CAP_TOUCH_RST, 1);
}


int mxxf1_eeprom_init (unsigned dev_addr);
int eeprom_get_mac_addr(void);
int eeprom_addr;

static PanelVersion check_version(void)
{
	PanelVersion ret=VER_UNKNOWN;
	eeprom_addr = MXXF1_EEPROM_ADDR2;
	i2c_set_bus_num(1);
	i2c_set_bus_speed(CONFIG_SYS_I2C_SPEED);
	ret = i2c_probe(eeprom_addr);

	if (ret == 0)
	{
		printf("%s: I2C EEPROM at 0x%x - DIN RAIL version\n", __func__, eeprom_addr);
		ret = VER_DIN;
	}
	else
	{
		eeprom_addr = MXXF1_EEPROM_ADDR;
		ret = i2c_probe(eeprom_addr);
		if (ret == 0)
		{
			printf("%s: I2C EEPROM at 0x%x - PANEL version\n", __func__, eeprom_addr);
			ret = VER_PANEL;
		}
	}

	return ret;
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

#ifndef CONFIG_SPL_BUILD
static void setup_usb(void)
{
	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 0);
}
#endif


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
	case 1:
		if (on) {
			gpio_direction_output(GPIO_USB_H1_EN, 1);
			mdelay(10);
			gpio_direction_output(GPIO_USBP0_EN, 1);
			mdelay(1);
			gpio_direction_output(GPIO_USBP1_EN, 1);
			mdelay(1);
			gpio_direction_output(GPIO_USBP2_EN, 1);
			mdelay(1);
			gpio_direction_output(GPIO_USBP3_EN, 1);
			mdelay(2000);
		}
		else {
			gpio_direction_output(GPIO_USB_H1_EN, 0);
			gpio_direction_output(GPIO_USBP0_EN, 0);
			gpio_direction_output(GPIO_USBP1_EN, 0);
			gpio_direction_output(GPIO_USBP2_EN, 0);
			gpio_direction_output(GPIO_USBP3_EN, 0);
		}
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif
static const u32 buzzer_period = 1000000000/2300;

int board_early_init_f(void)
{

	SETUP_IOMUX_PADS(enet_pads);
	SETUP_IOMUX_PADS(extra_pads);
	SETUP_IOMUX_PADS(extra_nandf_pads);
	SETUP_IOMUX_PADS(extra_nvcc_gpio_pads);
	SETUP_IOMUX_PADS(uart1_pads);
	SETUP_IOMUX_PADS(ecspi1_pads);
	/* Bring up basic power for serial debug etc	*/

	gpio_direction_output(GPIO_PCIE_RST_N, 1);
	gpio_direction_output(GPIO_EEPROM_WP, 1);
	gpio_direction_output(GPIO_LAN2_EE_WP, 1);
	gpio_direction_output(GPIO_SPI_NOR_WP, 1);
	gpio_direction_output(GPIO_CAP_TOUCH_PWR, 1);
	gpio_direction_output(GPIO_LCD_EN, 0);
	gpio_direction_output(GPIO_BL_EN, 0);
	/* gpio_direction_output(GPIO_BL_PWM, 0); */
	gpio_direction_output(GPIO_FAN_EN, 0);
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	gpio_direction_output(GPIO_BUZ_EXT_EN, 0);
	gpio_direction_output(GPIO_LVDS_ROTATE, 0);
	gpio_direction_output(GPIO_AUX_5V, 1);
	gpio_direction_input(GPIO_DIMM_DN);
	gpio_direction_input(GPIO_DIMM_UP);
	gpio_direction_input(GPIO_TOUCH_IRQ);

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
#ifndef CONFIG_SPL_BUILD
	if (pwm_config(2, buzzer_period/2, buzzer_period))
		printf("%s: pwm_config for buzzer ERROR\n", __func__);
	pwm_disable(2);
#endif
	return 0;
}

#ifndef CONFIG_SPL_BUILD
int board_init(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;


	if (get_imx_reset_cause() == 0x00010) {
		clrbits_le32(&iomuxc_regs->gpr[12], IOMUXC_GPR12_APPS_LTSSM_ENABLE);
	}

	printf("Init\n");
	board_version = check_version();

	setup_display();

#ifdef USE_PWM_FOR_BL
	if (pwm_config(0, 2500, 5000))
		printf("%s: pwm_config for backlight ERROR\n", __func__);

	pwm_enable(0);
#else
	gpio_direction_output(GPIO_BL_PWM, 1);
#endif
	gpio_set_value(GPIO_LCD_EN, 1);
	gpio_set_value(GPIO_BL_EN, 1);
	setup_usb();
	setup_sata();

	if (mxxf1_eeprom_init(eeprom_addr))
	{
		printf("%s: Failed to init VPD eeprom\n", __func__);
	}
	return 0;
}
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 1) ? (IMX_GPIO_NR(3, 19)) : -1;
}
#endif

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
	char *version_from_env;
	char *fdt_defined;


	version_from_env = env_get("UBOOT_VERSION");
	if (version_from_env == 0 || strncmp(U_BOOT_VERSION, version_from_env, 128))
	{
		if (board_version == VER_PANEL)
		{
			env_set("stdout", "vga");
			env_set("stderr", "vga");
		}
		printf("U-Boot version [%s] differs from env [%s] - update\n", U_BOOT_VERSION, version_from_env);
		cmd_process(0, 3, reset_env_cmd, &rep, &ticks);
		env_set("UBOOT_VERSION", U_BOOT_VERSION);
		cmd_process(0, 2, save_env_cmd, &rep, &ticks);
	}

	fdt_defined = env_get("fdt_file");
	switch (board_version)
	{
	case VER_DIN:
		if (NULL == fdt_defined)
			env_set("fdt_file", "/boot/mxxf1-hdmi.dtb");
		break;

	case VER_PANEL:
		env_set("stdout", "vga");
		env_set("stderr", "vga");
		if (NULL == fdt_defined)
			env_set("fdt_file", "/boot/mxxf1.dtb");
		break;

	default:
		break;
	}
	printf("MXXF1 U-BOOT version [%s]\n", U_BOOT_VERSION);
	if (egalax_firmware_version() == 0)
		vpd_update_eeprom(egalax_fw);
	else
		vpd_update_eeprom(0);


	cmd_process(0, 2, usbcmd, &rep, &ticks);
	eeprom_get_mac_addr();

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}
#endif

int checkboard(void)
{
	puts("Board: MXXF1\n");
	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#include <asm/arch/mx6-ddr.h>
int mx6_ddr_init(ulong addr);

const struct mx6dq_iomux_ddr_regs mx6_ddr_ioregs = {
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

const struct mx6dq_iomux_grp_regs mx6_grp_ioregs = {
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
	.p0_mpwrdlctl =  0x40404040,
	.p1_mpwrdlctl =  0x40404040,
};

const struct mx6_mmdc_calibration mx6_mmcd_calib_static = {
	/* write leveling calibration */
	.p0_mpwldectrl0 = 0x00160018,
	.p0_mpwldectrl1 = 0x0025001A,
	.p1_mpwldectrl0 = 0x0017002B,
	.p1_mpwldectrl1 = 0x0017002F,
	/* Read DQS Gating calibration */
	.p0_mpdgctrl0 = 0x032C0338,
	.p0_mpdgctrl1 = 0x03280324,
	.p1_mpdgctrl0 = 0x03300340,
	.p1_mpdgctrl1 = 0x03280268,
	/* Read Calibration: DQS delay relative to DQ read access */
	.p0_mprddlctl = 0x3C343234,
	.p1_mprddlctl = 0x3636303E,
	/* Write Calibration: DQ/DM delay relative to DQS write access */
	.p0_mpwrdlctl = 0x363A3C3C,
	.p1_mpwrdlctl = 0x4630483C,
};


static struct mx6_mmdc_calibration calib = {0};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 1600,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
	.SRT = 1,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

int mxxf1_pmic_setup(void)
{
	int ret;
	i2c_set_bus_num(1);
	ret = i2c_probe(CONFIG_POWER_PFUZE100_I2C_ADDR);
	if (ret)
	{
		printf("%s: no pmic\n", __func__);
		return -ENODEV;
	}
	return 0;
}


static int mxxf1_pmic_set(pf100_regs reg, int mV)
{
	u8 values[2];

	switch (reg) {

	case SW1AB:
		if (mV > 1425) {
			printf("%s: SW1AB max is 1425 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		values[0] = (mV - 300) / 25;
		i2c_write(0x08, PFUZE100_SW1ABVOL, 1, values, 1);
		break;

	case SW1C:
		if (mV > 1425) {
			printf("%s: SW1AB max is 1425 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		values[0] = (mV - 300) / 25;
		i2c_write(0x08, PFUZE100_SW1CVOL, 1, values, 1);
		break;

	case SW3AB:
		if (mV > 1500) {
			printf("%s: SW3AB max is 1500 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		values[0] = (mV - 400) / 25;
		i2c_write(0x08, PFUZE100_SW3AVOL, 1, values, 1);
		i2c_write(0x08, PFUZE100_SW3BVOL, 1, values, 1);
		break;
	}

	return 0;
}
/*
 * This section requires the differentiation between iMX6 Sabre boards, but
 * for now, it will configure only for the mx6q variant.
 */

static struct mx6_ddr_sysinfo sysinfo = {
	/* width of data bus:0=16,1=32,2=64 */
	.dsize = 2,
	/* config for full 4GB range so that get_mem_size() works */
	.cs_density = 32, /* 32Gb per CS */
	/* single chip select */
	.ncs = 1,
	.cs1_mirror = 0,
	.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
	.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
	.walat = 1,	/* Write additional latency */
	.ralat = 5,	/* Read additional latency */
	.mif3_mode = 3,	/* Command prediction working mode */
	.bi_on = 1,	/* Bank interleaving enabled */
	.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.ddr_type = DDR_TYPE_DDR3,
	.refsel = 1,	/* Refresh cycles at 32KHz */
	.refr = 7,	/* 4 refresh commands per refresh cycle */
};


static void spl_dram_init(void)
{
	mx6dq_dram_iocfg(64, &mx6_ddr_ioregs, &mx6_grp_ioregs);
#ifdef DYNAMIC_CALIB
	mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
#else
	mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib_static, &mem_ddr);
#endif
}

static void do_hang_error(void)
{
	for(;;) {
		udelay(1000000);
	}
}

void board_init_f(ulong dummy)
{
	int err, n;
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
	err = mxxf1_pmic_setup();
	if (err == 0) {
		mxxf1_pmic_set(SW1AB, 1425);
		mxxf1_pmic_set(SW1C, 1425);
		mxxf1_pmic_set(SW3AB, 1350);
		udelay(10000);
	}

	/* DDR initialization */


	spl_dram_init();
#ifdef DYNAMIC_CALIB
	printf("Calibrating DDR3\n");
	err = mmdc_do_write_level_calibration(&sysinfo);
	if (err & 0x03) {
		printf("DDR3 write level calibration error - hang\n");
		do_hang_error();

	}

	err = mmdc_do_dqs_calibration(&sysinfo);
	if (err) {
		printf("DDR3 DQS calibration error - hang\n");
		do_hang_error();
	}
#else
	printf("Using static calibration values\n");
#endif
	mmdc_read_calibration(&sysinfo, &calib);
	printf("Calibration results for PHY0\n");
	printf("mpwldectrl0 = %08x\n", calib.p0_mpwldectrl0);
	printf("mpwldectrl1 = %08x\n", calib.p0_mpwldectrl1);
	printf("mpdgctrl0   = %08x\n", calib.p0_mpdgctrl0);
	printf("mpdgctrl1   = %08x\n", calib.p0_mpdgctrl1);
	printf("mprddlctl   = %08x\n", calib.p0_mprddlctl);
	printf("mpwrdlctl   = %08x\n", calib.p0_mpwrdlctl);
	printf("\nCalibration results for PHY1\n");
	printf("mpwldectrl0 = %08x\n", calib.p1_mpwldectrl0);
	printf("mpwldectrl1 = %08x\n", calib.p1_mpwldectrl1);
	printf("mpdgctrl0   = %08x\n", calib.p1_mpdgctrl0);
	printf("mpdgctrl1   = %08x\n", calib.p1_mpdgctrl1);
	printf("mprddlctl   = %08x\n", calib.p1_mprddlctl);
	printf("mpwrdlctl   = %08x\n", calib.p1_mpwrdlctl);
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

#ifndef CONFIG_SPL_WATCHDOG_SUPPORT
void reset_cpu(ulong addr)
{
}
#endif
#endif
