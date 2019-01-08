/*
 * Copyright (C) 2017 Data Respons AS
 *
 * Author: Hans Christian Lonstad <hcl@datarespons.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/spi.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <input.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#include <environment.h>
#include <usb.h>
#include <pwm.h>
#include <version.h>
#include <watchdog.h>

#include <asm/mach-imx/hab.h>
#include <vsprintf.h>

DECLARE_GLOBAL_DATA_PTR;

#include "../lm-common/lm_common_defs.h"

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

enum gpio_ids {
	GPIO_WL_VDDIO_EN = IMX_GPIO_NR(6, 11),
	GPIO_BT_WAKE = IMX_GPIO_NR(6, 8),
	GPIO_BT_REG_ON = IMX_GPIO_NR(2, 4),
	GPIO_WL_REG_ON = IMX_GPIO_NR(2, 3),
	GPIO_WL_DEV_WAKE = IMX_GPIO_NR(2, 7),
	GPIO_SPI_NOR_WP = IMX_GPIO_NR(3, 20),
	GPIO_WL_BAT_PWR_EN = IMX_GPIO_NR(3, 23),
	GPIO_HW_SETTING0 = IMX_GPIO_NR(4, 21),
	GPIO_HW_SETTING1 = IMX_GPIO_NR(4, 22),
	GPIO_HW_SETTING2 = IMX_GPIO_NR(4, 23),
	GPIO_PWR_BTN = IMX_GPIO_NR(3, 29),
	SPI_CS_GPIO = IMX_GPIO_NR(3, 19),

	CAN1_WAKE = IMX_GPIO_NR(4, 18),
	CAN1_EN = IMX_GPIO_NR(4, 10),
	CAN1_STB_N = IMX_GPIO_NR(4, 12),
	CAN1_RES_EN = IMX_GPIO_NR(3, 31),

	CAN2_WAKE = IMX_GPIO_NR(4, 19),
	CAN2_EN = IMX_GPIO_NR(1, 9),
	CAN2_STB_N = IMX_GPIO_NR(1, 2),
	CAN2_RES_EN = IMX_GPIO_NR(1, 6),

	EN_ANI1 = IMX_GPIO_NR(5, 15),
	EN_ANI2 = IMX_GPIO_NR(5, 16),
	STATUS_CPU = IMX_GPIO_NR(4, 31),

	GP_EN_GPO1 = IMX_GPIO_NR(4, 24),
	GP_EN_GPO2 = IMX_GPIO_NR(4, 25),

	GP_EN_PWR_ANT_GPS = IMX_GPIO_NR(4, 26),
	GP_PRST_WWAN_N = IMX_GPIO_NR(4, 27),
	GP_DISABLE_WWAN_N = IMX_GPIO_NR(4, 28),

	GP_ENABLE_LC_UART = IMX_GPIO_NR(7, 13),
	TPM_RESET_N = IMX_GPIO_NR(4, 8),
	GP_NINT_MCU = IMX_GPIO_NR(5, 15),

	GP_NRST_MCU = IMX_GPIO_NR(1, 4),

	GP_LED_R_STATUS = IMX_GPIO_NR(1, 18),
	GP_LED_G_STATUS = IMX_GPIO_NR(1, 20),

	GPIO_OTG_PWR = IMX_GPIO_NR(3, 22),
	ENET_PHY_RESET = IMX_GPIO_NR(1, 25)
};

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_18__GPIO7_IO13	      | MUX_PAD_CTRL(INPUT_PD_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	IOMUX_PADS(PAD_EIM_D26__UART2_TX_DATA 	| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D27__UART2_RX_DATA 	| MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart3_pads[] = {
	IOMUX_PADS(PAD_EIM_D24__UART3_TX_DATA 	| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D25__UART3_RX_DATA 	| MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart4_pads[] = {
	IOMUX_PADS(PAD_KEY_COL0__UART4_TX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW0__UART4_RX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL)),

};

static iomux_v3_cfg_t const uart5_pads_dte[] = {
	IOMUX_PADS(PAD_CSI0_DAT14__UART5_TX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT15__UART5_RX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT18__UART5_RTS_B		| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT19__UART5_CTS_B		| MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart5_pads_dce[] = {
	IOMUX_PADS(PAD_CSI0_DAT14__UART5_RX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT15__UART5_TX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT18__UART5_CTS_B		| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT19__UART5_RTS_B		| MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RXD1__GPIO1_IO26	| MUX_PAD_CTRL(WEAK_PULLUP)),			/* Phy Irq */
	/* AR8031 PHY Reset */
	IOMUX_PADS(PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL)),	/* Phy reset (low) */
};


static iomux_v3_cfg_t const usdhc3_pads[] = {
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
	IOMUX_PADS(PAD_SD3_RST__SD3_RESET  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const ecspi1_pads[] = {
	IOMUX_PADS(PAD_EIM_D16__ECSPI1_SCLK 	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D17__ECSPI1_MISO 	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D18__ECSPI1_MOSI 	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19 	| MUX_PAD_CTRL(SPI_PAD_CTRL)),	/* Use GPIO for CS 	*/
};

static iomux_v3_cfg_t const ecspi2_pads[] = {
	IOMUX_PADS(PAD_DISP0_DAT19__ECSPI2_SCLK 	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT17__ECSPI2_MISO 	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT16__ECSPI2_MOSI 	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT18__GPIO5_IO12 | MUX_PAD_CTRL(SPI_PAD_CTRL)),	/* Use GPIO for CS 	*/
	IOMUX_PADS(PAD_DISP0_DAT15__GPIO5_IO09 | MUX_PAD_CTRL(SPI_PAD_CTRL)),	/* Use GPIO for CS 	*/
};

static iomux_v3_cfg_t const hw_settings_pads[] = {
	IOMUX_PADS(PAD_DISP0_DAT0__GPIO4_IO21	| MUX_PAD_CTRL(NO_PULLUP)),	/* HW_Setting 0*/
	IOMUX_PADS(PAD_DISP0_DAT1__GPIO4_IO22	| MUX_PAD_CTRL(NO_PULLUP)),	/* HW Setting 1 */
	IOMUX_PADS(PAD_DISP0_DAT2__GPIO4_IO23	| MUX_PAD_CTRL(NO_PULLUP)),	/* HW Setting 2 */
};

static iomux_v3_cfg_t const otg_pads[] = {
	IOMUX_PADS(PAD_EIM_D21__USB_OTG_OC   	| MUX_PAD_CTRL(WEAK_PULLUP)),
	IOMUX_PADS(PAD_EIM_D22__USB_OTG_PWR  	| MUX_PAD_CTRL(NO_PULLUP)),
	IOMUX_PADS(PAD_KEY_ROW1__GPIO4_IO09 	| MUX_PAD_CTRL(NO_PULLUP)),	/* Note that this is not default, need DT config */
	IOMUX_PADS(PAD_ENET_RX_ER__USB_OTG_ID 	| MUX_PAD_CTRL(WEAK_PULLUP)),
};

static iomux_v3_cfg_t const can1_pads[] = {
	IOMUX_PADS(PAD_KEY_COL2__GPIO4_IO10		| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* GP_CAN1_EN */
	IOMUX_PADS(PAD_KEY_COL3__GPIO4_IO12		| MUX_PAD_CTRL(NO_PULLUP)),		/* GP_CAN1_STB */
	IOMUX_PADS(PAD_EIM_D31__GPIO3_IO31		| MUX_PAD_CTRL(WEAK_PULLUP)),	/* GP_CAN1_RES_EN */
	IOMUX_PADS(PAD_GPIO_7__FLEXCAN1_TX		| MUX_PAD_CTRL(NO_PULLUP)),
	IOMUX_PADS(PAD_GPIO_8__FLEXCAN1_RX		| MUX_PAD_CTRL(WEAK_PULLUP)),
	IOMUX_PADS(PAD_DI0_PIN2__GPIO4_IO18		| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* GP_CAN1_WAKE */
};

static iomux_v3_cfg_t const can2_pads[] = {
	IOMUX_PADS(PAD_GPIO_9__GPIO1_IO09		| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* GP_CAN2_EN */
	IOMUX_PADS(PAD_GPIO_2__GPIO1_IO02		| MUX_PAD_CTRL(NO_PULLUP)),		/* GP_CAN2_STB */
	IOMUX_PADS(PAD_GPIO_6__GPIO1_IO06		| MUX_PAD_CTRL(WEAK_PULLUP)),	/* GP_CAN2_RES_EN */
	IOMUX_PADS(PAD_KEY_COL4__FLEXCAN2_TX	| MUX_PAD_CTRL(NO_PULLUP)),
	IOMUX_PADS(PAD_KEY_ROW4__FLEXCAN2_RX	| MUX_PAD_CTRL(WEAK_PULLUP)),
	IOMUX_PADS(PAD_DI0_PIN3__GPIO4_IO19		| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* GP_CAN2_WAKE */
};

static iomux_v3_cfg_t const wwan_pads[] = {
	IOMUX_PADS(PAD_DISP0_DAT5__GPIO4_IO26	| MUX_PAD_CTRL(NO_PULLUP)),	/* GP_EN_PWR--ANT_GPS */
	IOMUX_PADS(PAD_DISP0_DAT6__GPIO4_IO27	| MUX_PAD_CTRL(NO_PULLUP)),	/* GP_nPRST_WWAN */
	IOMUX_PADS(PAD_DISP0_DAT7__GPIO4_IO28	| MUX_PAD_CTRL(NO_PULLUP)),	/* GP_nDISABLE_WWAN   */
	IOMUX_PADS(PAD_DISP0_DAT8__GPIO4_IO29	| MUX_PAD_CTRL(NO_PULLUP)),	/* GP_nWAKE_L--WWAN   */
	IOMUX_PADS(PAD_DISP0_DAT9__GPIO4_IO30	| MUX_PAD_CTRL(NO_PULLUP)),	/* LED_nWWAN   */
};

static iomux_v3_cfg_t const other_pads[] = {
	IOMUX_PADS(PAD_KEY_COL1__GPIO4_IO08		| MUX_PAD_CTRL(WEAK_PULLUP)),	/* GP_NRESET_TPM */

	IOMUX_PADS(PAD_CSI0_DAT16__GPIO6_IO02	| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* GP-EN--BUCK_5V--CPU */

	IOMUX_PADS(PAD_GPIO_0__ASRC_EXT_CLK		| MUX_PAD_CTRL(NO_PAD_CTRL)),	/* External audio clk */
	IOMUX_PADS(PAD_GPIO_1__WDOG2_B			| MUX_PAD_CTRL(NO_PULLUP)),

	IOMUX_PADS(PAD_NANDF_CLE__GPIO6_IO07	| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* GP--BT_HOST_WAKE */
	IOMUX_PADS(PAD_NANDF_CS0__GPIO6_IO11	| MUX_PAD_CTRL(WEAK_PULLDOWN)),		/* GP--WL_VDDIO_EN */
	IOMUX_PADS(PAD_NANDF_ALE__GPIO6_IO08	| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* GP--BT_WAKE */
	IOMUX_PADS(PAD_NANDF_RB0__GPIO6_IO10	| MUX_PAD_CTRL(NO_PAD_CTRL)),	/* PMIC_INT_B */
	IOMUX_PADS(PAD_NANDF_D3__GPIO2_IO03		| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* WL_REG_ON (WiFI power) */
	IOMUX_PADS(PAD_NANDF_D4__GPIO2_IO04		| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* BT_REG_ON (Bluetooth power) */
	IOMUX_PADS(PAD_NANDF_D5__GPIO2_IO05		| MUX_PAD_CTRL(NO_PAD_CTRL)),	/* WL_IRQ (WiFI interrupt */
	IOMUX_PADS(PAD_NANDF_D6__GPIO2_IO06		| MUX_PAD_CTRL(NO_PAD_CTRL)),	/* GP--WL_HOST_WAKE */
	IOMUX_PADS(PAD_NANDF_D7__GPIO2_IO07		| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* GP--WL_DEV_WAKE */
	IOMUX_PADS(PAD_EIM_D23__GPIO3_IO23		| MUX_PAD_CTRL(NO_PULLUP)),		/* GP--WL_BAT_PWR_EN */

	IOMUX_PADS(PAD_EIM_D29__GPIO3_IO29		| MUX_PAD_CTRL(WEAK_PULLUP)),	/* PPWR_BTN_SNS */
	IOMUX_PADS(PAD_CSI0_DAT4__AUD3_TXC		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT5__AUD3_TXD		| MUX_PAD_CTRL(NO_PAD_CTRL)),

	IOMUX_PADS(PAD_CSI0_DAT6__AUD3_TXFS		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT7__AUD3_RXD		| MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__GPIO1_IO14		| MUX_PAD_CTRL(INPUT_PD_PAD_CTRL)),	/* GPI_2 */
	IOMUX_PADS(PAD_SD2_DAT2__GPIO1_IO13		| MUX_PAD_CTRL(INPUT_PD_PAD_CTRL)),	/* GPI_3 */
	IOMUX_PADS(PAD_SD2_DAT3__GPIO1_IO12		| MUX_PAD_CTRL(INPUT_PD_PAD_CTRL)),	/* GPI_1 */

	IOMUX_PADS(PAD_ENET_TXD1__GPIO1_IO29	| MUX_PAD_CTRL(NO_PAD_CTRL)),	/* USB_H1_PWR */

	IOMUX_PADS(PAD_DI0_PIN15__GPIO4_IO17	| MUX_PAD_CTRL(WEAK_PULLDOWN)),		/* MCU Boot0 */
	IOMUX_PADS(PAD_DISP0_DAT3__GPIO4_IO24	| MUX_PAD_CTRL(NO_PULLUP)),		/* GP_EN--GPO1  */
	IOMUX_PADS(PAD_DISP0_DAT4__GPIO4_IO25	| MUX_PAD_CTRL(NO_PULLUP)),		/* GP_EN--GPO2  */
	IOMUX_PADS(PAD_DISP0_DAT21__GPIO5_IO15	| MUX_PAD_CTRL(WEAK_PULLUP)),	/* GPO_nINT_MCU */
	IOMUX_PADS(PAD_DISP0_DAT22__GPIO5_IO16	| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* EN_ANI_1 */
	IOMUX_PADS(PAD_DISP0_DAT23__GPIO5_IO17	| MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* EN_ANI_2 */
	IOMUX_PADS(PAD_DISP0_DAT10__GPIO4_IO31 	| MUX_PAD_CTRL(WEAK_PULLUP)),	/* GP_Status--CPU */
};

static iomux_v3_cfg_t const revab_pads[] = {
	IOMUX_PADS(PAD_GPIO_4__GPIO1_IO04		| MUX_PAD_CTRL(WEAK_PULLUP)),	/* GP_NRST_MCU1 */
};


static iomux_v3_cfg_t const revc_pads[] = {
	IOMUX_PADS(PAD_SD1_CMD__GPIO1_IO18		| MUX_PAD_CTRL(NO_PULLUP)),		/* GP_LED_G_STATUS */
	IOMUX_PADS(PAD_SD1_CLK__GPIO1_IO20		| MUX_PAD_CTRL(NO_PULLUP)),		/* GP_LED_R_STATUS */
	IOMUX_PADS(PAD_SD1_DAT0__GPIO1_IO16		| MUX_PAD_CTRL(NO_PULLUP)),		/* CAN1_ERR_N */
	IOMUX_PADS(PAD_SD1_DAT1__GPIO1_IO17		| MUX_PAD_CTRL(NO_PULLUP)),		/* CAN2_ERR_N */
	IOMUX_PADS(PAD_GPIO_4__GPIO1_IO04		| MUX_PAD_CTRL(NO_PULLUP)),	/* GP_NRST_MCU1 */
};


I2C_PADS(i2c1_pads,
		PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		IMX_GPIO_NR(5, 27),
		PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		IMX_GPIO_NR(5, 26) );

I2C_PADS(i2c2_pads,
		PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		IMX_GPIO_NR(4, 12),
		PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		IMX_GPIO_NR(4, 13) );

I2C_PADS(i2c3_pads,
		PAD_GPIO_3__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		PAD_GPIO_3__GPIO1_IO03 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		IMX_GPIO_NR(1, 3),
		PAD_GPIO_16__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		IMX_GPIO_NR(1, 6) );

#ifndef CONFIG_SPL_BUILD
static const char* hw_string[8] = {
	"REVA",
	"REVB",
	"REVC",
	"REVD",
	"REVE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
};
#endif

static int get_version(void)
{
	return (((!gpio_get_value(GPIO_HW_SETTING0)) << 2) |
			((!gpio_get_value(GPIO_HW_SETTING1)) << 1) |
			(!gpio_get_value(GPIO_HW_SETTING2)) ) & 7;
}

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

	SETUP_IOMUX_PADS(usdhc3_pads);
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	SETUP_IOMUX_PADS(usdhc4_pads);
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

int board_usb_phy_mode(int port)
{
	unsigned int bmode = readl(&src_base->sbmr2);

	switch(port) {
	case 0:
#ifdef CONFIG_FACTORY_BOOT
		return USB_INIT_DEVICE;
#else
		if (((bmode >> 24) & 0x03) == 0x01)	{
			printf("USB OTG in serial download mode\n");
			return USB_INIT_DEVICE;
		}
		else
			return USB_INIT_HOST;
#endif
		break;
	case 1:
		return USB_INIT_HOST;
		break;
	default:
		return USB_INIT_DEVICE;
		break;
	}
}

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;
	printf("%s: Setting enet ref clock to 125 MHz\n", __func__);
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

#ifndef CONFIG_SPL_BUILD

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);
	/* Reset AR8031 PHY */
	gpio_request(ENET_PHY_RESET, "enet-phy-reset");
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

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_USB_EHCI_MX6)
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_OVER_CUR_POL	(1 << 8) /* OTG Polarity of Overcurrent */
#define UCTRL_PWR_POL		(1 << 9)

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 2)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);
	if (port == 0)
	{
		setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);
		clrbits_le32(usbnc_usb_ctrl, UCTRL_OVER_CUR_POL);
	}

	return 0;
}

int board_ehci_power(int port, int on) {
	if (port == 0 && board_usb_phy_mode(0) == USB_INIT_HOST) {
		printf("VBUS is %s\n", on ? "on" : "off");
		gpio_set_value(GPIO_OTG_PWR, on);
		if (on)
			mdelay(600);
	}
	return 0;
}

int board_early_init_f()
{
	return 0;
}

static void setup_usb(void)
{
	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */

	if (is_mx6dq())
		imx_iomux_set_gpr_register(1, 13, 1, 0);
}
#endif

int setup_pins(void)
{
	SETUP_IOMUX_PADS(hw_settings_pads);
	gpio_request(GPIO_HW_SETTING0, "hwrev0");
	gpio_direction_input(GPIO_HW_SETTING0);
	gpio_request(GPIO_HW_SETTING1, "hwrev1");
	gpio_direction_input(GPIO_HW_SETTING1);
	gpio_request(GPIO_HW_SETTING2, "hwrev2");
	gpio_direction_input(GPIO_HW_SETTING2);

	SETUP_IOMUX_PADS(otg_pads);

	SETUP_IOMUX_PADS(uart1_pads);
	SETUP_IOMUX_PADS(uart2_pads);
	SETUP_IOMUX_PADS(uart3_pads);
	SETUP_IOMUX_PADS(uart4_pads);

	SETUP_IOMUX_PADS(can1_pads);
	SETUP_IOMUX_PADS(can2_pads);
	SETUP_IOMUX_PADS(ecspi1_pads);
	SETUP_IOMUX_PADS(ecspi2_pads);
	SETUP_IOMUX_PADS(other_pads);
	SETUP_IOMUX_PADS(wwan_pads);

	/* gpio_direction_output(GP_ENABLE_LC_UART, 0);	 Disable LC uart driver */
	gpio_request(TPM_RESET_N, "tpm-reset");
	gpio_direction_output(TPM_RESET_N, 1);
	gpio_request(GPIO_WL_BAT_PWR_EN, "bat-pwr");
	gpio_direction_output(GPIO_WL_BAT_PWR_EN, 0);
	gpio_request(GPIO_WL_VDDIO_EN, "vddio-en");
	gpio_direction_output(GPIO_WL_VDDIO_EN, 0);
	gpio_request(GPIO_WL_DEV_WAKE, "wl-dev-wake");
	gpio_direction_output(GPIO_WL_DEV_WAKE, 0);

	gpio_request(GPIO_WL_REG_ON, "wl-reg-on");
	gpio_direction_output(GPIO_WL_REG_ON, 0); 		/* WiFI off */
	gpio_request(GPIO_BT_REG_ON, "bt-reg-on");
	gpio_direction_output(GPIO_BT_REG_ON, 0);		/* Bluetooth off */

	gpio_request(GPIO_PWR_BTN, "pwr-btn");
	gpio_direction_input(GPIO_PWR_BTN);

	gpio_request(CAN1_WAKE, "can1-wake");
	gpio_direction_input(CAN1_WAKE);
	gpio_request(CAN1_EN, "can1-en");
	gpio_direction_output(CAN1_EN, 0);
	gpio_request(CAN1_STB_N, "can1-stb");
	gpio_direction_output(CAN1_STB_N, 1);
	gpio_request(CAN1_RES_EN, "can1-res-en");
	gpio_direction_output(CAN1_RES_EN, 0);

	gpio_request(CAN2_WAKE, "can2-wake");
	gpio_direction_input(CAN2_WAKE);
	gpio_request(CAN2_EN, "can2-en");
	gpio_direction_output(CAN2_EN, 0);
	gpio_request(CAN2_STB_N, "can2-stb");
	gpio_direction_output(CAN2_STB_N, 1);
	gpio_request(CAN2_RES_EN, "can2-res-en");
	gpio_direction_output(CAN2_RES_EN, 0);

	gpio_request(EN_ANI1, "ain1-en");
	gpio_direction_output(EN_ANI1, 0);
	gpio_request(EN_ANI2, "ain2-en");
	gpio_direction_output(EN_ANI2, 0);

	gpio_request(GP_NINT_MCU, "nint-mcu");
	gpio_direction_input(GP_NINT_MCU);

	gpio_request(GP_EN_GPO1, "en-gpo1");
	gpio_direction_output(GP_EN_GPO1, 0);
	gpio_request(GP_EN_GPO2, "en-gpo2");
	gpio_direction_output(GP_EN_GPO2, 0);
	gpio_request(GP_EN_PWR_ANT_GPS, "ant-pwr");
	gpio_direction_output(GP_EN_PWR_ANT_GPS, 0);

	gpio_request(GP_PRST_WWAN_N, "wwan-rst");
	gpio_direction_output(GP_PRST_WWAN_N, 0);
	gpio_request(GP_DISABLE_WWAN_N, "wwan-dis");
	gpio_direction_output(GP_DISABLE_WWAN_N, 0);

	int version = get_version();

	if (version == 0)
		SETUP_IOMUX_PADS(uart5_pads_dte);
	else
		SETUP_IOMUX_PADS(uart5_pads_dce);

	gpio_request(STATUS_CPU, "status-cpu");
	gpio_request(GP_NRST_MCU, "nrst-mcu");

	if (version >= 2) {
		SETUP_IOMUX_PADS(revc_pads);
		gpio_direction_input(STATUS_CPU);
		gpio_request(GP_LED_G_STATUS, "led-g");
		gpio_direction_output(GP_LED_G_STATUS, 0);
		gpio_request(GP_LED_R_STATUS, "led-r");
		gpio_direction_output(GP_LED_R_STATUS, 0);
		gpio_direction_output(GP_NRST_MCU, 0);
		gpio_request(GPIO_OTG_PWR, "otg-vbus-en");
		gpio_direction_output(GPIO_OTG_PWR, 1);
	}
	else {
		SETUP_IOMUX_PADS(revab_pads);
		gpio_direction_output(STATUS_CPU, 1);
		gpio_direction_output(GP_NRST_MCU, 1);
	}
	//setup_i2c(0, 100000, 0x7f, I2C_PADS_INFO(i2c1_pads));
	//setup_i2c(1, 100000, 0x7f, I2C_PADS_INFO(i2c2_pads));
	return 0;
}

#ifndef CONFIG_SPL_BUILD
static char * const usbcmd[] = {"usb", "start"};


int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	setup_pins();
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


int board_late_init(void)
{
	int rep;
	ulong ticks;
	int version = get_version();
	switch (version)
	{
	case 0:
		env_set("fdt_file", "/boot/cargotec-gw-revA.dtb");
		break;

	case 1:
		if (is_mx6dl())
			env_set("fdt_file", "/boot/cargotec-gw-revB-dl.dtb");
		else
			env_set("fdt_file", "/boot/cargotec-gw-revB-q.dtb");
		break;

	default:
		if (is_mx6dl())
			env_set("fdt_file", "/boot/cargotec-gw-revC-dl.dtb");
		else
			env_set("fdt_file", "/boot/cargotec-gw-revC-q.dtb");
		break;

	}
	setup_usb();
	gpio_set_value(GP_PRST_WWAN_N, 1);
	gpio_set_value(GP_DISABLE_WWAN_N, 1);
	cmd_process(0, 2, usbcmd, &rep, &ticks);
	printf("CARGOTEC GW version: %s\n", hw_string[version]);

	printf("U-BOOT version [%s]\n", U_BOOT_VERSION);
#ifdef CONFIG_SECURE_BOOT
	if (imx_hab_is_enabled())
	{
		printf("HAB enabled, setting up secure bootscript\n");
		env_set("bootscript", BOOTSCRIPT_SECURE);
		env_set("zimage", ZIMAGE_SECURE);
		env_set("initrd_file", "/boot/initrd-ivt_signed");
#ifdef CONFIG_BOOT_USB_INITRD
		env_set("bootscript_usb", BOOTSCRIPT_USB_SECURE);
#else
		env_set("bootscript_usb", BOOTSCRIPT_SECURE);
#endif
		//env_set("bootdelay", "0");
	}
	else
	{
		printf("HAB disabled, setting up regular bootscript\n");
		env_set("bootscript", BOOTSCRIPT_NOSECURE);
		env_set("bootscript_usb", BOOTSCRIPT_NOSECURE);
	}
#else
	env_set("bootscript", BOOTSCRIPT_NOSECURE);
	env_set("bootscript_usb", BOOTSCRIPT_NOSECURE);
#endif

	return 0;
}
#endif

int checkboard(void)
{
	puts("Board: Cargotec GW\n");
	return 0;
}

static int do_led(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[])
{
	if (argc < 2)
		return CMD_RET_USAGE;
	if (get_version() < 2)
		return 0;

	if (strncmp(argv[1], "red", 3) == 0)
	{
		gpio_set_value(GP_LED_G_STATUS, 0);
		gpio_set_value(GP_LED_R_STATUS, 1);
		return 0;
	}

	if (strncmp(argv[1], "green", 5) == 0)
	{
		gpio_set_value(GP_LED_G_STATUS, 1);
		gpio_set_value(GP_LED_R_STATUS, 0);
		return 0;
	}
	if (strncmp(argv[1], "off", 3) == 0)
	{
		gpio_set_value(GP_LED_G_STATUS, 0);
		gpio_set_value(GP_LED_R_STATUS, 0);
		return 0;
	}
	if (strncmp(argv[1], "orange", 6) == 0)
	{
		gpio_set_value(GP_LED_G_STATUS, 1);
		gpio_set_value(GP_LED_R_STATUS, 1);
		return 0;
	}
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	set_led, 2, 1, do_led, "LED control",
	"set_led color [red,green,off]\n"
	);

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
#include <spl.h>

const struct mx6sdl_iomux_ddr_regs mx6dl_ddr_ioregs = {
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

const struct mx6sdl_iomux_grp_regs mx6dl_grp_ioregs = {
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

const struct mx6dq_iomux_ddr_regs mx6q_ddr_ioregs = {
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

const struct mx6dq_iomux_grp_regs mx6q_grp_ioregs = {
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
	.p0_mpwldectrl0 =  0x001F001F,
	.p0_mpwldectrl1 =  0x001F001F,
	.p1_mpwldectrl0 =  0x001F001F,
	.p1_mpwldectrl1 =  0x001F001F,
	.p0_mpdgctrl0 =  0x40404040,
	.p0_mpdgctrl1 =  0x40404040,
	.p1_mpdgctrl0 =  0x40404040,
	.p1_mpdgctrl1 =  0x40404040,
	.p0_mprddlctl =  0x40404040,
	.p1_mprddlctl =  0x40404040,
	.p0_mpwrdlctl =  0x40404040,
	.p1_mpwrdlctl =  0x40404040,
};

const struct mx6_mmdc_calibration mx6q_mmcd_calib = {
	.p0_mpwldectrl0 =  0x001B0012,
	.p0_mpwldectrl1 =  0x0027001D,
	.p1_mpwldectrl0 =  0x00100021,
	.p1_mpwldectrl1 =  0x000E001C,

	.p0_mpdgctrl0 =  0x43180330,
	.p0_mpdgctrl1 =  0x031C0310,
	.p1_mpdgctrl0 =  0x4320032C,
	.p1_mpdgctrl1 =  0x031C0264,

	.p0_mprddlctl =  0x4C40444A,
	.p1_mprddlctl =  0x46403E4E,
	.p0_mpwrdlctl =  0x32383836,
	.p1_mpwrdlctl =  0x3A304438,
};

const struct mx6_mmdc_calibration mx6dl_mmcd_calib = {
	.p0_mpwldectrl0 =  0x0048004D,
	.p0_mpwldectrl1 =  0x003F0041,

	.p0_mpdgctrl0 =  0x423C023C,
	.p0_mpdgctrl1 =  0x02280224,

	.p0_mprddlctl =  0x40424A46,
	.p0_mpwrdlctl =  0x3638322E,
};

static struct mx6_ddr3_cfg mem_ddr_128x16 = {
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
};

static struct mx6_ddr_sysinfo sysinfo = {
	/* width of data bus:0=16,1=32,2=64 */
	.dsize = 1,
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
	.refr = 7,	/* 8 refresh commands per refresh cycle */
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

int pmic_setup(int bus)
{
	int ret;
	i2c_set_bus_num(bus);
	ret = i2c_probe(CONFIG_POWER_PFUZE100_I2C_ADDR);
	if (ret)
	{
		printf("%s: no pmic\n", __func__);
		return -ENODEV;
	}
	return 0;
}


static int pmic_set(int bus, pf100_regs reg, int mV)
{
	u8 values[2];
	i2c_set_bus_num(bus);
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
#ifdef GPIO_DDR_SETTING
	int four_chip = gpio_get_value(GPIO_DDR_SETTING);
#endif

	if (is_mx6dl())
	{
		mx6sdl_dram_iocfg(32, &mx6dl_ddr_ioregs, &mx6dl_grp_ioregs);
		sysinfo.dsize = 1;
		mx6_dram_cfg(&sysinfo, &mx6dl_mmcd_calib, &mem_ddr_128x16);
	}
	else
	{
		sysinfo.dsize = 2;
		mx6dq_dram_iocfg(64, &mx6q_ddr_ioregs, &mx6q_grp_ioregs);
		mx6_dram_cfg(&sysinfo, &mx6q_mmcd_calib, &mem_ddr_128x16);
	}
}

static void do_hang_error(void)
{

	for(;;) {
		udelay(1000000);
	}

}

static struct mx6_mmdc_calibration calib;

void board_init_f(ulong dummy)
{
	int err;
	int pmic_bus = 3;
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();
#ifdef CONFIG_SPL_WATCHDOG_SUPPORT
	void hw_watchdog_init(void);
	hw_watchdog_init();
#endif
	/* iomux and setup of i2c */
	setup_pins();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();
	int version = get_version();
	printf("SPL started on version %d\n", version);
	if (version >= 1 || is_mx6dq())
		pmic_bus = 1;

	err = pmic_setup(pmic_bus);
	if (err == 0) {
		pmic_set(pmic_bus, SW1AB, 1425);
		pmic_set(pmic_bus, SW1C, 1425);
		pmic_set(pmic_bus, SW3AB, 1350);
		udelay(10000);
	}
	/* DDR initialization */
	printf("Memory width %d\n", is_mx6dl() ? 32 : 64);

	spl_dram_init();
#ifdef DYNAMIC_CALIB
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

	mmdc_read_calibration(&sysinfo, &calib);
	printf("Calibration results for PHY0\n");
	printf("mpwldectrl0 = %08x\n", calib.p0_mpwldectrl0);
	printf("mpwldectrl1 = %08x\n", calib.p0_mpwldectrl1);
	printf("mpdgctrl0   = %08x\n", calib.p0_mpdgctrl0);
	printf("mpdgctrl1   = %08x\n", calib.p0_mpdgctrl1);
	printf("mprddlctl   = %08x\n", calib.p0_mprddlctl);
	printf("mpwrdlctl   = %08x\n", calib.p0_mpwrdlctl);
	if (sysinfo.dsize == 2) {
		printf("\nCalibration results for PHY1\n");
		printf("mpwldectrl0 = %08x\n", calib.p1_mpwldectrl0);
		printf("mpwldectrl1 = %08x\n", calib.p1_mpwldectrl1);
		printf("mpdgctrl0   = %08x\n", calib.p1_mpdgctrl0);
		printf("mpdgctrl1   = %08x\n", calib.p1_mpdgctrl1);
		printf("mprddlctl   = %08x\n", calib.p1_mprddlctl);
		printf("mpwrdlctl   = %08x\n", calib.p1_mpwrdlctl);
	}
#endif

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}


#endif
