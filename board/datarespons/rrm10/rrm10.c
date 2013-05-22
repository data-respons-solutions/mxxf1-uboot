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
#include <asm/arch/mx6q_pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |               \
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define SLOWOUT_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define REGINP_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_DISABLE  | PAD_CTL_HYS)

#define SPI_OUT_PAD_CTRL (PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define SPI_IN_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_HIGH |               \
	PAD_CTL_DSE_DISABLE   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define GPIO_AUX_5V IMX_GPIO_NR(6, 10)
#define GPIO_PCIE_RST_N IMX_GPIO_NR(7, 12)
#define GPIO_EEPROM_WP	IMX_GPIO_NR(1, 5)
#define GPIO_SPI_NOR_WP	IMX_GPIO_NR(6, 11)
#define GPIO_LAN2_EE_WP	IMX_GPIO_NR(6, 14)
#define GPIO_CAP_TOUCH_RST	IMX_GPIO_NR(6, 16)
#define GPIO_CAP_TOUCH_PWR	IMX_GPIO_NR(1, 4)

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
	MX6_PAD_NANDF_CS2__GPIO_6_15		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* LCD_EN		*/
	MX6_PAD_NANDF_CS3__GPIO_6_16		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* CAP_TCH_RST	*/

	MX6_PAD_NANDF_ALE__GPIO_6_8			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* CAP_TCH_INT	*/
	MX6_PAD_NANDF_CLE__GPIO_6_7			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USBP3_OC		*/
	MX6_PAD_NANDF_WP_B__GPIO_6_9		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* USBP3_EN		*/
	MX6_PAD_NANDF_RB0__GPIO_6_10		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* AUX_5V_EN	*/

	MX6_PAD_NANDF_D2__GPIO_2_2			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USBP1_OC		*/
	MX6_PAD_NANDF_D3__GPIO_2_3			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* USBP1_EN		*/

	MX6_PAD_NANDF_D4__GPIO_2_4			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* WDOG_PWR_EN	*/
	MX6_PAD_NANDF_D5__GPIO_2_5			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* EMMC_RST#	*/
	MX6_PAD_NANDF_D6__GPIO_2_6			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USBP2_OC		*/
	MX6_PAD_NANDF_D7__GPIO_2_7			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* USBP2_EN		*/
};

iomux_v3_cfg_t const extra_nvcc_gpio_pads[] = {
	MX6_PAD_KEY_COL0__GPIO_4_6			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* DIMM-	*/
	MX6_PAD_KEY_ROW0__GPIO_4_7			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* DIMM+	*/
	MX6_PAD_KEY_COL1__GPIO_4_8			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* TEMP_OS	*/

	MX6_PAD_KEY_COL2__GPIO_4_10			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* CODEC_PWR_EN	*/
	MX6_PAD_KEY_ROW2__GPIO_4_11			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* HDMI_CEC_IN	*/
	MX6_PAD_KEY_COL3__I2C2_SCL			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* I2C2_SCL		*/
	MX6_PAD_KEY_ROW3__I2C2_SDA			| MUX_PAD_CTRL(PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm | PAD_CTL_ODE),	/* I2C2_SDA	*/

	MX6_PAD_KEY_COL4__CAN2_TXCAN		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* CAN2_TX	*/
	MX6_PAD_KEY_ROW4__CAN2_RXCAN		| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* CAN2_RX	*/

	MX6_PAD_GPIO_0__CCM_CLKO			| MUX_PAD_CTRL(PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_HIGH),	/* AUD_MCLK	*/
	MX6_PAD_GPIO_1__GPIO_1_1			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* WDOG_B		*/
	MX6_PAD_GPIO_2__GPIO_1_2			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* BLON_3V_H	*/
	MX6_PAD_GPIO_3__I2C3_SCL			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* I2C3_SCL		*/

	MX6_PAD_GPIO_4__GPIO_1_4			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* TCH_PWRON	*/
	MX6_PAD_GPIO_5__GPIO_1_5			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* EEPROM_WP	*/
	MX6_PAD_GPIO_6__I2C3_SDA			| MUX_PAD_CTRL(PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm | PAD_CTL_ODE),	/* I2C3_SDA	*/
	MX6_PAD_GPIO_7__CAN1_TXCAN			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* CAN1_TX	*/

	MX6_PAD_GPIO_8__CAN1_RXCAN			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* CAN1_RX		*/
	MX6_PAD_GPIO_9__PWM1_PWMO			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* BL_PWM		*/
	MX6_PAD_GPIO_17__GPIO_7_12			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* PCIE_RST	*/
	MX6_PAD_GPIO_18__GPIO_7_13			| MUX_PAD_CTRL(PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_100K_DOWN),	/* PMIC_INT_B	*/

};

iomux_v3_cfg_t const extra_early_pads[] = {
	MX6_PAD_ENET_RX_ER__GPIO_1_24		| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USB_OTG_ID	*/
	MX6_PAD_ENET_RXD0__GPIO_1_27		| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* UOK_B		*/
	MX6_PAD_ENET_TXD1__GPIO_1_29		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* USB_H1_PWR_EN	*/
	MX6_PAD_SD1_CMD__GPIO_1_18			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* FAN_EN		*/
	MX6_PAD_SD1_DAT1__GPIO_1_17			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* BUZ_INT_EN	*/
	MX6_PAD_SD1_DAT2__GPIO_1_19			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* BUZ_EXT_EN	*/

	MX6_PAD_CSI0_DATA_EN__GPIO_5_20		| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* PCIE_WAKE_B	*/
	MX6_PAD_CSI0_DAT8__I2C1_SDA			| MUX_PAD_CTRL(PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm | PAD_CTL_ODE),	/* I2C1_SDA	*/
	MX6_PAD_CSI0_DAT9__I2C1_SCL			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* I2C1_SCL	*/

	MX6_PAD_EIM_D21__GPIO_3_21			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USB_OTG_OC		*/
	MX6_PAD_EIM_D22__GPIO_3_22			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* USB_ORG_PWR_EN	*/
	MX6_PAD_EIM_D29__GPIO_3_29			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* PWR_BTN_SNS		*/
	MX6_PAD_EIM_D30__GPIO_3_30			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USB_H1_OC		*/
	MX6_PAD_EIM_D31__GPIO_3_31			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* USB_P0_OC		*/
	MX6_PAD_EIM_D28__GPIO_3_28			| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* USBP0_EN			*/

	MX6_PAD_EIM_D16__ECSPI1_SCLK		| MUX_PAD_CTRL(SPI_OUT_PAD_CTRL),	/* CSPI1_CLK		*/
	MX6_PAD_EIM_D17__ECSPI1_MISO		| MUX_PAD_CTRL(SPI_IN_PAD_CTRL),	/* CSPI1_MISO		*/
	MX6_PAD_EIM_D18__ECSPI1_MOSI		| MUX_PAD_CTRL(SPI_OUT_PAD_CTRL),	/* CSPI1_MOSI		*/
	MX6_PAD_EIM_D19__ECSPI1_SS1			| MUX_PAD_CTRL(SPI_OUT_PAD_CTRL),	/* CSPI1_CS0		*/

	MX6_PAD_EIM_D23__GPIO_3_23			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* LAN2_DEV_OFF#	*/
	MX6_PAD_EIM_D24__GPIO_3_24			| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* LAN2_SMB_ALRT#	*/

};

iomux_v3_cfg_t const audio_pads[] = {
	MX6_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC	| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* AUD3_TXC		*/
	MX6_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD	| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* AUD3_TXD		*/
	MX6_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS	| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),	/* AUD3_TXFS	*/
	MX6_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD	| MUX_PAD_CTRL(REGINP_PAD_CTRL),	/* AUD3_RXD		*/
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC			| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__ENET_RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__ENET_RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__ENET_RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__ENET_RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__ENET_RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__ENET_RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__ENET_RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__ENET_RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__ENET_RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__ENET_RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* AR8031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO_1_25		| MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__GPIO_1_28		| MUX_PAD_CTRL(ENET_PAD_CTRL), 	/* ENET_WOL_INT (low)	*/
	MX6_PAD_ENET_RXD1__GPIO_1_26		| MUX_PAD_CTRL(ENET_PAD_CTRL), 	/* RGMII_INT (low)	*/
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
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
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC2_BASE_ADDR},
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
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return 0;
	       }

	       if (fsl_esdhc_initialize(bis, &usdhc_cfg[i]))
			printf("Warning: failed to initialize mmc dev %d\n", i);
	}

	return 0;
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

int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_enet();

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return 0;
}

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(extra_early_pads, ARRAY_SIZE(extra_early_pads));
	imx_iomux_v3_setup_multiple_pads(extra_nandf_pads, ARRAY_SIZE(extra_nandf_pads));
	imx_iomux_v3_setup_multiple_pads(extra_nvcc_gpio_pads, ARRAY_SIZE(extra_nvcc_gpio_pads));
	setup_iomux_uart();
	/* Bring up basic power for serial debug etc	*/
	gpio_direction_output(GPIO_AUX_5V, 1);
	gpio_direction_output(GPIO_PCIE_RST_N, 0);
	gpio_direction_output(GPIO_EEPROM_WP, 0);
	gpio_direction_output(GPIO_LAN2_EE_WP, 0);
	gpio_direction_output(GPIO_SPI_NOR_WP, 0);
	gpio_direction_output(GPIO_CAP_TOUCH_PWR, 1);
	gpio_direction_output(GPIO_CAP_TOUCH_RST, 0);
	//udelay(1000);

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gpio_set_value(GPIO_PCIE_RST_N, 1);
	gpio_set_value(GPIO_CAP_TOUCH_RST, 1);
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

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

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: MX6Q-RRM10\n");

	return 0;
}
