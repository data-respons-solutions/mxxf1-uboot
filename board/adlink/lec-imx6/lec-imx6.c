/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014 LiPPERT ADLINK Technology GmbH
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
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#if CONFIG_I2C_MXC
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#endif
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
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

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define EPDC_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#if CONFIG_I2C_MXC
#define I2C_PC MUX_PAD_CTRL(I2C_PAD_CTRL)

static const struct i2c_pads_info i2c_pad_info1 = { /* PCIe switch, SMARC I2C_CAM */
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | I2C_PC,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO_5_27 | I2C_PC,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | I2C_PC,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO_5_26 | I2C_PC,
		.gp = IMX_GPIO_NR(5, 26)
	}
};

static const struct i2c_pads_info i2c_pad_info2 = { /* PFUZE100 PMIC, SMARC I2C_GP, I2C_LCD */
	.scl = {
		.i2c_mode = MX6_PAD_EIM_EB2__I2C2_SCL | I2C_PC,
		.gpio_mode = MX6_PAD_EIM_EB2__GPIO_2_30 | I2C_PC,
		.gp = IMX_GPIO_NR(2, 30)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D16__I2C2_SDA | I2C_PC,
		.gpio_mode = MX6_PAD_EIM_D16__GPIO_3_16 | I2C_PC,
		.gp = IMX_GPIO_NR(3, 16)
	}
};

static const struct i2c_pads_info i2c_pad_info3 = { /* BMC, GPIO expander, SMARC I2C_PM */
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL | I2C_PC,
		.gpio_mode = MX6_PAD_EIM_D17__GPIO_3_17 | I2C_PC,
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | I2C_PC,
		.gpio_mode = MX6_PAD_EIM_D18__GPIO_3_18 | I2C_PC,
		.gp = IMX_GPIO_NR(3, 18)
	}
};
#endif

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

static const iomux_v3_cfg_t uart1_pads[] = {	/* SMARC SER0 */
	MX6_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D20__UART1_RTS  | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D19__UART1_CTS  | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static const iomux_v3_cfg_t uart2_pads[] = {	/* SMARC SER1 */
	MX6_PAD_EIM_D26__UART2_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D27__UART2_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static const iomux_v3_cfg_t uart4_pads[] = {	/* SMARC SER3 */
	MX6_PAD_KEY_COL0__UART4_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW0__UART4_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static const iomux_v3_cfg_t uart5_pads[] = {	/* SMARC SER2 */
	MX6_PAD_KEY_COL1__UART5_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW1__UART5_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_COL4__UART5_RTS | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW4__UART5_CTS | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static const iomux_v3_cfg_t enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
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
	/* AR8035 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO_1_25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset AR8035 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
}

#ifdef CONFIG_SYS_USE_SPINOR
#if !CONFIG_I2S_AUDIO
static const iomux_v3_cfg_t ecspi1_pads[] = {	/* SMARC SPI0 (or I2S0) */
	MX6_PAD_CSI0_DAT4__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
//	MX6_PAD_CSI0_DAT7__ECSPI1_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL), /* SMARC CS0 */
	MX6_PAD_CSI0_DAT7__GPIO_5_25  | MUX_PAD_CTRL(NO_PAD_CTRL),  //"sf probe 0:39168"
//	MX6_PAD_EIM_D24__ECSPI1_SS2   | MUX_PAD_CTRL(SPI_PAD_CTRL), /* SMARC CS1 */
	MX6_PAD_EIM_D24__GPIO_3_24    | MUX_PAD_CTRL(NO_PAD_CTRL),  //"sf probe 0:22530"
};
#endif

static const iomux_v3_cfg_t ecspi2_pads[] = {	/* SMARC SPI1 */
	MX6_PAD_EIM_CS0__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_CS1__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_OE__ECSPI2_MISO  | MUX_PAD_CTRL(SPI_PAD_CTRL),
//	MX6_PAD_EIM_RW__ECSPI2_SS0  | MUX_PAD_CTRL(SPI_PAD_CTRL), /* SMARC CS1 */
	MX6_PAD_EIM_RW__GPIO_2_26   | MUX_PAD_CTRL(NO_PAD_CTRL),  //"sf probe 1:14848"
//	MX6_PAD_EIM_LBA__ECSPI2_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL), /* SMARC CS0 */
	MX6_PAD_EIM_LBA__GPIO_2_27  | MUX_PAD_CTRL(NO_PAD_CTRL),  //"sf probe 1:15105"
};

static const iomux_v3_cfg_t ecspi4_pads[] = {	/* Onboard U-Boot SPI */
	MX6_PAD_EIM_D21__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D28__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D22__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
//	MX6_PAD_EIM_D25__ECSPI4_SS3 | MUX_PAD_CTRL(SPI_PAD_CTRL), /* Onboard U-Boot */
	MX6_PAD_EIM_D25__GPIO_3_25  | MUX_PAD_CTRL(NO_PAD_CTRL),  //"sf probe 3:22787"
};

static void setup_spinor(void)
{
#if !CONFIG_I2S_AUDIO
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	gpio_direction_output(IMX_GPIO_NR(5, 25), 1); //allows CS across multiple xfers, see mxc_spi.c:decode_cs()
	gpio_direction_output(IMX_GPIO_NR(3, 24), 1);
#endif
	imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
	gpio_direction_output(IMX_GPIO_NR(2, 26), 1);
	gpio_direction_output(IMX_GPIO_NR(2, 27), 1);
	imx_iomux_v3_setup_multiple_pads(ecspi4_pads, ARRAY_SIZE(ecspi4_pads));
	gpio_direction_output(IMX_GPIO_NR(3, 25), 1);
}
#endif

static const iomux_v3_cfg_t usdhc1_pads[] = {	/* onboard eMMC */
	MX6_PAD_SD1_CLK__USDHC1_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__USDHC1_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__USDHC1_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__USDHC1_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__USDHC1_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D0__USDHC1_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D1__USDHC1_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D2__USDHC1_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D3__USDHC1_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static const iomux_v3_cfg_t usdhc2_pads[] = {	/* SMARC SDIO */
	MX6_PAD_SD2_CLK__USDHC2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__USDHC2_DAT0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__USDHC2_DAT1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__USDHC2_DAT2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__USDHC2_DAT3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_4__USDHC2_CD	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_KEY_ROW2__GPIO_4_11	| MUX_PAD_CTRL(NO_PAD_CTRL), /* SMARC SDIO_PWR_EN */
};

static const iomux_v3_cfg_t usdhc4_pads[] = {	/* SMARC SDMMC */
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
	MX6_PAD_NANDF_ALE__USDHC4_RST | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
static iomux_v3_cfg_t const epdc_enable_pads[] = {
	MX6_PAD_EIM_A16__EPDC_SDDO_0	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA10__EPDC_SDDO_1	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA12__EPDC_SDDO_2	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA11__EPDC_SDDO_3	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_LBA__EPDC_SDDO_4	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_EB2__EPDC_SDDO_5	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_CS0__EPDC_SDDO_6	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_RW__EPDC_SDDO_7	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_A21__EPDC_GDCLK	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_A22__EPDC_GDSP	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_A23__EPDC_GDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_A24__EPDC_GDRL	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_D31__EPDC_SDCLK	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_D27__EPDC_SDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA1__EPDC_SDLE	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_EB1__EPDC_SDSHR	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA2__EPDC_BDR_0	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA4__EPDC_SDCE_0	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA5__EPDC_SDCE_1	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
	MX6_PAD_EIM_DA6__EPDC_SDCE_2	| MUX_PAD_CTRL(EPDC_PAD_CTRL),
};

static iomux_v3_cfg_t const epdc_disable_pads[] = {
	MX6_PAD_EIM_A16__GPIO_2_22,
	MX6_PAD_EIM_DA10__GPIO_3_10,
	MX6_PAD_EIM_DA12__GPIO_3_12,
	MX6_PAD_EIM_DA11__GPIO_3_11,
	MX6_PAD_EIM_LBA__GPIO_2_27,
	MX6_PAD_EIM_EB2__GPIO_2_30,
	MX6_PAD_EIM_CS0__GPIO_2_23,
	MX6_PAD_EIM_RW__GPIO_2_26,
	MX6_PAD_EIM_A21__GPIO_2_17,
	MX6_PAD_EIM_A22__GPIO_2_16,
	MX6_PAD_EIM_A23__GPIO_6_6,
	MX6_PAD_EIM_A24__GPIO_5_4,
	MX6_PAD_EIM_D31__GPIO_3_31,
	MX6_PAD_EIM_D27__GPIO_3_27,
	MX6_PAD_EIM_DA1__GPIO_3_1,
	MX6_PAD_EIM_EB1__GPIO_2_29,
	MX6_PAD_EIM_DA2__GPIO_3_2,
	MX6_PAD_EIM_DA4__GPIO_3_4,
	MX6_PAD_EIM_DA5__GPIO_3_5,
	MX6_PAD_EIM_DA6__GPIO_3_6,
};
#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
	imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));
}

#ifdef CONFIG_I2C_MXC
static int setup_pmic_voltages(void)
{
	unsigned char value, rev_id = 0 ;

	i2c_set_bus_num(1);
	if (!i2c_probe(0x8)) {
		if (i2c_read(0x8, 0, 1, &value, 1)) {
			printf("Read device ID error!\n");
			return -1;
		}
		if (i2c_read(0x8, 3, 1, &rev_id, 1)) {
			printf("Read Rev ID error!\n");
			return -1;
		}
		printf("PMIC:  PFUZE100, device ID %x, revision %x\n", value, rev_id);

		/* set SW1AB staby volatage 0.975V*/
		if (i2c_read(0x8, 0x21, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0x3f;
		value |= 0x1b;
		if (i2c_write(0x8, 0x21, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}
		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		if (i2c_read(0x8, 0x24, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0xc0;
		value |= 0x40;
		if (i2c_write(0x8, 0x24, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}

		/* set SW1C staby volatage 0.975V*/
		if (i2c_read(0x8, 0x2f, 1, &value, 1)) {
			printf("Read SW1CSTBY error!\n");
			return -1;
		}
		value &= ~0x3f;
		value |= 0x1b;
		if (i2c_write(0x8, 0x2f, 1, &value, 1)) {
			printf("Set SW1CSTBY error!\n");
			return -1;
		}

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		if (i2c_read(0x8, 0x32, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0xc0;
		value |= 0x40;
		if (i2c_write(0x8, 0x32, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}
	}

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned char value;
	i2c_set_bus_num(1);
	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0;	/* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		/* increase VDDARM to 1.425V */
		if (i2c_read(0x8, 0x20, 1, &value, 1)) {
			printf("Read SW1AB error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x2d;
		if (i2c_write(0x8, 0x20, 1, &value, 1)) {
			printf("Set SW1AB error!\n");
			return;
		}
		/* increase VDDSOC to 1.425V */
		if (i2c_read(0x8, 0x2e, 1, &value, 1)) {
			printf("Read SW1C error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x2d;
		if (i2c_write(0x8, 0x2e, 1, &value, 1)) {
			printf("Set SW1C error!\n");
			return;
		}
	}
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		/* decrease VDDARM to 1.175V */
		if (i2c_read(0x8, 0x20, 1, &value, 1)) {
			printf("Read SW1AB error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x23;
		if (i2c_write(0x8, 0x20, 1, &value, 1)) {
			printf("Set SW1AB error!\n");
			return;
		}
		/* increase VDDSOC to 1.175V */
		if (i2c_read(0x8, 0x2e, 1, &value, 1)) {
			printf("Read SW1C error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x23;
		if (i2c_write(0x8, 0x2e, 1, &value, 1)) {
			printf("Set SW1C error!\n");
			return;
		}

		set_anatop_bypass();
		printf("Switched to LDO bypass mode\n");
	}
}
#endif
#endif


#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = {
	{USDHC1_BASE_ADDR},
	{USDHC2_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#ifdef CONFIG_ENV_IS_IN_MMC
int mmc_get_env_devno(void)
{
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	u32 dev_no;

	/* BOOT_CFG2[3] and BOOT_CFG2[4] */
	dev_no = (soc_sbmr & 0x00001800) >> 11;

	/* need ubstract 1 to map to the mmc device id
	 * see the comments in board_mmc_init function
	 */

	dev_no--;

	setenv_ulong("mmcdev", dev_no);

	return dev_no;
}
#endif

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = 1; /* onboard eMMC is always present */
		break;
	case USDHC2_BASE_ADDR:
//		ret = -1; /* use native controller function */
		ret = 1; /* level shifter is wrong :-( */
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* ext. SD/MMC is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i;
	printf("%s: called\n", __func__);
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    eMMC
	 * mmc1                    SD2
	 * mmc2                    SD4
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			gpio_direction_output(IMX_GPIO_NR(4, 11), 1); /* SMARC SDIO_PWR_EN */
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case 2:
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

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#ifdef CONFIG_SPLASH_SCREEN
extern int mmc_get_env_devno(void);
int setup_splash_img(void)
{
#ifdef CONFIG_SPLASH_IS_IN_MMC
	int mmc_dev = mmc_get_env_devno();
	ulong offset = CONFIG_SPLASH_IMG_OFFSET;
	ulong size = CONFIG_SPLASH_IMG_SIZE;
	ulong addr = 0;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	s = getenv("splashimage");

	if (NULL == s) {
		puts("env splashimage not found!\n");
		return -1;
	}
	addr = simple_strtoul(s, NULL, 16);

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
					blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#endif

	return 0;
}
#endif

vidinfo_t panel_info = {
	.vl_refresh = 85,
	.vl_col = 800,
	.vl_row = 600,
	.vl_pixclock = 26666667,
	.vl_left_margin = 8,
	.vl_right_margin = 100,
	.vl_upper_margin = 4,
	.vl_lower_margin = 8,
	.vl_hsync = 4,
	.vl_vsync = 1,
	.vl_sync = 0,
	.vl_mode = 0,
	.vl_flag = 0,
	.vl_bpix = 3,
	.cmap = 0,
};

struct epdc_timing_params panel_timings = {
	.vscan_holdoff = 4,
	.sdoed_width = 10,
	.sdoed_delay = 20,
	.sdoez_width = 10,
	.sdoez_delay = 20,
	.gdclk_hp_offs = 419,
	.gdsp_offs = 20,
	.gdoe_offs = 0,
	.gdclk_offs = 5,
	.num_ce = 1,
};

static void setup_epdc_power(void)
{
	/* Setup epdc voltage */

	/* EIM_A17 - GPIO2[21] for PWR_GOOD status */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_A17__GPIO_2_21 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as input */
	gpio_direction_input(IMX_GPIO_NR(2, 21));

	/* EIM_D17 - GPIO3[17] for VCOM control */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_D17__GPIO_3_17 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* Set as output */
	gpio_direction_output(IMX_GPIO_NR(3, 17), 1);

	/* EIM_D20 - GPIO3[20] for EPD PMIC WAKEUP */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_D20__GPIO_3_20 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as output */
	gpio_direction_output(IMX_GPIO_NR(3, 20), 1);

	/* EIM_A18 - GPIO2[20] for EPD PWR CTL0 */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_A18__GPIO_2_20 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));
	/* Set as output */
	gpio_direction_output(IMX_GPIO_NR(2, 20), 1);
}

int setup_waveform_file(void)
{
#ifdef CONFIG_WAVEFORM_FILE_IN_MMC
	int mmc_dev = mmc_get_env_devno();
	ulong offset = CONFIG_WAVEFORM_FILE_OFFSET;
	ulong size = CONFIG_WAVEFORM_FILE_SIZE;
	ulong addr = CONFIG_WAVEFORM_BUF_ADDR;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
				      blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#else
	return -1;
#endif
}

static void epdc_enable_pins(void)
{
	/* epdc iomux settings */
	imx_iomux_v3_setup_multiple_pads(epdc_enable_pads,
				ARRAY_SIZE(epdc_enable_pads));
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to GPIO */
	imx_iomux_v3_setup_multiple_pads(epdc_disable_pads,
				ARRAY_SIZE(epdc_disable_pads));
}

static void setup_epdc(void)
{
	unsigned int reg;

	/*** epdc Maxim PMIC settings ***/

	/* EPDC PWRSTAT - GPIO2[21] for PWR_GOOD status */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_A17__GPIO_2_21 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* EPDC VCOM0 - GPIO3[17] for VCOM control */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_D17__GPIO_3_17 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* UART4 TXD - GPIO3[20] for EPD PMIC WAKEUP */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_D20__GPIO_3_20 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/* EIM_A18 - GPIO2[20] for EPD PWR CTL0 */
	imx_iomux_v3_setup_pad(MX6_PAD_EIM_A18__GPIO_2_20 |
				MUX_PAD_CTRL(EPDC_PAD_CTRL));

	/*** Set pixel clock rates for EPDC ***/

	/* EPDC AXI clk (IPU2_CLK) from PFD_400M, set to 396/2 = 198MHz */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CSCDR3);
	reg &= ~0x7C000;
	reg |= (1 << 16) | (1 << 14);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CSCDR3);

	/* EPDC AXI clk enable */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR3);
	reg |= 0x00C0;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR3);

	/* EPDC PIX clk (IPU2_DI1_CLK) from PLL5, set to 650/4/6 = ~27MHz */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CSCDR2);
	reg &= ~0x3FE00;
	reg |= (2 << 15) | (5 << 12);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CSCDR2);

	/* PLL5 enable (defaults to 650) */
	reg = readl(ANATOP_BASE_ADDR + ANATOP_PLL_VIDEO);
	reg &= ~((1 << 16) | (1 << 12));
	reg |= (1 << 13);
	writel(reg, ANATOP_BASE_ADDR + ANATOP_PLL_VIDEO);

	/* EPDC PIX clk enable */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR3);
	reg |= 0x0C00;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR3);

	panel_info.epdc_data.working_buf_addr = CONFIG_WORKING_BUF_ADDR;
	panel_info.epdc_data.waveform_buf_addr = CONFIG_WAVEFORM_BUF_ADDR;

	panel_info.epdc_data.wv_modes.mode_init = 0;
	panel_info.epdc_data.wv_modes.mode_du = 1;
	panel_info.epdc_data.wv_modes.mode_gc4 = 3;
	panel_info.epdc_data.wv_modes.mode_gc8 = 2;
	panel_info.epdc_data.wv_modes.mode_gc16 = 2;
	panel_info.epdc_data.wv_modes.mode_gc32 = 2;

	panel_info.epdc_data.epdc_timings = panel_timings;

	setup_epdc_power();

	/* Assign fb_base */
	gd->fb_base = CONFIG_FB_BASE;
}

void epdc_power_on(void)
{
	unsigned int reg;

	/* Set EPD_PWR_CTL0 to high - enable EINK_VDD (3.15) */
	gpio_set_value(IMX_GPIO_NR(2, 20), 1);
	udelay(1000);

	/* Enable epdc signal pin */
	epdc_enable_pins();

	/* Set PMIC Wakeup to high - enable Display power */
	gpio_set_value(IMX_GPIO_NR(3, 20), 1);

	/* Wait for PWRGOOD == 1 */
	while (1) {
		reg = readl(GPIO2_BASE_ADDR + GPIO_PSR);
		if (!(reg & (1 << 21)))
			break;

		udelay(100);
	}

	/* Enable VCOM */
	gpio_set_value(IMX_GPIO_NR(3, 17), 1);

	udelay(500);
}

void epdc_power_off(void)
{
	/* Set PMIC Wakeup to low - disable Display power */
	gpio_set_value(IMX_GPIO_NR(3, 20), 0);

	/* Disable VCOM */
	gpio_set_value(IMX_GPIO_NR(3, 17), 0);

	epdc_disable_pins();

	/* Set EPD_PWR_CTL0 to low - disable EINK_VDD (3.15) */
	gpio_set_value(IMX_GPIO_NR(2, 20), 0);
}
#endif

#ifdef CONFIG_CMD_SATA
static int setup_sata(void)
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

static int mx6_rgmii_rework(struct phy_device *phydev)
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

#if defined(CONFIG_VIDEO_IPUV3)
struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

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

static struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= NULL,
	.mode	= {
		.name           = "Hannstar-XGA",
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
	.detect	= NULL,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
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
	int i;
	int ret;
	char const *panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
			printf("No panel detected: default to %s\n", panel);
			i = 0;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			if (displays[i].enable)
				displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

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
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
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
#ifdef CONFIG_I2C_MXC
	setup_iomux_uart();
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0, &i2c_pad_info2);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0, &i2c_pad_info3);
#endif
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

#ifdef CONFIG_SYS_USE_SPINOR
	setup_spinor();
#endif

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
	setup_epdc();
#endif

	return 0;
}

static void setup_baseboard(void)
{
	/* Init the NXP PCF8575 I2C GPIO expander (=device 20 on I2C1) on the
	 * ADLINK LEC-Base carrier board, otherwise the on-baseboard USB hubs
	 * won't work. */
	i2c_set_bus_num(0);
	static const uchar gpio_state[2] = {
		/* 0: HUB1_RESET	 8: 3G1_DISABLE#
		 * 1: CAN1_STB (NC)	 9: 3G1_DOWN#
		 * 2: CAN2_STB (NC)	10: 3G2_DISABLE#
		 * 3: LCD_RESET# (NC)	11: 3G2_DOWN#
		 * 4: SUP_RESET# (NC)	12: HDMI_RESET#
		 * 5: LVDS_VDD_EN	13: 3G_PW_EN
		 * 6: CAM_PW_EN (NC)	14: PCAM_LE#
		 * 7: GPS_VDD_EN	15: HUB2_RESET */
		   0xfe,		0x3f /* bitmask: 0=pulled low, 1=high/input */
	};
	if (i2c_write(0x20, 0, 0, gpio_state, sizeof(gpio_state)))
		printf("Error initializing ADLINK LEC-Base carrier board GPIOs!\n");
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"emmc", MAKE_CFGVAL(0x40, 0x20, 0x00, 0x00)},
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd4",	 MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
	int ret = 0;
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_I2C_MXC
	/* Init I2C buses. 1st param is host controller nr (0=I2C1) i.e. must
	 * match pads given. setup_i2c() knows all base addresses already. 3rd
	 * param "slave_addr" is actually never used.
	 * Later i2c_set_bus_num() selects current bus for r/w. Here nr depends
	 * on setup order i.e. 0=1st bus on which setup_i2c() was called. */

	ret = setup_pmic_voltages();

	setup_baseboard();
#endif

	return ret;
}

static const iomux_v3_cfg_t gpio_pads[] = {
	MX6_PAD_GPIO_3__GPIO_1_3      | MUX_PAD_CTRL(NO_PAD_CTRL), /* AEN# */
	MX6_PAD_GPIO_5__GPIO_1_5      | MUX_PAD_CTRL(NO_PAD_CTRL), /* SMARC FORCE_RECOV# */
	MX6_PAD_GPIO_8__GPIO_1_8      | MUX_PAD_CTRL(NO_PAD_CTRL), /* SMARC TEST# */
	MX6_PAD_ENET_TX_EN__GPIO_1_28 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CPU_BIOS_DEFAULT# */
};

static void setup_gpios(void)
{
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));
	/* Mux control: 0=SMARC SPI0, 1=I2S0 enabled; GPIO_3 defaults to input
	 * and AEN# has a PD, so power-up default is SPI0. */
	gpio_direction_output(IMX_GPIO_NR(1, 3), CONFIG_I2S_AUDIO);
	gpio_direction_input(IMX_GPIO_NR(1,  5));
	gpio_direction_input(IMX_GPIO_NR(1,  8));
	gpio_direction_input(IMX_GPIO_NR(1, 28));
	/* Override bootcmd var; change is temporary unless replacement itself
	 * contains actual "saveenv" command. In order of increasing priority. */
	if (!gpio_get_value(IMX_GPIO_NR(1,  8)))
		setenv("bootcmd", CONFIG_BOOTCMD_TEST);
	if (!gpio_get_value(IMX_GPIO_NR(1, 28)))
		setenv("bootcmd", CONFIG_BOOTCMD_CPU_BIOS_DEFAULT);
	if (!gpio_get_value(IMX_GPIO_NR(1,  5)))
		setenv("bootcmd", CONFIG_BOOTCMD_FORCE_RECOV);
}

/* Notify the BMC we're alive and kicking, or after a while it will effect a
 * power cycle and switch to the alternate SPI flash.
 * Should be done as late as possible. */
static void bmc_send_alive_signal(void)
{
	/* BMC (=device 28 on I2C3) command 25: "BIOS alive" */
	static const uchar i_am_alive[] = { 1/*sizeof(i_am_alive)-1*/, 0x03 };
	i2c_set_bus_num(2);
	if (i2c_write(0x28, 0x25, 1, i_am_alive, sizeof(i_am_alive)))
		printf("Error sending 'U-Boot alive' signal to BMC!\n");
}

int last_stage_init(void)
{
	setup_gpios();

#ifdef CONFIG_I2C_MXC
	bmc_send_alive_signal();
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: ADLINK LEC-iMX6\n");

	return 0;
}
