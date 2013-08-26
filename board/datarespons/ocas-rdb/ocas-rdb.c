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
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6q_pins.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <i2c.h>
#include <ipu_pixfmt.h>

#include <config.h>

// GPIO
#define GPIO_NET_12V_EN		IMX_GPIO_NR(1, 7)
#define GPIO_NET_3V35_EN	IMX_GPIO_NR(1, 18)
#define GPIO_LAN_3V3_EN		IMX_GPIO_NR(1, 20)

#define GPIO_LAN_RESETn			IMX_GPIO_NR(3, 23)
#define GPIO_LAN_INTRn			IMX_GPIO_NR(3, 29)
#define GPIO_LAN_LINK_STATUS1	IMX_GPIO_NR(3, 30)
#define GPIO_LAN_LINK_STATUS2	IMX_GPIO_NR(3, 31)

#define GPIO_FPGA_12V_EN		IMX_GPIO_NR(4, 8)
#define GPIO_FPGA_3V3_EN		IMX_GPIO_NR(4, 9)
#define GPIO_FPGA_2V5_EN		IMX_GPIO_NR(4, 10)
#define GPIO_FPGA_1V1_EN		IMX_GPIO_NR(4, 11)

#define GPIO_ADM_BUS_VMB_TX_EN	IMX_GPIO_NR(5, 5)
#define GPIO_ADM_BUS_VMB_RX_EN	IMX_GPIO_NR(5, 6)
#define GPIO_ADM_BUS_PMB_TX_EN	IMX_GPIO_NR(5, 7)
#define GPIO_ADM_BUS_PMB_RX_EN	IMX_GPIO_NR(5, 8)
#define GPIO_FPGA_DONE			IMX_GPIO_NR(5, 14)
#define GPIO_FPGA_STATUSn		IMX_GPIO_NR(5, 15)
#define GPIO_FPGA_CONFIGn		IMX_GPIO_NR(5, 16)
#define GPIO_MAIN_3V35_PGOOD	IMX_GPIO_NR(5, 17)
#define GPIO_PCIE_RESETn		IMX_GPIO_NR(5, 21)
#define GPIO_SATA_DA_DSS		IMX_GPIO_NR(5, 22)
#define GPIO_SATA_DP			IMX_GPIO_NR(5, 23)

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL_UP  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define ENET_PAD_CTRL_DN  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define SLOWOUT_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define SLOWIN_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define REGINP_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW   |		\
	PAD_CTL_DSE_DISABLE  | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define MX6_PAD_GPIO_16__RMII_REF_CLK  IOMUX_PAD(0x0618, 0x0248, IOMUX_CONFIG_SION | 2, 0x083C, 1, 0)

struct i2c_pads_info i2c_pad_info0 =
{
	.scl =
	{
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gpio_mode = MX6_PAD_CSI0_DAT9__GPIO_5_27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gp = IMX_GPIO_NR(5, 27)
	},
	.sda =
	{
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO_5_26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 26)
	}
};

struct i2c_pads_info i2c_pad_info1 =
{ .scl =
{ .i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gpio_mode = MX6_PAD_KEY_COL3__GPIO_4_12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gp =
        IMX_GPIO_NR(4, 12) }, .sda =
{ .i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gpio_mode = MX6_PAD_KEY_ROW3__GPIO_4_13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gp = IMX_GPIO_NR(4, 13) } };

/* I2C3, J15 - RGB connector */
struct i2c_pads_info i2c_pad_info2 =
{ .scl =
{ .i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gpio_mode = MX6_PAD_GPIO_3__GPIO_1_3 | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gp = IMX_GPIO_NR(1, 3) }, .sda =
{ .i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gpio_mode = MX6_PAD_GPIO_6__GPIO_1_6 | MUX_PAD_CTRL(I2C_PAD_CTRL),
        .gp = IMX_GPIO_NR(1, 6) } };

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *) PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

iomux_v3_cfg_t const uart1_pads[] =
{
	MX6_PAD_CSI0_DAT10__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_CSI0_DAT11__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] =
{
	MX6_PAD_ENET_TXD0__ENET_TDATA_0 | MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_ENET_TXD1__ENET_TDATA_1 | MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_ENET_RXD0__ENET_RDATA_0 | MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_ENET_RXD1__ENET_RDATA_1 | MUX_PAD_CTRL(ENET_PAD_CTRL_DN),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN  | MUX_PAD_CTRL(ENET_PAD_CTRL_DN), /* ENET_WOL_INT (up)	*/
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN  | MUX_PAD_CTRL(ENET_PAD_CTRL_UP),
	MX6_PAD_GPIO_16__RMII_REF_CLK   | MUX_PAD_CTRL(ENET_PAD_CTRL_UP),
};

iomux_v3_cfg_t const ecspi1_pads[] =
{
/* SPI NOR Flash */
	MX6_PAD_EIM_D19__GPIO_3_19 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi2_pads[] =
{
/* FPGA Configuration */
	MX6_PAD_DISP0_DAT18__ECSPI2_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__ECSPI2_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT17__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT16__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT19__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi3_pads[] =
{
/* Voltage Monitor */
	MX6_PAD_DISP0_DAT3__ECSPI3_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__ECSPI3_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__ECSPI3_SS2 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT0__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi4_pads[] =
{
/* LAN_CRTL */
	MX6_PAD_EIM_D20__ECSPI4_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D22__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D28__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D21__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi5_pads[] =
{
/* LO Clock Divider */
	MX6_PAD_SD2_DAT1__ECSPI5_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_SD2_DAT0__ECSPI5_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_SD2_CMD__ECSPI5_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_SD2_CLK__ECSPI5_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const wdog_pads[] =
{
	MX6_PAD_GPIO_9__WDOG1_WDOG_B | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL),
};

iomux_v3_cfg_t const extra_gpio_pads[] =
{
	MX6_PAD_GPIO_7__GPIO_1_7 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* NET_12_EN	*/
	MX6_PAD_SD1_CMD__GPIO_1_18 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* NET_3V3_EN	*/
	MX6_PAD_SD1_CLK__GPIO_1_20 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* LAN_3V3_EN	*/

	MX6_PAD_EIM_D23__GPIO_3_23 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* LAN_RESETn	*/
	MX6_PAD_EIM_D29__GPIO_3_29 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* LAN_INTRn	*/
	MX6_PAD_EIM_D30__GPIO_3_30 | MUX_PAD_CTRL(SLOWIN_PAD_CTRL), /* LAN_LINK_STATUS1		*/
	MX6_PAD_EIM_D31__GPIO_3_31 | MUX_PAD_CTRL(SLOWIN_PAD_CTRL), /* LAN_LINK_STATUS2		*/

	MX6_PAD_KEY_COL1__ENET_MDIO | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* FPGA_12V_EN	*/
	MX6_PAD_KEY_ROW1__GPIO_4_9 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* FPGA_1V1_EN	*/
	MX6_PAD_KEY_COL2__ENET_MDC | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* FPGA_2V5_EN	*/
	MX6_PAD_KEY_ROW2__GPIO_4_11 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* FPGA_3V3_EN	*/

	MX6_PAD_DISP0_DAT11__GPIO_5_5 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* ADM_BUS_VMB_TX_EN	*/
	MX6_PAD_DISP0_DAT12__GPIO_5_6 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* ADM_BUS_VMB_RX_EN	*/
	MX6_PAD_DISP0_DAT13__GPIO_5_7 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* ADM_BUS_PMB_TX_EN	*/
	MX6_PAD_DISP0_DAT14__GPIO_5_8 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* ADM_BUS_PMB_RX_EN	*/
	MX6_PAD_DISP0_DAT20__GPIO_5_14 | MUX_PAD_CTRL(SLOWIN_PAD_CTRL), /* FPGA DONE	*/
	MX6_PAD_DISP0_DAT21__GPIO_5_15 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* FPGA_STATUSn	*/
	MX6_PAD_DISP0_DAT22__GPIO_5_16 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* FPGA_CONFIGn	*/
	MX6_PAD_DISP0_DAT23__GPIO_5_17 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* MAIN_3V35_PGOOD		*/
	MX6_PAD_CSI0_VSYNC__GPIO_5_21 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* PCIE_RESETn	*/
	MX6_PAD_CSI0_DAT4__GPIO_5_22 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* SATA_DA/DSS	*/
	MX6_PAD_CSI0_DAT5__GPIO_5_23 | MUX_PAD_CTRL(SLOWOUT_PAD_CTRL), /* SATA_DP		*/
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_spi(void)
{
	// NOR Flash - boot image
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	// FPGA configuration
	imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
	// Voltage Monitor
	imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
	// LAN Control
	imx_iomux_v3_setup_multiple_pads(ecspi4_pads, ARRAY_SIZE(ecspi4_pads));
	// Clock Divider & Fanout (AD9512BCPZ)
	imx_iomux_v3_setup_multiple_pads(ecspi5_pads, ARRAY_SIZE(ecspi5_pads));
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
int fec_probe_nophy(bd_t *bd, int dev_id, uint32_t base_addr);

int board_eth_init(bd_t *bis)
{
	int ret;
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	struct eth_device *fec;
	uchar mac_addr[6];

	/* Use external RMII clock */
	struct iomuxc_base_regs * const iomuxc_regs =
		        (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	clrsetbits_le32(&iomuxc_regs->gpr[1], 1 << 21, 0);
    /* LAN GPIO pins*/
    gpio_direction_output(GPIO_LAN_RESETn, 0);
    gpio_direction_input(GPIO_LAN_INTRn);
    gpio_direction_input(GPIO_LAN_LINK_STATUS1);
    gpio_direction_input(GPIO_LAN_LINK_STATUS2);
    udelay(1000);
    gpio_set_value(GPIO_LAN_RESETn, 1);
    udelay(10000);

	/*
    ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);
	*/

	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, 0x0E , PHY_INTERFACE_MODE_RMII);
	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("using phy at %d\n", phydev->addr);

	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
		return ret;
	}

	fec = eth_get_dev_by_name("FEC");
	if (!fec) {
		printf("%s: Could not get FEC\n", __func__);
		return -EINVAL;
	}

	ret = eth_getenv_enetaddr("ethaddr", mac_addr);


	if (!ret) {
		printf("%s: No environment MAC for FEC\n", __func__);
		return -EINVAL;
	}

	printf("%s: Using MAC %pM for FEC, Link1 %d, Link2 %d\n", __func__, mac_addr,
			gpio_get_value(GPIO_LAN_LINK_STATUS1),
			gpio_get_value(GPIO_LAN_LINK_STATUS2));
	memcpy(fec->enetaddr, mac_addr, 6);
	ret = fec->write_hwaddr(fec);
	return ret;




	return ret;
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
	imx_iomux_v3_setup_multiple_pads(extra_gpio_pads,
	        ARRAY_SIZE(extra_gpio_pads));
	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	setup_iomux_uart();

	/* Bring up basic power for serial debug etc	*/
	gpio_direction_output(GPIO_NET_12V_EN, 1);
	gpio_direction_output(GPIO_NET_3V35_EN, 1);
	gpio_direction_output(GPIO_LAN_3V3_EN, 1);
	gpio_direction_output(GPIO_FPGA_12V_EN, 1);
	gpio_direction_output(GPIO_FPGA_3V3_EN, 1);
	gpio_direction_output(GPIO_FPGA_2V5_EN, 1);
	gpio_direction_output(GPIO_FPGA_1V1_EN, 1);
	gpio_direction_input(GPIO_MAIN_3V35_PGOOD);

	/* Administration Bus */
	gpio_direction_output(GPIO_ADM_BUS_VMB_TX_EN, 1);
	gpio_direction_output(GPIO_ADM_BUS_VMB_RX_EN, 1);
	gpio_direction_output(GPIO_ADM_BUS_PMB_TX_EN, 1);
	gpio_direction_output(GPIO_ADM_BUS_PMB_RX_EN, 1);

	gpio_direction_output(GPIO_PCIE_RESETn, 1);

	return 0;
}

#ifdef CONFIG_CMD_SATA
int setup_sata(void)
{
	/* SATA */
	gpio_direction_input(GPIO_SATA_DA_DSS);
	gpio_direction_input(GPIO_SATA_DP);

	struct iomuxc_base_regs * const iomuxc_regs =
	        (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;

	int ret = enable_sata_clock();
	if (ret)
		return ret;

	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
	        IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
	        | IOMUXC_GPR13_SATA_PHY_7_SATA2M
			| IOMUXC_GPR13_SATA_SPEED_3G
			| (3 << IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			| IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			| IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			| IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			| IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			| IOMUXC_GPR13_SATA_PHY_1_SLOW);

	return 0;
}
#endif

int board_init(void)
{
	int ret;
	/* address of boot parameters */
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
	// SATA - Linux image
	ret = setup_sata();
	if (ret)
	{
		printf("setup_sata failed with %d\n", ret);
		return ret;
	}
#endif

	printf("%s: OK\n", __func__);
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] =
{
/* 8 bit bus width */
{ "sd3", MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00) },
{ NULL, 0 }, };
#endif

int board_late_init(void)
{
	//setenv("stdout", "serial");
	//setenv("stdin", "serial");
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: OCAS iMX6DQ-CRB\n");

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)

int overwrite_console(void)
{
	return 1;
}

struct display_info_t
{
	int bus;
	int addr;
	int pixfmt;
	int (*detect)(struct display_info_t const *dev);
	void (*enable)(struct display_info_t const *dev);
	struct fb_videomode mode;
};

static void enable_lvds(void)
{
	struct iomuxc *iomux = (struct iomuxc *)
	IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);
	gpio_direction_output(GPIO_LCD_EN, 1);
	gpio_direction_output(GPIO_BL_EN, 1);
	gpio_direction_output(GPIO_BL_PWM, 1);
}

static struct fb_videomode disp =
{
	.name = "RRM10-XGA",
	.refresh = 60,
	.xres = 1024,
	.yres = 768,
	.pixclock = 15385,
	.left_margin = 220,
	.right_margin = 40,
	.upper_margin = 21,
	.lower_margin = 7,
	.hsync_len = 60,
	.vsync_len = 10,
	.sync = FB_SYNC_EXT,
	.vmode = FB_VMODE_NONINTERLACED
};

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg;

	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |= MXC_CCM_CCGR3_IPU1_IPU_DI0_OFFSET
	|MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set PFD1_FRAC to 0x13 == 455 MHz (480*18)/0x13 */
	writel(ANATOP_PFD_480_PFD1_FRAC_MASK, &anatop->pfd_480_clr);
	writel(0x13<<ANATOP_PFD_480_PFD1_FRAC_SHIFT, &anatop->pfd_480_set);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
			|MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	|(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK
			|MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK
			|MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
			<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET)
	|(CHSCCDR_PODF_DIVIDE_BY_3
			<<MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET)
	|(CHSCCDR_IPU_PRE_CLK_540M_PFD
			<<MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	|IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	|IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	|IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	|IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	|IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	|IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	|IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	|IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
	| (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
			<<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
	ipuv3_fb_init(&disp, 0, IPU_PIX_FMT_LVDS666);
	enable_lvds();
	/* backlights off until needed */
}
#endif

