/*
 * (C) Copyright 2006
 * Heiko Schocher, DENX Software Engineering, hs@denx.de
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

/*
 * Altera FPGA configuration support
 */

#include <asm/errno.h>
#include <common.h>
#include <altera.h>
#include <ACEX1K.h>
#include <command.h>
#include <spi.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/gpio.h>
#include <fpga.h>

#include "ocas-pins.h"

DECLARE_GLOBAL_DATA_PTR;

#define CONFIG_FPGA_COUNT 1


/* Plattforminitializations */
/* Here we have to set the FPGA Chain */
/* PROGRAM_PROG_EN	= HIGH */
/* PROGRAM_SEL_DPR	= LOW */

/*int gpio_set_value(unsigned gpio, int value);
int gpio_get_value(unsigned gpio);
int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);*/


static int fpga_pre_fn(int cookie)
{
    /* FPGA Configuration */
    printf("%s: FPGA_STATUSn %d\n", __func__, gpio_get_value(GPIO_FPGA_STATUSn));
    printf("%s: FPGA_DONE %d\n", __func__, gpio_get_value(GPIO_FPGA_DONE));
	return FPGA_SUCCESS;
}

/* Set the state of CONFIG Pin */
static int fpga_config_fn(int assert_config, int flush, int cookie)
{

	if (assert_config) {
		gpio_set_value(GPIO_FPGA_CONFIGn, 0);
	} else {
		gpio_set_value(GPIO_FPGA_CONFIGn, 1);
	}

	return FPGA_SUCCESS;
}

/* Returns the state of STATUS Pin */
static int fpga_status_fn(int cookie)
{
	if (!gpio_get_value(GPIO_FPGA_STATUSn)) {
		printf("STATUS = LOW\n");
		return 0;
	}
	printf("STATUS = HIGH\n");
	return 1;
}

/* Returns the state of CONF_DONE Pin */
static int fpga_done_fn(int cookie)
{
	return gpio_get_value(GPIO_FPGA_DONE);

}

static int fpga_write_bitbang(const void *buf, size_t len, int flush, int cookie)
{
	int ret, n;
	const uint8_t *ptr = buf;
	uint8_t dat;
	int clk = 0;

	gpio_direction_output(IMX_GPIO_NR(5, 13), 0);
	gpio_direction_output(IMX_GPIO_NR(5, 10), 0);

	for (ret = 0; ret < len; ret++) {
		dat = ptr[ret];
		for (n=0; n < 8; n++) {
			gpio_set_value(IMX_GPIO_NR(5, 10), (dat & (1 << n)) ? 1 : 0);
			clk = clk ? 0 : 1;
			gpio_set_value(IMX_GPIO_NR(5, 13), clk);
		}
	}
	gpio_set_value(IMX_GPIO_NR(5, 13), 1);
	gpio_set_value(IMX_GPIO_NR(5, 10), 1);
	return 0;

}
/* writes the complete buffer to the FPGA
   writing the complete buffer in one function is much faster,
   then calling it for every bit */
static int fpga_write_fn(const void *buf, size_t len, int flush, int cookie)
{
#define SPI2_BUS 1
#define SPI2_CS0_FPGA_CONFIG 0
#define SPI2_BUS_FREQ	20000000
#define BIT_LENGTH	8
	const uint8_t *ptr = buf;
	struct spi_slave * spi_slave;
	size_t bytecount = 0;
	int ret = 0;
	unsigned long flags = 0;

	/* Slave setup */
	spi_slave = spi_setup_slave(SPI2_BUS, SPI2_CS0_FPGA_CONFIG | (IMX_GPIO_NR(5,12) << 8),
				SPI2_BUS_FREQ, SPI_MODE_0 | SPI_LSB_FIRST);

	if (spi_slave == NULL) {
		printf("%s: spi_setup_slave returns NULL\n", __func__);
		return -EINVAL;
	}
	printf("OCAS FPGA write ... buffer at 0x%08x, size = %d\n", (u32)ptr, len);
	spi_claim_bus(spi_slave);

	udelay(100);

	/* Transfer data */
	while (bytecount < len) {
		ret = spi_xfer(spi_slave, BIT_LENGTH*4, (ptr+bytecount), NULL, flags);
		if (ret != 0) {
			printf("ERROR: Spi xfer failed, at byte cnt: %d!, err = %d\n", bytecount, ret);
			break;
		}
		bytecount += 4;
		if (bytecount % 0x10000 == 0)
			printf(".");
	}

	spi_release_bus(spi_slave);
	spi_free_slave(spi_slave);

	return ret == 0 ? FPGA_SUCCESS : FPGA_FAIL;
}

/* called, when programming is aborted */
static int fpga_abort_fn(int cookie)
{
	return FPGA_SUCCESS;
}

/* called, when programming was succesful */
static int fpga_post_fn(int cookie)
{
	return fpga_abort_fn(cookie);
}

/* Note that these are pointers to code that is in Flash.  They will be
 * relocated at runtime.
 */
Altera_CYC2_Passive_Serial_fns fpga_fns = {
	fpga_pre_fn,
	fpga_config_fn,
	fpga_status_fn,
	fpga_done_fn,
	fpga_write_bitbang,
	fpga_abort_fn,
	fpga_post_fn
};

Altera_desc fpga[CONFIG_FPGA_COUNT] = {
	{Altera_CYC2,
	 passive_serial,
	 0x6b217c,
	 (void *) &fpga_fns,
	 NULL,
	 0}
};

/*
 * Initialize the fpga.  Return 1 on success, 0 on failure.
 */
int ocas_fpga_init(void)
{
	int i;

	printf("%s:%d: Initialize FPGA interface\n", __func__, __LINE__);
	fpga_init();

	for (i = 0; i < CONFIG_FPGA_COUNT; i++) {
		printf("%s:%d: Adding fpga %d\n", __func__, __LINE__, i);
		fpga_add(fpga_altera, &fpga[i]);
	}
	return 1;
}

