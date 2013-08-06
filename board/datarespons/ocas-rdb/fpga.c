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

#include <common.h>
#include <altera.h>
#include <ACEX1K.h>
#include <command.h>
#include <spi.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/gpio.h>
#include "fpga.h"

DECLARE_GLOBAL_DATA_PTR;

#define CONFIG_FPGA_COUNT 1

#define GPIO_FPGA_DONE			IMX_GPIO_NR(5, 14)
#define GPIO_FPGA_STATUSn		IMX_GPIO_NR(5, 15)
#define GPIO_FPGA_CONFIGn		IMX_GPIO_NR(5, 16)

/* Plattforminitializations */
/* Here we have to set the FPGA Chain */
/* PROGRAM_PROG_EN	= HIGH */
/* PROGRAM_SEL_DPR	= LOW */

/*int gpio_set_value(unsigned gpio, int value);
int gpio_get_value(unsigned gpio);
int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);*/


int fpga_pre_fn(int cookie)
{
    /* FPGA Configuration */
	/* First we set STATUS to 1 then as an input */
    gpio_set_value(GPIO_FPGA_STATUSn, 1);
    gpio_direction_input(GPIO_FPGA_STATUSn);

	/* output */
    gpio_direction_output(GPIO_FPGA_CONFIGn, 1);
    gpio_set_value(GPIO_FPGA_CONFIGn, 1);

   	/* input */
    gpio_direction_input(GPIO_FPGA_DONE);
    gpio_set_value(GPIO_FPGA_DONE, 0);

	return FPGA_SUCCESS;
}

/* Set the state of CONFIG Pin */
int fpga_config_fn(int assert_config, int flush, int cookie)
{
	if (assert_config) {
		gpio_set_value(GPIO_FPGA_CONFIGn, 0);
	} else {
		gpio_set_value(GPIO_FPGA_CONFIGn, 1);
	}
	return FPGA_SUCCESS;
}

/* Returns the state of STATUS Pin */
int fpga_status_fn(int cookie)
{
	if (!gpio_get_value(GPIO_FPGA_STATUSn)) {
		printf("STATUS = LOW\n");
		return FPGA_FAIL;
	}
	printf("STATUS = HIGH\n");
	return FPGA_SUCCESS;
}

/* Returns the state of CONF_DONE Pin */
int fpga_done_fn(int cookie)
{
	if (!gpio_get_value(GPIO_FPGA_DONE)) {
		printf("CONF_DON = LOW\n");
		return FPGA_FAIL;
	}
	printf("CONF_DON = HIGH\n");
	return FPGA_SUCCESS;
}


/* writes the complete buffer to the FPGA
   writing the complete buffer in one function is much faster,
   then calling it for every bit */
int fpga_write_fn(const void *buf, size_t len, int flush, int cookie)
{
#define SPI2_BUS 1
#define SPI2_CS0_FPGA_CONFIG 0
#define SPI2_BUS_FREQ	20000000
#define BIT_LENGTH	8
	// TODO: Verify that u-bout counts from 0 (SPI2 = bus 1)
	// TODO: Verify that FPGA support SPI_MODE_0
	struct spi_slave * spi_slave;
	size_t bytecount = 0;
	uint8_t *data = (uint8_t *) buf;
	uint8_t val = 0;
	uint8_t ret = 0;

	/* Slave setup */
	spi_slave = spi_setup_slave(SPI2_BUS, SPI2_CS0_FPGA_CONFIG,
				SPI2_BUS_FREQ, SPI_MODE_0);

	spi_claim_bus(spi_slave);

	/* Transfer data */
	while (bytecount < len) {
			val = data[bytecount];
			ret = spi_xfer(spi_slave, BIT_LENGTH, &val, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
			if (ret != 0) {
				printf("ERROR: Spi xfer failed, at byte cnt: %d!\n", bytecount);
				break;
			}
			bytecount++;
	}

	spi_release_bus(spi_slave);
	spi_free_slave(spi_slave);

	return ret == 0 ? FPGA_SUCCESS : FPGA_FAIL;
}

/* called, when programming is aborted */
int fpga_abort_fn(int cookie)
{
	return FPGA_SUCCESS;
}

/* called, when programming was succesful */
int fpga_post_fn(int cookie)
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
	fpga_write_fn,
	fpga_abort_fn,
	fpga_post_fn
};

Altera_desc fpga[CONFIG_FPGA_COUNT] = {
	{Altera_CYC2,
	 passive_serial,
	 Altera_EP2C35_SIZE,
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

