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
#include <asm/io.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
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
	//gpio_set_value(GPIO_FPGA_CS_N, 0);
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
		printf("%s: STATUS = 0\n", __func__);
		return 0;
	}
	printf("%s: STATUS = 1\n", __func__);
	return 1;
}



static int fpga_write_bitbang(const void *buf, size_t len, int flush, int cookie)
{
	int index, n;
	const uint8_t *ptr = buf;
	uint8_t dat;

	printf("OCAS FPGA write (BITBANG)... buffer at 0x%08x, size = %d\n", (u32)ptr, len);
	gpio_direction_output(IMX_GPIO_NR(5, 13), 0);
	gpio_direction_output(IMX_GPIO_NR(5, 10), 0);

	for (index = 0; index < len; index++) {
		dat = ptr[index];
		for (n=0; n < 8; n++) {
			gpio_set_value(IMX_GPIO_NR(5, 13), 0);
			gpio_set_value(IMX_GPIO_NR(5, 10), (dat & (1 << n)) ? 1 : 0);
			gpio_set_value(IMX_GPIO_NR(5, 13), 1);
		}
		if (index % 0x10000 == 0)
					printf(".");
	}
	printf("%s: Done\n", __func__);
	return 0;

}

const uint8_t byte_rev_table[256] = {
	0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
	0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
	0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
	0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
	0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
	0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
	0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
	0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
	0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
	0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
	0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
	0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
	0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
	0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
	0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
	0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
	0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
	0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
	0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
	0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
	0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
	0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
	0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
	0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
	0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
	0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
	0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
	0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
	0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
	0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
	0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
	0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
};


#define bitrev8(x) (byte_rev_table[x])

uint16_t bitrev16(uint16_t x)
{
	return (bitrev8(x & 0xff) << 8) | bitrev8(x >> 8);
}

/**
 * bitrev32 - reverse the order of bits in a u32 value
 * @x: value to be bit-reversed
 */
uint32_t bitrev32(uint32_t x)
{
	return (bitrev16(x & 0xffff) << 16) | bitrev16(x >> 16);
}

/* writes the complete buffer to the FPGA
   writing the complete buffer in one function is much faster,
   then calling it for every bit */
static int fpga_write_fn(const void *buf, size_t len, int flush, int cookie)
{
#define SPI2_BUS 1
#define SPI2_CS1_FPGA_CONFIG 1
#define SPI2_BUS_FREQ	30000000
#define BIT_LENGTH	8
	const int c_bsize = 256;
	uint8_t *ptr = (uint8_t*)buf;
	struct spi_slave * spi_slave;
	int bytecount = 0;
	int ret = 0;
	unsigned long flags = 0;
	int n;
	unsigned config = SPI2_CS1_FPGA_CONFIG;
	int paddedLen = len+1;		/* Need some clocks after programming */
	int blocks = paddedLen/c_bsize;
	int rem = paddedLen % c_bsize;
	int bcnt;


	for (n=0; n < len; n++) {
		ptr[n] = byte_rev_table[ptr[n]];
	}

	/* Slave setup */

	spi_slave = spi_setup_slave(SPI2_BUS, config, SPI2_BUS_FREQ, SPI_MODE_0  );

	if (spi_slave == NULL) {
		printf("%s: spi_setup_slave returns NULL\n", __func__);
		return -EINVAL;
	}
	printf("OCAS FPGA write ... buffer at 0x%08x, size = %d, blocks = %d, rem = %d\n",
			(u32)ptr, len, blocks, rem);
	spi_claim_bus(spi_slave);


	/* Transfer data */
	for (bcnt = 0; bcnt < blocks; bcnt++, ptr += c_bsize) {
		ret = spi_xfer(spi_slave, BIT_LENGTH*c_bsize, ptr, NULL, flags);
		if (ret != 0) {
			printf("ERROR: Spi xfer failed, at byte cnt: %d!, err = %d\n", bytecount, ret);
			break;
		}
		if (bcnt % 0x200 == 0)
			printf(".");
	}
	for (n=0; n < rem; n++, ptr++) {
		ret = spi_xfer(spi_slave, BIT_LENGTH, ptr, NULL, flags);
		if (ret != 0) {
			printf("ERROR: Spi xfer failed, at byte cnt: %d!, err = %d\n", bytecount, ret);
			break;
		}
	}
	printf("\n SPI TFX DONE\n");

	spi_release_bus(spi_slave);
	spi_free_slave(spi_slave);

	return ret == 0 ? FPGA_SUCCESS : FPGA_FAIL;
}

/* Returns the state of CONF_DONE Pin */
static int fpga_done_fn(int cookie)
{

	return gpio_get_value(GPIO_FPGA_DONE);

}

/* called, when programming is aborted */
static int fpga_abort_fn(int cookie)
{
	return 0;
}

/* called, when programming was succesful */
static int fpga_post_fn(int cookie)
{
	if (gpio_get_value(GPIO_FPGA_STATUSn) == 0) {
		printf("%s: STATUSn low after prog!!! failed\n", __func__);
		return FPGA_FAIL;
	}

	return FPGA_SUCCESS;
}

/* Note that these are pointers to code that is in Flash.  They will be
 * relocated at runtime.
 */
Altera_CYC2_Passive_Serial_fns fpga_fns = {
	.pre = fpga_pre_fn,
	.config = fpga_config_fn,
	.status = fpga_status_fn,
	.done = fpga_done_fn,
#ifdef BIT_BANG_FPGA
	.write = fpga_write_bitbang,
#else
	.write = fpga_write_fn,
#endif
	.abort = fpga_abort_fn,
	.post = fpga_post_fn
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
	u32 reg = readl(CCM_CCGR1);
	printf("%s: Initialize FPGA interface CCM_CCGR1 = 0x%08x\n", __func__, reg);

	fpga_init();

	for (i = 0; i < CONFIG_FPGA_COUNT; i++) {
		printf("%s:%d: Adding fpga %d\n", __func__, __LINE__, i);
		fpga_add(fpga_altera, &fpga[i]);
	}
	return 1;
}

