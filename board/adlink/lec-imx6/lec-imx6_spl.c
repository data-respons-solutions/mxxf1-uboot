/*
 * Author: Tungyi Lin <tungyilin1127@gmail.com>
 *
 * Derived from EDM_CF_IMX6 code by TechNexion,Inc
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */
#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#ifdef CONFIG_SPL
#include <spl.h>
#include <asm/arch/mx6_ddrtune.h>
#endif
#include <asm/errno.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <asm/imx_pwm.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SPL_BUILD)
int ddr_init(void);
static enum boot_device boot_dev;
enum boot_device get_boot_device(void);
typedef enum  { SW1AB, SW1C, SW3AB } pf100_regs;

static inline void setup_boot_device(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4 ;
	uint bt_mem_type = (soc_sbmr & 0x00000008) >> 3;
	uint bt_mem_mmc = (soc_sbmr & 0x00001000) >> 12;

	switch (bt_mem_ctl) {
	case 0x0:
		if (bt_mem_type)
			boot_dev = MX6_ONE_NAND_BOOT;
		else
			boot_dev = MX6_WEIM_NOR_BOOT;
		break;
	case 0x2:
			boot_dev = MX6_SATA_BOOT;
		break;
	case 0x3:
		if (bt_mem_type)
			boot_dev = MX6_I2C_BOOT;
		else
			boot_dev = MX6_SPI_NOR_BOOT;
		break;
	case 0x4:
	case 0x5:
		if (bt_mem_mmc)
			boot_dev = MX6_SD0_BOOT;
		else
			boot_dev = MX6_SD1_BOOT;
		break;
	case 0x6:
	case 0x7:
		boot_dev = MX6_MMC_BOOT;
		break;
	case 0x8 ... 0xf:
		boot_dev = MX6_NAND_BOOT;
		break;
	default:
		boot_dev = MX6_UNKNOWN_BOOT;
		break;
	}
}

enum boot_device get_boot_device(void) {
	return boot_dev;
}

#include "asm/arch/mx6_ddr_regs.h"

int rrm10_pmic_setup(void)
{
	int ret;
	i2c_set_bus_num(1);
	ret = i2c_probe(0x08);
	if (ret)
	{
		printf("%s: no pmic\n", __func__);
		return -ENODEV;
	}
	return 0;
}


static int rrm_pmic_set(pf100_regs reg, int mV)
{
	u8 values[2];

	switch (reg) {

	case SW1AB:
		if (mV > 1425) {
			printf("%s: SW1AB max is 1425 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		values[0] = (mV - 300) / 25;
		i2c_write(0x08, 0x20, 1, values, 1);
		break;

	case SW1C:
		if (mV > 1425) {
			printf("%s: SW1AB max is 1425 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		values[0] = (mV - 300) / 25;
		i2c_write(0x08, 0x2E, 1, values, 1);
		break;

	case SW3AB:
		if (mV > 1500) {
			printf("%s: SW3AB max is 1500 mV, reject %d mV\n", __func__, mV);
			return -EINVAL;
		}
		values[0] = (mV - 400) / 25;
		i2c_write(0x08, 0x3C, 1, values, 1);
		i2c_write(0x08, 0x43, 1, values, 1);
		break;
	}

	return 0;
}
#define GPIO_LCD_EN	IMX_GPIO_NR(6, 15)
#define GPIO_BL_EN	IMX_GPIO_NR(1, 2)
#define GPIO_BL_PWM	IMX_GPIO_NR(1, 9)

void board_init_f(ulong dummy)
{	
	int err;
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	/* Set global data pointer. */
	gd = &gdata;

	board_early_init_f();	

	timer_init();
	preloader_console_init();

	err = rrm10_pmic_setup();
	if (err == 0) {
		/*rrm_pmic_set(SW1AB, 1425);
		rrm_pmic_set(SW1C, 1425); */
		rrm_pmic_set(SW3AB, 1350);
		udelay(100000);
	}

	err = mx6_ddr_init(CONFIG_SYS_MEMTEST_START);

	if (err) {
		printf("DDR3 calibration error - hang\n");
		while(1);

	}
	board_init_r(NULL, 0);
}

void spl_board_init(void)
{
	setup_boot_device();
}

u32 spl_boot_device(void)
{
	puts("Boot Device: ");
	switch (get_boot_device()) {
	case MX6_SD0_BOOT:
		printf("SD0\n");
		return BOOT_DEVICE_MMC1;
	case MX6_SD1_BOOT:
		printf("SD1\n");
		return BOOT_DEVICE_MMC2;
	case MX6_MMC_BOOT:
		printf("MMC\n");
		return BOOT_DEVICE_MMC2;
	case MX6_NAND_BOOT:
		printf("NAND\n");
		return BOOT_DEVICE_NAND;
	case MX6_SATA_BOOT:
		printf("SATA\n");
		return BOOT_DEVICE_SATA;
	case MX6_SPI_NOR_BOOT:
		printf("SPI\n");
		return BOOT_DEVICE_SPI;
	case MX6_UNKNOWN_BOOT:
	default:
		printf("UNKNOWN\n");
		return BOOT_DEVICE_NONE;
	}
}

u32 spl_boot_mode(void)
{
	switch (spl_boot_device()) {
	case BOOT_DEVICE_MMC1:
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC2_2:
		return MMCSD_MODE_RAW;
		break;
	case BOOT_DEVICE_SATA:
		return SATA_MODE;
		break;
	//case BOOT_DEVICE_NAND:
	//	return 0;
	//	break;
	default:
		puts("spl: ERROR:  unsupported device\n");
		hang();
	}
}

void reset_cpu(ulong addr)
{
	puts("reset_cpu\n");
	__REG16(WDOG1_BASE_ADDR) = 4;
}
#endif

