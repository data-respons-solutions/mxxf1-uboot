/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MXXF1_CONFIG_H
#define __MXXF1_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/gpio.h>

#ifdef CONFIG_SPL
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SYS_SPI_U_BOOT_OFFS (256 * 1024)
#define CONFIG_SPL_PWM_SUPPORT
#define CONFIG_IMX_WATCHDOG
#define CONFIG_HW_WATCHDOG

#define CONFIG_IMX_WATCHDOG_ASSERT_WDOG_B
#define CONFIG_WATCHDOG_TIMEOUT_MSECS 128000

#include "imx6_spl.h"
#undef CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 512
#endif


#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc0"

#ifdef CONFIG_EMU_SABRESD
#define CONFIG_DEFAULT_FDT_FILE	"/boot/simpad2-sabresd.dtb"
#define CONFIG_MMCROOT			"/dev/mmcblk1p1"
#define CONFIG_MMC_DEV "1"
#else
#define CONFIG_MMCROOT			"/dev/mmcblk0p1"
#define CONFIG_MMC_DEV "0"
#endif
#define PHYS_SDRAM_SIZE		(1u * 512 * 1024 * 1024)
#define CONFIG_MX6_DDRTUNE_WR_LEVEL
#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

#define CONFIG_VIDEO

#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_SIZE			(64 * 1024)


/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_SYS_CONSOLE_FG_COL 255
#define CONFIG_SYS_CONSOLE_BG_COL 0
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#undef CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#undef CONFIG_VIDEO_LOGO
#undef CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP
#define CONFIG_CMD_BMP
#endif
#include "lm_common.h"

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC3 */
#endif

#define CONFIG_IMX6_PWM_PER_CLK	66000000

#define CMD_CRC32


/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		  100000
#define CONFIG_SYS_I2C_MXC_I2C4

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08


#endif                         /* __MXXF1_CONFIG_H */
