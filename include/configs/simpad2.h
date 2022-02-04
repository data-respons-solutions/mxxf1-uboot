/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SIMPAD2_CONFIG_H
#define __SIMPAD2_CONFIG_H

#include "lm_common.h"

#ifdef CONFIG_SPL
#define CONFIG_SYS_SPI_U_BOOT_OFFS (256 * 1024)
#define CONFIG_IMX_WATCHDOG
#define CONFIG_HW_WATCHDOG

#define CONFIG_WATCHDOG_TIMEOUT_MSECS 128000

#include "imx6_spl.h"
#undef CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 512
#endif

#define CONSOLE_DEV "ttymxc0"

#define CONFIG_MMCROOT "/dev/mmcblk0p1"
#define MMC_DEV "0"
#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

/* Framebuffer */
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#undef CONFIG_VIDEO_LOGO
#undef CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#define CONFIG_CMD_BMP

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV 0 /* SDHC3 */
#endif

#define CONFIG_PWM
#define CONFIG_PWM_IMX
#define CONFIG_IMX6_PWM_PER_CLK 66000000

#define CMD_CRC32


/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1 /* enable I2C bus 1 */
#undef CONFIG_SYS_I2C_MXC_I2C2 /* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3
#define CONFIG_SYS_I2C_SPEED 100000
#define CONFIG_SYS_I2C_MXC_I2C4

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR 0x08


/* USB Configs */
#define CONFIG_USBD_HS
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS 0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1 /* Enabled USB controller number */

#endif /* __SIMPAD2_CONFIG_H */
