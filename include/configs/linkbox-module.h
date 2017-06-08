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
#include <asm/imx-common/gpio.h>

#ifdef CONFIG_SPL

#include "imx6_spl.h"
#undef CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 512
#endif

#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV		"ttymxc0"
#define CONFIG_IMX_WATCHDOG
#undef CONFIG_IMX_WATCHDOG_USE_WD2

#define CONFIG_MMCROOT			"/dev/mmcblk0p1"
#define MMC_DEV "0"
#define PHYS_SDRAM_SIZE		(1u * 512 * 1024 * 1024)
#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

#undef CONFIG_VIDEO
#include "lm_common.h"

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC3 */
#endif

#define CONFIG_PWM_IMX
#define CONFIG_IMX6_PWM_PER_CLK	66000000
#define CONFIG_SYS_I2C_MXC_I2C1
#define CONFIG_SYS_I2C_MXC_I2C2
#define CONFIG_SYS_I2C_MXC_I2C3
#define CONFIG_SYS_I2C_MXC_I2C4

#define CMD_CRC32


/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		  100000
/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */

#endif

#endif                         /* __MXXF1_CONFIG_H */
