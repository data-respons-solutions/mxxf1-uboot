/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014 LiPPERT ADLINK Technology GmbH
 *
 * Configuration settings for the ADLINK LEC-iMX6 board.
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

#ifndef __LEC_IMX6_CONFIG_H
#define __LEC_IMX6_CONFIG_H

#define CONFIG_MACH_TYPE	4444
/* Potential confusion: i.MX6 UART1,2,4,5 (gap!) implement SMARC SER0,1,3,2
 * (order!), CONFIG_CONS_INDEX=1,2,3,4, Linux enumerates ttymxc0,1,2,3. */
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV	"ttymxc0"

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_MXC_USB_PORT	1
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0

#include "lec-imx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SYS_MMC_ENV_DEV		0	/* eMMC */
#define CONFIG_SYS_MMC_ENV_PART		0	/* user partition */

/* Controls multiplexer: 0 enables SMARC SPI0, 1 enables SMARC I2S0 instead. */
#define CONFIG_I2S_AUDIO		0

#ifdef CONFIG_SYS_USE_SPINOR
#define CONFIG_SF_DEFAULT_CS   (3|(IMX_GPIO_NR(3, 25)<<8))
#endif

/*#define CONFIG_SPLASH_SCREEN*/
/*#define CONFIG_MXC_EPDC*/ /* E-ink Panel Display, consumer grade i.MX6 DL/S/SL only */

/*
 * SPLASH SCREEN Configs
 */
#if defined(CONFIG_SPLASH_SCREEN) && defined(CONFIG_MXC_EPDC)
	/*
	 * Framebuffer and LCD
	 */
	#define CONFIG_CMD_BMP
	#define CONFIG_LCD
	#define CONFIG_FB_BASE				(CONFIG_SYS_TEXT_BASE + 0x300000)
	#define CONFIG_SYS_CONSOLE_IS_IN_ENV
	#undef LCD_TEST_PATTERN
	/* #define CONFIG_SPLASH_IS_IN_MMC			1 */
	#define LCD_BPP					LCD_MONOCHROME
	/* #define CONFIG_SPLASH_SCREEN_ALIGN		1 */

	#define CONFIG_WORKING_BUF_ADDR			(CONFIG_SYS_TEXT_BASE + 0x100000)
	#define CONFIG_WAVEFORM_BUF_ADDR		(CONFIG_SYS_TEXT_BASE + 0x200000)
	#define CONFIG_WAVEFORM_FILE_OFFSET		0x600000
	#define CONFIG_WAVEFORM_FILE_SIZE		0xF0A00
	#define CONFIG_WAVEFORM_FILE_IN_MMC

#ifdef CONFIG_SPLASH_IS_IN_MMC
	#define CONFIG_SPLASH_IMG_OFFSET		0x4c000
	#define CONFIG_SPLASH_IMG_SIZE			0x19000
#endif
#endif /* CONFIG_SPLASH_SCREEN && CONFIG_MXC_EPDC */

#endif                         /* __LEC_IMX6_CONFIG_H */
