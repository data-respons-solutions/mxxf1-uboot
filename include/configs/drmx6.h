/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.         See the
 * GNU General Public License for more details.
 */

#ifndef __DRMX6_CONFIG_H
#define __DRMX6_CONFIG_H


#ifdef CONFIG_SPL

#define CONFIG_SPL_FRAMEWORK

#define	CONFIG_SPL_LDSCRIPT	"arch/arm/cpu/armv7/mx6/u-boot-spl.lds"
#define CONFIG_SPL_TEXT_BASE	0x00910000
#define CONFIG_SPL_MAX_SIZE	(128 * 1024)
#define CONFIG_SPL_START_S_PATH	"arch/arm/cpu/armv7"

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT

#define CONFIG_SPL_GPIO_SUPPORT

#undef CONFIG_SPL_FAT_SUPPORT

#if defined(CONFIG_SPL_FAT_SUPPORT) || defined(CONFIG_SPL_EXT_SUPPORT)
#define CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION	1
#define CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME	"u-boot.img"
#define CONFIG_SPL_LIBDISK_SUPPORT
#endif

#define CONFIG_SPL_I2C_SUPPORT
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_BUS 0
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SYS_SPI_U_BOOT_OFFS (1024 * 256)

#define CONFIG_SPL_SPI_CS   (1|(IMX_GPIO_NR(3, 19)<<8))
#undef CONFIG_SPL_SATA_SUPPORT
#define CONFIG_SPL_PWM_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	2*256
#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS	800
#define CONFIG_SYS_MONITOR_LEN	(CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS/2*1024)

#define CONFIG_SPL_BOARD_INIT

#define CONFIG_SPL_BSS_START_ADDR	0x18200000
#define CONFIG_SPL_BSS_MAX_SIZE		0x100000
#define CONFIG_SYS_SPL_MALLOC_START	0x00930000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x00010000
#define CONFIG_SYS_TEXT_BASE		0x17800000
#define CONFIG_SPL_STACK 0x00928000

#define CONFIG_MX6_DDRTUNE

#endif


#define CONFIG_MACH_TYPE	4444
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc0"
#define CONFIG_MMCROOT			"/dev/mmcblk0p1"
#define CONFIG_DEFAULT_FDT_FILE	"/boot/drmx6.dtb"
#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)

/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.         See the
 * GNU General Public License for more details.
 */

#define CONFIG_MX6

#include "mx6_common.h"

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#ifndef CONFIG_SPL_BUILD
#define RRM_VIDEO
#endif

#ifdef RRM_VIDEO
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_CONSOLE_MUX
#define CONFIG_CFB_CONSOLE_ANSI
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_IMX_HDMI
#endif
/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(32 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_IMX_PWM

#undef CONFIG_IMX_WATCHDOG
#define CONFIG_WATCHDOG_TIMEOUT_MSECS 128000
#define CONFIG_IMX_WATCHDOG_USE_WD2

/* SPI */

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_CMD_SPI
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   (1|(IMX_GPIO_NR(3, 19)<<8))
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif


/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		5

#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* SATA */
#define CONFIG_CMD_SATA

#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#define CONFIG_DRIVE_SATA "sata "
#else
#define CONFIG_DRIVE_SATA
#endif

/* I2C Configs */

#define CONFIG_I2C_MULTI_BUS
#define CONFIG_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_CMD_I2C
#define CONFIG_EEPROM_ON_BUS 1
#define CONFIG_EEPROM_ADDR	0x50


/* OCOTP Configs */
#define CONFIG_CMD_IMXOTP
#ifdef CONFIG_CMD_IMXOTP
#define CONFIG_IMX_OTP
#define IMX_OTP_BASE			OCOTP_BASE_ADDR
#define IMX_OTP_ADDR_MAX		0x7F
#define IMX_OTP_DATA_ERROR_VAL		0xBADABADA
#define IMX_OTPWRITE_ENABLED
#endif


/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_MXC_USB_PORT	1
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0
#define CONFIG_USB_KEYBOARD


#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE               115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY               4

#define CONFIG_LOADADDR                0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000

#define CONFIG_ENV_SIZE			(64 * 1024)

#define CONFIG_ENV_OFFSET		0xc0000
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0
#endif
#if defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_SECT_SIZE		0x10000

#endif



/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE


#define CONFIG_EXTRA_ENV_SETTINGS \
	"zimage=/boot/zImage\0" \
	"panel=RRM10-XGA\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdt_addr=0x11000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=try\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"setargs=setenv bootargs console=${console},${baudrate} root=${rootdev} rootwait rw rootfstype=ext4 fec_mac=${ethaddr}\0" \
	"bootdev=0\0" \
	"bootpart=1\0" \
	"bootfrom=mmc\0" \
	"rootdev=/dev/mmcblk0p1\0" \
	"loadbootscript=if ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/boot.txt; then env import -t ${loadaddr} ${filesize}; fi; \0" \
	"setmmc=setenv bootfrom mmc; setenv bootdev 0; setenv bootpart 1; setenv rootdev /dev/mmcblk0p1; echo Setting boot to mmc; \0 " \
	"setusb=setenv bootfrom usb; setenv bootdev 0; setenv bootpart 1; setenv rootdev /dev/sda1; echo Setting boot to usb; \0 " \
	"setsata=setenv bootfrom sata; setenv bootdev 0; setenv bootpart 1; setenv rootdev /dev/sda1; echo Setting boot to sata; \0 " \
	"loaduboot=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/u-boot.imx; \0" \
	"loadimage=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} ${zimage}; \0" \
	"loadfdt=ext4load ${bootfrom} ${bootdev}:${bootpart} ${fdt_addr} ${fdt_file}; \0" \
	"bootscript=run setargs; run loadimage; run loadfdt; bootz ${loadaddr} - ${fdt_addr}; \0" \
	"flash_uboot=if usb start && usb storage; then "\
			"run setusb; " \
			"if run loaduboot; then " \
				"echo WARNING: Flashing u-boot; sf probe; sf erase 0 0xd0000; sf write ${loadaddr} 0x400 ${filesize};" \
				"echo DONE, resetting; reset; " \
			"else " \
				"echo No u-boot.imx in /boot;" \
			"fi;" \
		"fi; \0" \

#define CONFIG_BOOTCOMMAND \
	"if usb start && usb storage; then " \
		"run setusb; echo booting from USB ...;" \
	"else " \
		"mmc dev 1; " \
		"if mmc rescan; then " \
			"run setmmc; echo booting from MMC ...;" \
		"fi; "\
	"fi; " \
	"run loadbootscript;" \
	"run bootscript;"

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_PROMPT              "U-Boot (rrm10) > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              512

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS             16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR
#define CONFIG_SYS_HZ                  1000

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_FSL_USDHC_NUM	2




#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif





#endif                         /* __DRMX6_CONFIG_H */
