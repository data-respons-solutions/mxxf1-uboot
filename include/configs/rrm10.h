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

#ifndef __RRM10_CONFIG_H
#define __RRM10_CONFIG_H

#define CONFIG_MACH_TYPE	4444
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc0"
#define CONFIG_MMCROOT			"/dev/mmcblk0p1"
#define CONFIG_DEFAULT_FDT_FILE	"rrm10.dtb"
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
#define CONFIG_MX6Q

#include "mx6_common.h"

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#undef DEBUG
//#define CONFIG_SYS_DCACHE_OFF
/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(32 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

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
#define CONFIG_CMD_I2C
#ifdef CONFIG_CMD_I2C
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_EEPROM_ON_BUS 1
#define CONFIG_EEPROM_ADDR	0x50
#endif

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


/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE               115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY               8

#define CONFIG_LOADADDR                0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000

#define CONFIG_ENV_SIZE			(8 * 1024)

#undef CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_IS_IN_SPI_FLASH
#undef CONFIG_ENV_IS_IN_MMC

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SDHC3 */
#define CONFIG_SYS_MMC_ENV_PART		2	/* Boot partition 2 */
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		0xc0000
#define CONFIG_ENV_SECT_SIZE		0x10000
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

/* Framebuffer and LCD */
#undef CONFIG_VIDEO

#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_IPUV3_CLK 260000000
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE


#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=/boot/boot.scr\0" \
	"uimage=/boot/uImage\0" \
	"initrd=/boot/initrd\0" \
	"ip_dyn=yes\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"boot_dev=mmc 1:1\0" \
	"root_dev=/dev/mmcblk0p1\0" \
	"kernelargs=setenv bootargs console=${console},${baudrate} " \
		"root=${root_dev} rootwait rw rootfstype=ext4 fec_mac=${ethaddr}\0" \
	"boot_mmc=usb start; setenv boot_dev mmc 1:1; setenv root_dev /dev/mmcblk0p1; run kernelargs; run loaduimage; bootm ${loadaddr}\0" \
	"boot_usb=usb start; setenv boot_dev usb 0:1; setenv root_dev /dev/sda1; sleep 2; run kernelargs; run loaduimage; bootm ${loadaddr}\0" \
	"boot_sata=sata init; setenv boot_dev sata 0:1; setenv root_dev /dev/sda1; run kernelargs; run loaduimage; bootm ${loadaddr}\0" \
	"loadbootscript=ext4load ${boot_dev} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from ${boot_dev} ...; source\0" \
	"loaduimage=ext4load ${boot_dev} ${loadaddr} ${uimage}\0" \

#define CONFIG_BOOTCOMMAND \
	"mmc dev 1;" \
	"if mmc rescan; then " \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"run boot_mmc; " \
		"fi; " \
	"else run boot_usb; fi"

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_PROMPT              "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              256

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
#define CONFIG_STACKSIZE               (128 * 1024)

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





#endif                         /* __RRM10_CONFIG_H */
