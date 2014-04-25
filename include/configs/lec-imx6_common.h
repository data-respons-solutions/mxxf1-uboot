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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.         See the
 * GNU General Public License for more details.
 */

#ifndef __LEC_IMX6_COMMON_CONFIG_H
#define __LEC_IMX6_COMMON_CONFIG_H

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
#define CONFIG_SPL_MMC_NUM 1
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

#define CONFIG_MX6

/*
#define CONFIG_SYS_L2_PL310
#define CONFIG_SYS_PL310_BASE		0x00A02000
#define CONFIG_SYS_CACHELINE_SIZE	32
*/
#ifdef CONFIG_MX6SOLO
#define CONFIG_MX6DL
#endif

/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

#undef CONFIG_ARCH_MISC_INIT

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

#include "mx6_common.h"
#include <asm/sizes.h>

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(32 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_LAST_STAGE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

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
#define CONFIG_FEC_MXC_PHYADDR		6

#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

/* FLASH organization */
/* #define CONFIG_SYS_NO_FLASH */			/* also sets default for _CMD_FLASH, _CMD_IMLS */
#ifndef CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_FLASH_BASE	0x19000000	/* LEC-iMX6 flash.c shadows SPI here: CRAMFS operates only mem mapped */
#define CONFIG_SYS_MAX_FLASH_BANKS	1	/* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT	2048	/* max number of sectors on one chip */
#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_MTDPARTS
/*#define CONFIG_MTD_PARTITIONS */			/* needed only for UBI */
#define CONFIG_CMD_JFFS2			/* U-Boot CRAMFS implementation only called via JFFS2 code */
#define CONFIG_SYS_JFFS2_SORT_FRAGMENTS		/* needed if FS was mounted & written to, instead of just mkfs.jffs2 */
#define CONFIG_CMD_CRAMFS
/* #define CONFIG_CRAMFS_CMDLINE */			/* read image from RAM, skipping MTD code */
#define MTDIDS_DEFAULT		"nor0=spi32764.0"
#define MTDPARTS_DEFAULT	"mtdparts=spi32764.0:384K(u-boot)ro,-(root)"
#define MTDPARTS_DEFAULT_ENV	"mtdparts=" MTDPARTS_DEFAULT "\0"
#define PARTITION_DEFAULT	"nor0,1"
#else
#define MTDPARTS_DEFAULT_ENV	""
#endif

/* Command definition */
#include <config_cmd_default.h>

/* #define CONFIG_CMD_BMODE */
#define CONFIG_CMD_BOOTZ
#undef CONFIG_CMD_FLASH			/* flinfo, erase, protect useless, we have "sf" cmd */
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_LOADB			/* we have usb, sata, sd, network, who needs ymodem? */
#undef CONFIG_CMD_LOADS			/* ... or S-Records? */
#define CONFIG_NO_CMD_DISKBOOT		/* boot_... vars below are more flexible than usbboot cmd */

#define CONFIG_BOOTDELAY               4

#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

#ifndef CONFIG_DEFAULT_FDT_FILE
#define CONFIG_DEFAULT_FDT_FILE "boot/lec-imx6q.dtb"
#endif
/* Default environment variables */
#define CONFIG_BOOTCMD_TEST		"run boot_rom" /* override bootcmd via GPIO */
#define CONFIG_BOOTCMD_CPU_BIOS_DEFAULT	"env default -f -a" /* but no "...; save" */
#define CONFIG_BOOTCMD_FORCE_RECOV	"run boot_usb"
#define CONFIG_BOOTFILE			"/boot/zImage"
#ifdef CONFIG_MX6DL
#define BOOT_SATA_DEFAULT_ENV ""
#else
#define BOOT_SATA_DEFAULT_ENV "boot_sata=dcache off; sata init; set load ext4load sata 0:1; set bootargs sda1 ro; run boot\0"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	"zimage=/boot/zImage\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdt_addr=0x11000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=try\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"setargs=setenv bootargs console=${console},${baudrate} root=${rootdev} rootwait rw rootfstype=ext4 fec_mac=${ethaddr} earlyprintk\0" \
	"bootdev=0\0" \
	"bootpart=1\0" \
	"bootfrom=mmc\0" \
	"rootdev=/dev/mmcblk1p1\0" \
	"loadbootscript=if ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/boot.txt; then env import -t ${loadaddr} ${filesize}; fi; \0" \
	"setmmc=setenv bootfrom mmc; setenv bootdev 1; setenv bootpart 1; setenv rootdev /dev/mmcblk1p1; echo Setting boot to mmc; \0 " \
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
#define CONFIG_SYS_PROMPT              "U-Boot> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              1024

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

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)

#if defined CONFIG_SYS_BOOT_SPINOR
#define CONFIG_SYS_USE_SPINOR
#define CONFIG_ENV_IS_IN_SPI_FLASH
#elif defined CONFIG_SYS_BOOT_EIMNOR
#define CONFIG_SYS_USE_EIMNOR
#define CONFIG_ENV_IS_IN_FLASH
#elif defined CONFIG_SYS_BOOT_NAND
#define CONFIG_SYS_USE_NAND
#define CONFIG_ENV_IS_IN_NAND
#elif defined CONFIG_SYS_BOOT_SATA
#define CONFIG_ENV_IS_IN_SATA
#define CONFIG_CMD_SATA
#else
#define CONFIG_ENV_IS_IN_MMC
#endif

#ifndef CONFIG_MX6DL
/*
 * SATA Configs
 */
#define CONFIG_CMD_SATA
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif
#endif

#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SST
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS 3
#define CONFIG_SF_DEFAULT_SPEED 15000000 /* SST26VF064B 104MHz, SST25VF032B 66MHz, iMX6 60MHz, but need to use 15MHz due to poor signal quality :-( */
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)

#ifdef CONFIG_SYS_USE_EIMNOR
#undef CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_FLASH_BASE           WEIM_ARB_BASE_ADDR
#define CONFIG_SYS_FLASH_SECT_SIZE	(128 * 1024)
#define CONFIG_SYS_MAX_FLASH_BANKS 1    /* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT 256   /* max number of sectors on one chip */
#define CONFIG_SYS_FLASH_CFI            /* Flash memory is CFI compliant */
#define CONFIG_FLASH_CFI_DRIVER         /* Use drivers/cfi_flash.c */
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE /* Use buffered writes*/
#define CONFIG_SYS_FLASH_EMPTY_INFO
#endif

#ifdef CONFIG_SYS_USE_NAND
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND stuff */
#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(8 * 64 * 1024)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(384 * 1024 - CONFIG_ENV_SIZE)
#define CONFIG_ENV_SECT_SIZE		CONFIG_ENV_SIZE
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_FLASH)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_SIZE			CONFIG_SYS_FLASH_SECT_SIZE
#define CONFIG_ENV_SECT_SIZE		CONFIG_SYS_FLASH_SECT_SIZE
#define CONFIG_ENV_OFFSET		(4 * CONFIG_SYS_FLASH_SECT_SIZE)
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(8 << 20)
#define CONFIG_ENV_SECT_SIZE		(128 << 10)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#elif defined(CONFIG_ENV_IS_IN_SATA)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_SATA_ENV_DEV		0
#define CONFIG_SYS_DCACHE_OFF /* remove when sata driver support cache */
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

/*
 * I2C configs
 */
#define CONFIG_CMD_I2C
/* #define CONFIG_HARD_I2C         1 */
#define CONFIG_I2C_MXC          1
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_I2C_SPEED            100000

/* Framebuffer */
#undef CONFIG_VIDEO
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_IMX_HDMI
#endif

#endif                         /* __LEC_IMX6_COMMON_CONFIG_H */
