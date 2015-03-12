/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MXXF1_COMMON_CONFIG_H
#define __MXXF1_COMMON_CONFIG_H

#define CONFIG_MX6

#include "mx6_common.h"
#include <linux/sizes.h>

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_DM
#define CONFIG_DM_THERMAL
#define CONFIG_SYS_MALLOC_F_LEN	(1 << 10)
#define CONFIG_IMX6_THERMAL

#define CONFIG_SYS_GENERIC_BOARD

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

#define CONFIG_CMD_FUSE
#if defined(CONFIG_CMD_FUSE) || defined(CONFIG_IMX6_THERMAL)
#define CONFIG_MXC_OCOTP
#endif
#define CONFIG_IMX_DDRTUNE

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
#define CONFIG_FEC_MXC_PHYADDR		1

#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		1
#define CONFIG_SF_DEFAULT_SPEED		20000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY               4

#define CONFIG_LOADADDR                        0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000

#define CONFIG_EXTRA_ENV_SETTINGS \
	"zimage=/boot/zImage\0" \
	"fdt_addr=0x11000000\0" \
	"ip_dyn=try\0" \
	"console=" CONFIG_CONSOLE_DEV ",115200\0" \
	"fdt_high=0xffffffff\0"	  \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"initrd_file=/boot/initrd\0" \
	"initrd_high=0xffffffff\0" \
	"loglevel=7\0" \
	"consoleblank=300\0" \
	"showtty=console=ttymxc0,115200 console=tty1\0" \
	"setargs=setenv bootargs console=${console} root=${rootdev} rootwait ro rootfstype=ext4 fec_mac=${ethaddr} consoleblank=${consoleblank} loglevel=${loglevel} ${showtty}\0" \
	"bootdev=0\0" \
	"bootpart=1\0" \
	"bootfrom=mmc\0" \
	"mmc_root=" CONFIG_MMCROOT "\0" \
	"usb_root=/dev/sda1\0" \
	"loadbootscript=if ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/boot.txt; then env import -t ${loadaddr} ${filesize}; fi; \0" \
	"setmmc=setenv bootfrom mmc; setenv bootdev "CONFIG_MMC_DEV" ; setenv bootpart 1; setenv rootdev ${mmc_root}; echo Setting boot to mmc; \0 " \
	"setusb=setenv bootfrom usb; setenv bootdev 0; setenv bootpart 1; setenv rootdev ${usb_root}; echo Setting boot to usb; \0 " \
	"loaduboot=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/u-boot.img; \0" \
	"loadspl=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/SPL; \0" \
	"flashspl=if run loadspl; then sf erase 0 10000; sf write ${loadaddr} 400 ${filesize}; fi; \0" \
	"flashuboot=if run loaduboot; then sf erase 40000 90000; sf write ${loadaddr} 40000 ${filesize}; fi; \0" \
	"loadimage=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} ${zimage}; \0" \
	"loadinitrd=ext4load ${bootfrom} ${bootdev}:${bootpart} ${initrd_addr} ${initrd_file}; \0" \
	"loadfdt=ext4load ${bootfrom} ${bootdev}:${bootpart} ${fdt_addr} ${fdt_file}; \0" \
	"bootscript=run setargs; if run loadimage loadfdt; then bootz ${loadaddr} - ${fdt_addr}; else echo ERROR: Could not load image; fi; \0" \
	"check_usb_boot=if usb storage; then run setusb loadfdt; fi;\0" \
	"initrd_addr=0x12C00000\0" \
	"initrd_high=0xffffffff\0" \
	"factory_args=setenv bootargs console=${console} rdinit=/linuxrc enable_wait_mode=off" \
	"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
	"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
	"g_mass_storage.iSerialNumber=\"\" \0"\
	"factory_boot=run factory_args; bootz ${loadaddr} ${initrd_addr} ${fdt_addr}; \0" \
	"testfact=run loadfdt loadimage loadinitrd factory_boot; \0"
#ifdef CONFIG_FACTORY_BOOT
#define CONFIG_BOOTCOMMAND \
	"run factory_boot;"
#else
#define CONFIG_BOOTCOMMAND \
	"mmc dev "CONFIG_MMC_DEV"; mmc rescan; " \
	"if run check_usb_boot; then " \
		"echo booting from USB ...;" \
	"else " \
		"run setmmc; echo booting from MMC ...;" \
	"fi; " \
	"run loadbootscript;" \
	"run bootscript;"
#endif

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              256
#define CONFIG_SYS_MAXARGS             16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

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
#define CONFIG_ENV_OVERWRITE


#define CONFIG_ENV_SIZE			(64 * 1024)

#ifdef CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SECT_SIZE 	(64 * 1024)
#define CONFIG_ENV_SPI_BUS             CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS              CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE            CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ          CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_OFFSET		(0xc0000)
#else
#define CONFIG_ENV_IS_NOWHERE
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

/* Framebuffer */
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
#define CONFIG_IMX_VIDEO_SKIP
#endif

#endif                         /* __MXXF1_COMMON_CONFIG_H */
