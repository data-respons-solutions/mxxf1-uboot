/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MXXF1_CONFIG_H
#define __MXXF1_CONFIG_H


#ifdef CONFIG_SPL
#define CONFIG_SYS_SPI_U_BOOT_OFFS (256 * 1024)

#include "imx6_spl.h"
#undef CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 512
#endif

#include "mx6_common.h"

#define CONFIG_MACH_TYPE	0xffffffff
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV		"ttymxc0"
#define CONFIG_MMCROOT			"/dev/mmcblk0p1"
#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)
#define CONFIG_IMX_WATCHDOG
#define CONFIG_IMX_THERMAL
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD

#if defined(CONFIG_CMD_FUSE) || defined(CONFIG_IMX6_THERMAL)
#define CONFIG_MXC_OCOTP
#endif

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(40 * SZ_1M)

#define CONFIG_MXC_UART

/* MMC Configs */
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0


#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1

#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS


#ifdef CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		1
#define CONFIG_SF_DEFAULT_SPEED		20000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

#define CONFIG_LOADADDR                0x12000000

#define CONFIG_EXTRA_ENV_SETTINGS \
	"zimage=/boot/zImage\0" \
	"uimage=/boot/uImage\0" \
	"panel=MXXF1-XGA\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=try\0" \
	"hasusbkbd=no\0" \
	"allow_usbkbd=yes\0" \
	"fdt_high=0xffffffff\0"	  \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"initrd_high=0xffffffff\0" \
	"loglevel=3\0" \
	"consoleblank=0\0" \
	"stdin=serial,usbkbd\0" \
	"showtty=console=ttymxc0,115200 console=tty1\0" \
	"console=" CONSOLE_DEV ",115200\0" \
	"setargs=setenv bootargs console=${console} root=${rootdev} rootwait ro rootfstype=ext4 fec_mac=${ethaddr} consoleblank=${consoleblank} loglevel=${loglevel} ${showtty}\0" \
	"bootdev=0\0" \
	"bootpart=1\0" \
	"bootfrom=mmc\0" \
	"rootdev=/dev/mmcblk0p1\0" \
	"usb_root=/dev/sda1\0" \
	"has_sata=0\0" \
	"loadbootscript=if ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/boot.txt; then env import -t ${loadaddr} ${filesize}; fi; \0" \
	"setmmc=setenv bootfrom mmc; setenv bootdev 0; setenv bootpart 1; setenv rootdev /dev/mmcblk0p1; echo Setting boot to mmc; \0 " \
	"setusb=setenv bootfrom usb; setenv bootdev 0; setenv bootpart 1; setenv rootdev ${usb_root}; echo Setting boot to usb; \0 " \
	"setsata=setenv bootfrom sata; setenv bootdev 0; setenv bootpart 1; setenv rootdev /dev/sda1; echo Setting boot to sata; \0 " \
	"loaduboot=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/u-boot.img; \0" \
	"loadspl=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} /boot/SPL; \0" \
	"flashspl=if run loadspl; then sf erase 0 10000; sf write ${loadaddr} 400 ${filesize}; fi; \0" \
	"flashuboot=if run loaduboot; then sf erase 40000 90000; sf write ${loadaddr} 40000 ${filesize}; fi; \0" \
	"loadimage=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} ${zimage}; \0" \
	"loaduimage=ext4load ${bootfrom} ${bootdev}:${bootpart} ${loadaddr} ${uimage}; \0" \
	"loadfdt=ext4load ${bootfrom} ${bootdev}:${bootpart} ${fdt_addr} ${fdt_file}; \0" \
	"bootscript=run setargs; if run loadimage loadfdt; then bootz ${loadaddr} - ${fdt_addr}; else echo ERROR: Could not load image; fi; \0" \
	"bootscript_legacy=run setargs; if run loaduimage ; then bootm ${loadaddr}; else echo ERROR: Could not load legacy image; fi; \0" \
	"check_usb_boot=if usb storage; then run setusb loadfdt; fi;\0" \
	"check_sata=if sata init; then setenv usb_root /dev/sdb1; setenv has_sata 1; fi;\0" \

#define CONFIG_BOOTCOMMAND \
	"mmc dev 1; mmc rescan; run check_sata; " \
	"if run check_usb_boot; then " \
		"echo booting from USB ...;" \
	"else " \
		"if itest ${has_sata} == 1; then " \
			"if run setsata loadfdt; then echo booting from SATA; else run setmmc; echo SATA failed, trying MMC ...; fi; " \
		"else " \
			"run setmmc; echo booting from MMC ...;" \
		"fi; " \
	"fi; " \
	"run loadbootscript;" \
	"run bootscript;" \
	"run bootscript_legacy;"

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR


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

/*  FLASH and environment organization */
#define CONFIG_ENV_OVERWRITE

#define CONFIG_ENV_SIZE			(64 * 1024)
#define CONFIG_ENV_SECT_SIZE 	(64 * 1024)
#define CONFIG_ENV_SPI_BUS             CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS              CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE            CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ          CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_OFFSET		(0xc0000)



/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#undef CONFIG_VIDEO_LOGO
#undef CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP
#endif

#define CONFIG_SYS_FSL_USDHC_NUM	2
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC3 */
#endif

#define CONFIG_PWM_IMX
#define CONFIG_IMX6_PWM_PER_CLK	66000000

#define MXXF1_EEPROM_ADDR   0x50
#define MXXF1_EEPROM_ADDR2  0x70
#define MXXF1_EEPROM_SIZE 	2048


#undef CONFIG_CMD_PCI
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(7, 12)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(3, 19)
#endif

/* SATA */
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2 /* Enabled USB controller number */
#define CONFIG_USBD_HS

#define TSUP_I2C 0x2a /* New touch i2c address */
#if 0
#define CONFIG_USB_KEYBOARD
#define CONFIG_CONSOLE_MUX
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP
#define CONFIG_SYS_STDIO_DEREGISTER
#endif

#endif

#endif                         /* __MXXF1_CONFIG_H */
