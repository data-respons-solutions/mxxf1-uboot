/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <asm/imx-common/sata.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
//#include <asm/arch/mx6-ddr.h>

#include <usb.h>
#include <pwm.h>
#include <version.h>
#include <watchdog.h>
#include <video.h>

DECLARE_GLOBAL_DATA_PTR;

#include "../lm-common/lm_common_defs.h"
#include "sperre_pins.h"
#include "sperre_gpio.h"

struct fsl_esdhc_cfg usdhc_cfg = USDHC4_BASE_ADDR;

#ifndef CONFIG_SPL_BUILD
static const char* hw_string[8] = {
	"REVA",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
	"FUTURE",
};
#endif


