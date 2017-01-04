#
# Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
#
# (C) Copyright 2011 Freescale Semiconductor, Inc.
#
# SPDX-License-Identifier:	GPL-2.0+
#


obj-y  := linkbox-module.o 
ifdef CONFIG_SPL_BUILD
obj-y += ../lm-common/spl_stage.o
else
obj-y += ../lm-common/vpd.o ../lm-common/param.o
endif
