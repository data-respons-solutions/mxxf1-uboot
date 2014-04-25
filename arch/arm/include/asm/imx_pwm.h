/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __IMX_PWM_H__
#define __IMX_PWM_H__

struct pwm_device {
	unsigned long mmio_base;
	int duty_ns;
	int period_ns;
	int pwmo_invert;
};

int imx_pwm_config(int pwm_id, int duty_ns, int period_ns);
int imx_pwm_enable(int pwm_id);
int imx_pwm_disable(int pwm_id);

#endif
