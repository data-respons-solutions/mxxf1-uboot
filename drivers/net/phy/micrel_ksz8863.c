/*
 * Micrel PHY drivers
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
 *
 * Copyright 2010-2011 Freescale Semiconductor, Inc.
 * author Andy Fleming
 *
 */
#include <config.h>
#include <common.h>
#include <micrel.h>
#include <phy.h>


static int ksz8863_config(struct phy_device *phydev)
{
	printf("%s: At addr %d\n", __func__, phydev->addr);

	if (phydev->addr < 4) {
		phydev->advertising = phydev->supported =  SUPPORTED_100baseT_Full;
	}
	else
		phydev->advertising = phydev->supported = phydev->drv->features;

	return 0;
}

static int ksz8863_startup(struct phy_device *phydev)
{
	printf("%s\n", __func__);
	phydev->advertising = phydev->supported =  SUPPORTED_100baseT_Full;
	genphy_update_link(phydev);
	phydev->speed = SPEED_100;
	return 0;
}

static struct phy_driver KSZ8863_driver = {
	.name = "Micrel KSZ8863",
	.uid = 0x00221430,
	.mask = 0xffffffff,
	.features = PHY_BASIC_FEATURES,
	.config = &ksz8863_config,
	.startup = &ksz8863_startup,
	.shutdown = &genphy_shutdown,
};




int phy_micrel8863_init(void)
{
	return phy_register(&KSZ8863_driver);
}
