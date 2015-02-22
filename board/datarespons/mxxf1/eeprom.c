#ifndef CONFIG_SPL_BUILD
#include <common.h>
#include <i2c.h>
#include <asm/errno.h>
#include <linux/ctype.h>
#include <u-boot/crc.h>
#include "param.h"
#include "mxxf1_gpio.h"
#include <malloc.h>
#include <version.h>
#include <asm/gpio.h>

static char eeprom_content[CONFIG_EEPROM_SIZE];

static struct param ee;
static int eeprom_ok = 0;
static unsigned eeprom_addr = 0;

int mxxf1_eeprom_init(unsigned dev_addr)
{
	unsigned offset = 0;
	unsigned end = CONFIG_EEPROM_SIZE;
	unsigned blk_off;
	char *buffer = eeprom_content;
	eeprom_addr = dev_addr;
	i2c_set_bus_num(1);
	//printf("%s with dev_addr= 0x%x, offs=0x%x, cnt=%d\n", __func__, dev_addr, offset, cnt);
	/* Read data until done or would cross a page boundary.
	 * We must write the address again when changing pages
	 * because the next page may be in a different device.
	 */
	while (offset < end)
	{
		unsigned len;
		unsigned maxlen;
		uchar segment_addr = dev_addr;
		blk_off = offset & 0xFF; /* block offset */

		segment_addr |= (offset >> 8) & 0x7; /* block number */
		len = end - offset;
		maxlen = 0x100 - blk_off;
		if ( maxlen > I2C_RXTX_LEN )
			maxlen = I2C_RXTX_LEN;
		if (len > maxlen)
			len = maxlen;

		if (i2c_read(segment_addr, blk_off, 1, (uint8_t*) buffer, len) != 0)
		{
			printf("%s: IO error\n", __func__);
			return -EIO;
		}

		buffer += len;
		offset += len;
	}
	param_init(&ee, eeprom_content, CONFIG_EEPROM_SIZE);
	eeprom_ok = param_parse(&ee) == 0 ? 1 : 0;
	return 0;
}

int mxxf1_eeprom_write(unsigned dev_addr, unsigned offset, uint8_t *buffer,
		unsigned cnt)
{
	unsigned end = offset + cnt;
	unsigned blk_off;
	int rcode = 0;
	i2c_set_bus_num(1);
	/* Write data until done or would cross a write page boundary.
	 * We must write the address again when changing pages
	 * because the address counter only increments within a page.
	 */
	printf("\n%s: Program eeprom at 0x%02x, offset %d, length %d ", __func__,
			dev_addr, offset, cnt);
	gpio_set_value(GPIO_EEPROM_WP, 0);
	while (offset < end)
	{
		unsigned len;
		unsigned maxlen;
		uchar segment_addr = dev_addr;

		blk_off = offset & 0xFF; /* block offset */
		segment_addr |= (offset >> 8) & 0x7; /* block number */
		len = end - offset;
		maxlen = 0x100 - blk_off;
		if ( maxlen > 16)
			maxlen = 16;
		if (len > maxlen)
			len = maxlen;

		printf("%s: addr=0x%x, offs=0x%x, len=0x%x\n", __func__, segment_addr, blk_off, len);
		if (i2c_write(segment_addr, blk_off, 1, buffer, len) != 0)
		{
			printf("%s: IO error\n", __func__);
			gpio_set_value(GPIO_EEPROM_WP, 1);
			return -EIO;
		}

		buffer += len;
		offset += len;
		printf(".");
		udelay(6 * 1000);

	}
	printf("\n%s: Done\n", __func__);
	gpio_set_value(GPIO_EEPROM_WP, 1);
	return rcode;
}

int eeprom_get_mac_addr(void)
{
	char *data, *key;
	int ret = param_find(&ee, "FEC_MAC_ADDR");
	if (ret >= 0)
	{
		ret = param_split(ee.param[ret], &key, &data);
		if (ret == 0)
		{
			setenv("ethaddr", data);
			free(key);
			return 0;
		}
		else
			return ret;

	}
	else
	{
		printf("%s: FEC_MAC_ADDR not present in eeprom\n", __func__);
		return -1;
	}

	char *base = (char*) &eeprom_content[4];
	char *at = base;
	char *at_end = base + sizeof(eeprom_content);
	int len;

	while (at < at_end)
	{
		char *p = strstr(at, "FEC_MAC_ADDR=");
		if (p)
		{
			printf("%s: found MAC info %s\n", __func__, p);
			p += 13;
			if ((len = strnlen(p, 20)) == 17)
			{
				setenv("ethaddr", p);
			}
			else
				printf("%s: Illegal length %d for mac address\n", __func__,
						len);
			break;
		}
		else
		{
			while (at < at_end && *at != '\0')
				at++;
			if (at >= at_end)
			{
				ret = -EINVAL;
				break;
			}
			else
				at++;
		}

	}
	if (ret)
		printf("%s: FEC_MAC_ADDR not present in eeprom\n", __func__);

	return ret;
}

int vpd_update_eeprom(void)
{
	char *data, *key;
	int status;
	int index = param_find(&ee, "UBOOT_VERSION");

	if (index < 0)
	{
		printf("%s: UBOOT_VERSION not found in EEPROM, adding it\n", __func__);
		status = param_add(&ee, strcat("UBOOT_VERSION=", U_BOOT_VERSION));
		if (status)
			return status;
	}
	else
	{
		status = param_split(ee.param[index], &key, &data);
		if (status < 0)
		{
			printf("%s: param_split failed [%d]\n", __func__, status);
			return 0;
		}
		if ((data && strncmp(data, U_BOOT_VERSION, 120)) || NULL == data)
		{
			printf("%s: Updating u-boot version from [%s] to [%s]\n", __func__,
					data ? data : "void", U_BOOT_VERSION);
			param_update(&ee, index, strcat("UBOOT_VERSION=", U_BOOT_VERSION));
		}
		else
			return 0;
	}
	param_generate(&ee);
	status = mxxf1_eeprom_write(eeprom_addr, 0, ee.data, ee.size);
	eeprom_ok = status ? 0 : 1;
	return 1;
}

static int do_vpd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int n;
	if (!eeprom_ok)
	{
		printf("BAD EEPROM CRC\n");
		return 0;
	}
	for (n = 0; n < ee.count; n++)
	{
		printf("%s\n", ee.param[n]);
	}
	return 0;
}

U_BOOT_CMD(
		vpd, 2, 1, do_vpd,
		"show VPD data",
		"  Print Vital Product Data for MXXF1\n"
);

#endif
