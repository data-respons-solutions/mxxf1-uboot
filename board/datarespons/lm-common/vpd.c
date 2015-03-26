#ifndef CONFIG_SPL_BUILD
#include <common.h>
#include <i2c.h>
#include <asm/errno.h>
#include <linux/ctype.h>
#include <u-boot/crc.h>
#include "../lm-common/param.h"
#include <malloc.h>
#include <version.h>
#include <asm/gpio.h>

#define VPD_SIZE 2048
static char vpd_content[VPD_SIZE];

static struct param ee;
static int vpd_ok = 0;
static int vpd_valid = 0;
static unsigned vpd_addr = 0;

int simpad2_vpd_read(void)
{
	unsigned offset = 0;
	unsigned end = VPD_SIZE;
	unsigned blk_off;
	char *buffer = vpd_content;

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
		uchar segment_addr = vpd_addr;
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

	return 0;
}

int simpad2_vpd_init(unsigned dev_addr)
{
	vpd_ok = 0;
	vpd_valid = 0;
	vpd_addr = dev_addr;
	int res = simpad2_vpd_read();
	if (res)
		return res;
	vpd_ok = 1;
	param_init(&ee, vpd_content, VPD_SIZE);
	vpd_valid = param_parse(&ee) == 0 ? 1 : 0;
	return 0;
}

int simpad2_vpd_write(unsigned dev_addr, unsigned offset, uint8_t *buffer,
		unsigned cnt, int wp_gpio)
{
	unsigned end = offset + cnt;
	unsigned blk_off;
	int rcode = 0;
	int retries;
	i2c_set_bus_num(1);
	/* Write data until done or would cross a write page boundary.
	 * We must write the address again when changing pages
	 * because the address counter only increments within a page.
	 */
	printf("\n%s: Program vpd at 0x%02x, offset %d, length %d\n", __func__,
			dev_addr, offset, cnt);
	gpio_set_value(wp_gpio, 0);
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

		retries = 10;
		while (retries-- && i2c_write(segment_addr, blk_off, 1, buffer, len));

		if (retries <= 0)
		{
			printf("%s: IO error remains after timeout - giving up\n", __func__);
			gpio_set_value(wp_gpio, 1);
			return -EIO;
		}

		buffer += len;
		offset += len;
		printf(".");
		udelay(6 * 1000);

	}
	printf("\n%s: Done\n", __func__);
	gpio_set_value(wp_gpio, 1);
	return rcode;
}

int vpd_get_mac_addr(void)
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
		printf("%s: FEC_MAC_ADDR not present in vpd\n", __func__);
		return -1;
	}

	char *base = (char*) &vpd_content[4];
	char *at = base;
	char *at_end = base + sizeof(vpd_content);
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
		printf("%s: FEC_MAC_ADDR not present in vpd\n", __func__);

	return ret;
}

static int check_and_update(const char *key, const char *value)
{
	int dirty = 0;
	int psize;
	char *param;
	int index;
	if (param_check_key(key) || param_check_data(value))
	{
		printf("%s: Illegal key/value\n", __func__);
		return -EINVAL;
	}
	psize = strlen(key) + strlen(value) + 4;
	param = malloc(psize);
	strcpy(param, key);
	strcat(param, "=");
	strcat(param, value);
	index = param_find(&ee, key);
	if ( index < 0 )
	{
		param_add(&ee, param);
		dirty = 1;
	}
	else
	{
		if (strcmp(ee.param[index], param))
		{
			param_update(&ee, index, param);
			dirty = 1;
		}
	}
	free(param);
	return dirty;
}

int vpd_update(char *touch_fw_ver, int wp_gpio)
{
	int retries=3;
	int dirty=0;
	dirty = check_and_update("UBOOT_VERSION", U_BOOT_VERSION);
	if (touch_fw_ver)
		dirty |= check_and_update("TOUCH_FW", touch_fw_ver);

	if (dirty)
	{
		param_generate(&ee);
		while (retries--)
		{
			simpad2_vpd_write(vpd_addr, 0, ee.data, ee.size, wp_gpio);
			printf("%s: Validating VPD\n", __func__);
			memset(vpd_content, 0xff, VPD_SIZE);
			simpad2_vpd_read();
			if (memcmp(vpd_content, ee.data, ee.size))
			{
				printf("%s: vpd did not validate!\n", __func__);
				vpd_ok = 0;
				vpd_valid = 0;
			}
			else
			{
				printf("%s: vpd validated\n", __func__);
				vpd_ok = 1;
				vpd_valid = 1;
				return 1;
			}
		}
		printf("%s: Failed to write vpd\n", __func__);
	}
	return 0;
}

static int do_vpd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int n;

	if (!vpd_valid)
	{
		printf("BAD vpd CRC\n");
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
		"  Print Vital Product Data for simpad2\n"
);

#endif
