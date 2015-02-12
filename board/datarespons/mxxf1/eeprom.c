#ifndef CONFIG_SPL_BUILD
#include <common.h>
#include <i2c.h>
#include <asm/errno.h>
#include <linux/ctype.h>
#include <u-boot/crc.h>
#include "param.h"

static char eeprom_content[CONFIG_EEPROM_SIZE];

static struct param ee;

int mxxf1_eeprom_init (unsigned dev_addr)
{
	unsigned offset = 0;
	unsigned end = CONFIG_EEPROM_SIZE;
	unsigned blk_off;
	uint32_t crc, c_crc;
	char *buffer = eeprom_content;
	i2c_set_bus_num(1);
	//printf("%s with dev_addr= 0x%x, offs=0x%x, cnt=%d\n", __func__, dev_addr, offset, cnt);
	/* Read data until done or would cross a page boundary.
	 * We must write the address again when changing pages
	 * because the next page may be in a different device.
	 */
	while (offset < end) {
		unsigned len;
		unsigned maxlen;
		uchar segment_addr = dev_addr;
		blk_off = offset & 0xFF;	/* block offset */

		segment_addr |=  (offset >> 8) & 0x7;		/* block number */
		len = end - offset;
		maxlen = 0x100 - blk_off;
		if (len > maxlen)
			len = maxlen;

		//printf("i2c_read from %s with addr= 0x%x, offs=0x%x, cnt=%d\n", __func__, addr[0], blk_off, len);
		if (i2c_read (segment_addr, blk_off, 1, (uint8_t*)buffer, len) != 0)
			return -EIO;

		buffer += len;
		offset += len;
	}
	crc = eeprom_content[0] | (eeprom_content[1] << 8) | (eeprom_content[2] << 16) | (eeprom_content[3] << 24);
	c_crc = crc32(0, (uint8_t*)eeprom_content + 4, CONFIG_EEPROM_SIZE-4);
	if (crc != c_crc) {
		printf("%s, bad CRC expected 0x%08x, got 0x%08x\n", __func__, crc, c_crc);
		return -1;
	}
	param_init(&ee, eeprom_content, CONFIG_EEPROM_SIZE);
	param_parse(&ee);
	return 0;
}


int mxxf1_eeprom_write (unsigned dev_addr, unsigned offset, uchar *buffer, unsigned cnt)
{
	unsigned end = offset + cnt;
	unsigned blk_off;
	int rcode = 0;
	i2c_set_bus_num(1);
	/* Write data until done or would cross a write page boundary.
	 * We must write the address again when changing pages
	 * because the address counter only increments within a page.
	 */

	while (offset < end) {
		unsigned len;
		unsigned maxlen;
		uchar segment_addr = dev_addr;

		blk_off = offset & 0xFF;	/* block offset */
		segment_addr |= (offset >> 8) & 0x7;		/* block number */
		len = end - offset;
		maxlen = 0x100 - blk_off;
		if (len > maxlen)
			len = maxlen;

		if (i2c_write (segment_addr, blk_off, 1, buffer, len) != 0)
			return -EIO;


		buffer += len;
		offset += len;


		udelay(2 * 1000);

	}

	return rcode;
}

int eeprom_get_mac_addr(void)
{
	char *data, *key;
	int ret=param_find(&ee, "FEC_MAC_ADDR");
	if ( ret >= 0) {
		param_split(ee.param[ret], &key, &data);
		setenv("ethaddr", data);
		return 0;
	}
	else {
		printf("%s: FEC_MAC_ADDR not present in eeprom\n", __func__);
		return -1;
	}


	char *base = (char*)&eeprom_content[4];
	char *at = base;
	char *at_end = base + sizeof(eeprom_content);
	int len;

	while ( at < at_end)
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
				printf("%s: Illegal length %d for mac address\n", __func__, len);
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

#endif
