#include <common.h>
#include <i2c.h>
#include <asm/errno.h>





int mxxf1_eeprom_read (unsigned dev_addr, unsigned offset, uchar *buffer, unsigned cnt)
{
	unsigned end = offset + cnt;
	unsigned blk_off;
	int rcode = 0;

	//printf("%s with dev_addr= 0x%x, offs=0x%x, cnt=%d\n", __func__, dev_addr, offset, cnt);
	/* Read data until done or would cross a page boundary.
	 * We must write the address again when changing pages
	 * because the next page may be in a different device.
	 */
	while (offset < end) {
		unsigned alen, len;
		unsigned maxlen;
		uchar segment_addr = dev_addr;
		blk_off = offset & 0xFF;	/* block offset */

		segment_addr |=  (offset >> 8) & 0x7;		/* block number */
		len = end - offset;
		maxlen = 0x100 - blk_off;
		if (len > maxlen)
			len = maxlen;

		//printf("i2c_read from %s with addr= 0x%x, offs=0x%x, cnt=%d\n", __func__, addr[0], blk_off, len);
		if (i2c_read (segment_addr, blk_off, 1, buffer, len) != 0)
			return -EIO;

		buffer += len;
		offset += len;
	}

	return rcode;
}


int mxxf1_eeprom_write (unsigned dev_addr, unsigned offset, uchar *buffer, unsigned cnt)
{
	unsigned end = offset + cnt;
	unsigned blk_off;
	int rcode = 0;

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






