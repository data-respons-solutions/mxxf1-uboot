/*
 * Implement U-Boot NOR flash interface for SPI on ADLINK LEC-iMX6.
 * SPI flash itself can be accessed via "sf" command, but we need the NOR MTD
 * interface to use CRAMFS easily (ls, fsload cmds).		--JR10/2013
 *
 * (C)2013 LiPPERT ADLINK Technology GmbH, released under the GNU GPLv2
 */
#include <common.h>
#include <spi_flash.h>
#include <linux/mtd/mtd.h>

static struct spi_flash *spi_flash[CONFIG_SYS_MAX_FLASH_BANKS];
flash_info_t flash_info[CONFIG_SYS_MAX_FLASH_BANKS];
static struct mtd_info mtd_info[CONFIG_SYS_MAX_FLASH_BANKS];
static char mtd_name[CONFIG_SYS_MAX_FLASH_BANKS][6]; //"nor99\0"

/* Helper for flash_init, fills in all the structs. U-Boot's MTD layers have too
 * many different interfaces! Ret: size, 0=error. Will spi_flash_free() in case
 * of error. Does check if spi==NULL or if idx was already used. */
static unsigned long add(struct spi_flash *spi, unsigned idx)
{
	u32 size, sector_size, sector_count;
	ulong start;
	int i;

	if (!spi)
		return 0;
	if (idx>=CONFIG_SYS_MAX_FLASH_BANKS || spi_flash[idx]) {
		spi_flash_free(spi);
		return 0;
	}
	size = spi->size;
	sector_size = spi->sector_size;
	sector_count = size / sector_size;
	if (sector_count > CONFIG_SYS_MAX_FLASH_SECT) {
		error("CONFIG_SYS_MAX_FLASH_SECT too small, need %u!\n", sector_count);
		sector_count = CONFIG_SYS_MAX_FLASH_SECT; //cap at maximum
		size = sector_size * sector_count;
	}
	if (!(start = (ulong)spi->memory_map))
		start = CONFIG_SYS_FLASH_BASE; //we'll need to shadow
	spi_flash[idx] = spi;

	flash_info[idx].size = size;
	flash_info[idx].sector_count = sector_count;
	flash_info[idx].flash_id = idx; //don't have real ID, and we need idx later!
	for (i = 0; i < sector_count; i++) {
		flash_info[idx].start[i] = start;
		flash_info[idx].protect[i] = 1; //default r/o
		start += sector_size;
	}

	mtd_info[idx].type = MTD_NORFLASH;
	mtd_info[idx].flags = MTD_CAP_NORFLASH;
	mtd_info[idx].size = size;
	mtd_info[idx].erasesize = sector_size;
	mtd_info[idx].writesize = 1; //not spi->page_size
	sprintf(mtd_name[idx], "nor%u", idx);
	mtd_info[idx].name = mtd_name[idx];
	mtd_info[idx].priv = &flash_info[idx];

	if (add_mtd_device(&mtd_info[idx])) {
		spi_flash_free(spi);
		spi_flash[idx] = NULL;
		flash_info[idx].size = 0;
		flash_info[idx].sector_count = 0;
		return 0;
	}
	printf("%s, ", spi->name);
	return size;
}

/* Detect SPI flash chips. Called once, early in U-Boot init. */
unsigned long flash_init(void)
{
	struct spi_flash *spi;
	unsigned long total_size = 0;

	spi = spi_flash_probe_silent(CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS,
	                             CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE);
	total_size += add(spi, 0);

	// ... add further devices here

	if (total_size)
		puts("total ");
	return total_size;
}

/* Called before reading from an partition to allow shadowing it in RAM.
 * offset, size are the extents of the partition. Ret: 0=ok. */
int flash_shadow_spi(flash_info_t *info, u32 offset, u32 size)
{
	static flash_info_t *sh_info = NULL;
	static u32 sh_offset, sh_size;
	struct spi_flash *spi = spi_flash[info->flash_id];
	int ret;

	if (spi->memory_map || !size)
		return 0; //no shadow necessary
	if (info==sh_info && offset>=sh_offset && offset+size<=sh_offset+sh_size)
		return 0; //we have all we need
	if (info==sh_info && offset<sh_offset
	    && offset+size>=sh_offset && offset+size<=sh_offset+sh_size) {
		size = sh_offset - offset; //extend to the left
		sh_offset = offset; sh_size += size;
	} else if (info==sh_info && offset+size>sh_offset+sh_size
	           && offset>=sh_offset && offset<=sh_offset+sh_size) {
		size -= sh_offset + sh_size - offset; //extend to the right
		offset = sh_offset + sh_size;
		sh_size += size;
	} else { // Different device or discontiguous range. The 'extends both
	         // left+right' case can only occur when repartitioning, never
	         // when just 'chpart'ing, so screw that!
		sh_info = info;
		sh_offset = offset; sh_size = size; //just shadow everything
	}
	ret = spi_flash_read(spi, offset, size, (void*)info->start[0]+offset);
	if (ret)
		sh_info = NULL; //:-(
	return ret;
}

/* Called whenever the flash 'mem mapped' address (flash_info[].start[]) is
 * written to, e.g. with "cp" command. SPI isn't really mem mapped, only
 * shadowed to make CRAMFS work, but the MTD layer assumes mem mapping. */
int write_buff(flash_info_t *info, uchar *src, ulong addr, ulong cnt)
{
	struct spi_flash *spi = spi_flash[info->flash_id];
	int ret;

	if (!(ret = spi_flash_write(spi, addr - info->start[0], cnt, src))
	    && !spi->memory_map)
		memcpy((void*)addr, src, cnt); //update shadow
	return ret;
}

#ifdef CONFIG_CMD_FLASH
/* Called by "erase" command. */
int flash_erase(flash_info_t *info, int s_first, int s_last)
{
	struct spi_flash *spi = spi_flash[info->flash_id];
	u32 ofs = s_first * spi->sector_size;
	size_t len = (s_last-s_first+1) * spi->sector_size;
	int ret;

	if (!(ret = spi_flash_erase(spi, ofs, len))
	    && !spi->memory_map)
		memset((void*)info->start[s_first], 0xFF, len); //update shadow
	puts(ret? "... failed\n": "... done\n");
	return ret;
}

/* Called by "flinfo" command. */
void flash_print_info(flash_info_t *info)
{
	unsigned idx = info->flash_id;
	struct spi_flash *spi = spi_flash[idx];

	printf("%s: %s ", mtd_name[idx], spi->name);
	print_size(info->size, " SPI flash ");
	printf("%sed @ 0x%08lX, ", spi->memory_map? "mapp": "shadow", info->start[0]);
	print_size(spi->sector_size, " erase size\n");
}
#endif
