/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011 Mike Smith <drziplok@me.com>
 * Copyright (C) 2016 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"
#include "lpc_common.h"

#define IAP_PGM_CHUNKSIZE 512 /* should fit in RAM on any device */

#define MIN_RAM_SIZE               1024
#define RAM_USAGE_FOR_IAP_ROUTINES 32 /* IAP routines use 32 bytes at top of ram */

#define IAP_ENTRY_MOST 0x1fff1ff1 /* all except LPC802, LPC804 & LPC84x */
#define IAP_ENTRY_84x  0x0f001ff1 /* LPC802, LPC804 & LPC84x */
#define IAP_RAM_BASE   0x10000000

#define LPC11XX_DEVICE_ID 0x400483F4
#define LPC8XX_DEVICE_ID  0x400483F8

/*
 * CHIP    Ram Flash page sector   Rsvd pages  EEPROM
 * LPX80x   2k   16k   64   1024            2
 * LPC804   4k   32k   64   1024            2
 * LPC8N04  8k   32k   64   1024           32
 * LPC810   1k    4k   64   1024            0
 * LPC811   2k    8k   64   1024            0
 * LPC812   4k   16k   64   1024
 * LPC822   4k   16k   64   1024
 * LPC822   8k   32k   64   1024
 * LPC832   4k   16k   64   1024
 * LPC834   4k   32k   64   1024
 * LPC844   8k   64k   64   1024
 * LPC845  16k   64k   64   1024
 */

static bool lpc11xx_read_uid(target *t, int argc, const char **argv);

const struct command_s lpc11xx_cmd_list[] = {
	{"readuid", lpc11xx_read_uid, "Read out the 16-byte UID."},
	{NULL, NULL, NULL},
};

static void lpc11xx_add_flash(target *t, const uint32_t addr, const size_t len, const size_t erase_block_len,
	const uint32_t iap_entry, const size_t reserved_pages)
{
	struct lpc_flash *lf = lpc_add_flash(t, addr, len);
	lf->f.blocksize = erase_block_len;
	lf->f.writesize = IAP_PGM_CHUNKSIZE;
	lf->f.write = lpc_flash_write_magic_vect;
	lf->iap_entry = iap_entry;
	lf->iap_ram = IAP_RAM_BASE;
	lf->iap_msp = IAP_RAM_BASE + MIN_RAM_SIZE - RAM_USAGE_FOR_IAP_ROUTINES;
	lf->reserved_pages = reserved_pages;
}

bool lpc11xx_probe(target *t)
{
	/* read the device ID register */
	/* For LPC11xx & LPC11Cxx see UM10398 Rev. 12.4 Chapter 26.5.11 Table 387
	 * For LPC11Uxx see UM10462 Rev. 5.5 Chapter 20.13.11 Table 377
	 * Nota Bene: the DEVICE_ID register at address 0x400483F4 is not valid
	 * for:
	 *   1) the LPC11xx & LPC11Cxx "XL" series, see UM10398 Rev.12.4 Chapter 3.1
	 *   2) the LPC11U3x series, see UM10462 Rev.5.5 Chapter 3.1
	 * But see the comment for the LPC8xx series below.
	 */
	uint32_t device_id = target_mem_read32(t, LPC11XX_DEVICE_ID);

	switch (device_id) {
	case 0x0A07102B: /* LPC1110 - 4K Flash 1K SRAM */
	case 0x1A07102B: /* LPC1110 - 4K Flash 1K SRAM */
	case 0x0A16D02B: /* LPC1111/002 - 8K Flash 2K SRAM */
	case 0x1A16D02B: /* LPC1111/002 - 8K Flash 2K SRAM */
	case 0x041E502B: /* LPC1111/101 - 8K Flash 2K SRAM */
	case 0x2516D02B: /* LPC1111/101/102 - 8K Flash 2K SRAM */
	case 0x0416502B: /* LPC1111/201 - 8K Flash 4K SRAM */
	case 0x2516902B: /* LPC1111/201/202 - 8K Flash 4K SRAM */
	case 0x0A23902B: /* LPC1112/102 - 16K Flash 4K SRAM */
	case 0x1A23902B: /* LPC1112/102 - 16K Flash 4K SRAM */
	case 0x042D502B: /* LPC1112/101 - 16K Flash 2K SRAM */
	case 0x2524D02B: /* LPC1112/101/102 - 16K Flash 2K SRAM */
	case 0x0425502B: /* LPC1112/201 - 16K Flash 4K SRAM */
	case 0x2524902B: /* LPC1112/201/202 - 16K Flash 4K SRAM */
	case 0x0434502B: /* LPC1113/201 - 24K Flash 4K SRAM */
	case 0x2532902B: /* LPC1113/201/202 - 24K Flash 4K SRAM */
	case 0x0434102B: /* LPC1113/301 - 24K Flash 8K SRAM */
	case 0x2532102B: /* LPC1113/301/302 - 24K Flash 8K SRAM */
	case 0x0A40902B: /* LPC1114/102 - 32K Flash 4K SRAM */
	case 0x1A40902B: /* LPC1114/102 - 32K Flash 4K SRAM */
	case 0x0444502B: /* LPC1114/201 - 32K Flash 4K SRAM */
	case 0x2540902B: /* LPC1114/201/202 - 32K Flash 4K SRAM */
	case 0x0444102B: /* LPC1114/301 - 32K Flash 8K SRAM */
	case 0x2540102B: /* LPC1114/301/302 & LPC11D14/302 - 32K Flash 8K SRAM */
	case 0x00050080: /* LPC1115/303 - 64K Flash 8K SRAM (redundant? see UM10398, XL has Device ID at different address) */
	case 0x1421102B: /* LPC11c12/301 - 16K Flash 8K SRAM */
	case 0x1440102B: /* LPC11c14/301 - 32K Flash 8K SRAM */
	case 0x1431102B: /* LPC11c22/301 - 16K Flash 8K SRAM */
	case 0x1430102B: /* LPC11c24/301 - 32K Flash 8K SRAM */
	case 0x095C802B: /* LPC11u12x/201 - 16K Flash 4K SRAM */
	case 0x295C802B: /* LPC11u12x/201 - 16K Flash 4K SRAM */
	case 0x097A802B: /* LPC11u13/201 - 24K Flash 4K SRAM */
	case 0x297A802B: /* LPC11u13/201 - 24K Flash 4K SRAM */
	case 0x0998802B: /* LPC11u14/201 - 32K Flash 4K SRAM */
	case 0x2998802B: /* LPC11u14/201 - 32K Flash 4K SRAM */
	case 0x2954402B: /* LPC11u22/301 - 16K Flash 6K SRAM */
	case 0x2972402B: /* LPC11u23/301 - 24K Flash 6K SRAM */
	case 0x2988402B: /* LPC11u24x/301 - 32K Flash 6K SRAM */
	case 0x2980002B: /* LPC11u24x/401 - 32K Flash 8K SRAM */
		t->driver = "LPC11xx";
		target_add_ram(t, 0x10000000, 0x2000);
		lpc11xx_add_flash(t, 0x00000000, 0x20000, 0x1000, IAP_ENTRY_MOST, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC11xx");
		return true;

	case 0x0A24902B:
	case 0x1A24902B:
		t->driver = "LPC1112";
		target_add_ram(t, 0x10000000, 0x1000);
		lpc11xx_add_flash(t, 0x00000000, 0x10000, 0x1000, IAP_ENTRY_MOST, 0);
		return true;
	case 0x1000002b: // FX LPC11U6 32 kB SRAM/256 kB flash (max)
		t->driver = "LPC11U6";
		target_add_ram(t, 0x10000000, 0x8000);
		lpc11xx_add_flash(t, 0x00000000, 0x40000, 0x1000, IAP_ENTRY_MOST, 0);
		return true;
	case 0x3000002B:
	case 0x3D00002B:
		t->driver = "LPC1343";
		target_add_ram(t, 0x10000000, 0x2000);
		lpc11xx_add_flash(t, 0x00000000, 0x8000, 0x1000, IAP_ENTRY_MOST, 0);
		return true;
	case 0x00008A04: /* LPC8N04 (see UM11074 Rev.1.3 section 4.5.19) */
		t->driver = "LPC8N04";
		target_add_ram(t, 0x10000000, 0x2000);
		/* UM11074/ Flash controller/15.2: The two topmost sectors
		 * contain the initialization code and IAP firmware.
		 * Do not touch them! */
		lpc11xx_add_flash(t, 0x00000000, 0x7800, 0x400, IAP_ENTRY_MOST, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC8N04");
		return true;
	}
	if ((t->designer_code != JEP106_MANUFACTURER_SPECULAR) && device_id) {
		DEBUG_INFO("LPC11xx: Unknown Device ID 0x%08" PRIx32 "\n", device_id);
	}
	/* For LPC802, see UM11045 Rev. 1.4 Chapter 6.6.29 Table 84
	 * For LPC804, see UM11065 Rev. 1.0 Chapter 6.6.31 Table 87
	 * For LPC81x, see UM10601 Rev. 1.6 Chapter 4.6.33 Table 50
	 * For LPC82x, see UM10800 Rev. 1.2 Chapter 5.6.34 Table 55
	 * For LPC83x, see UM11021 Rev. 1.1 Chapter 5.6.34 Table 53
	 * For LPC84x, see UM11029 Rev. 1.4 Chapter 8.6.49 Table 174
	 *
	 * Not documented, but the DEVICE_ID register at address 0x400483F8
	 * for the LPC8xx series is also valid for the LPC11xx "XL" and the
	 * LPC11U3x variants.
	 */
	device_id = target_mem_read32(t, LPC8XX_DEVICE_ID);
	switch (device_id) {
	case 0x00008021: /* LPC802M001JDH20 - 16K Flash 2K SRAM */
	case 0x00008022: /* LPC802M011JDH20 */
	case 0x00008023: /* LPC802M001JDH16 */
	case 0x00008024: /* LPC802M001JHI33 */
		t->driver = "LPC802";
		target_add_ram(t, 0x10000000, 0x800);
		lpc11xx_add_flash(t, 0x00000000, 0x4000, 0x400, IAP_ENTRY_84x, 2);
		target_add_commands(t, lpc11xx_cmd_list, "LPC802");
		return true;
	case 0x00008040: /* LPC804M101JBD64 - 32K Flash 4K SRAM */
	case 0x00008041: /* LPC804M101JDH20 */
	case 0x00008042: /* LPC804M101JDH24 */
	case 0x00008043: /* LPC804M111JDH24 */
	case 0x00008044: /* LPC804M101JHI33 */
		t->driver = "LPC804";
		target_add_ram(t, 0x10000000, 0x1000);
		lpc11xx_add_flash(t, 0x00000000, 0x8000, 0x400, IAP_ENTRY_84x, 2);
		target_add_commands(t, lpc11xx_cmd_list, "LPC804");
		return true;
	case 0x00008100: /* LPC810M021FN8 - 4K Flash 1K SRAM */
	case 0x00008110: /* LPC811M001JDH16 - 8K Flash 2K SRAM */
	case 0x00008120: /* LPC812M101JDH16 - 16K Flash 4K SRAM */
	case 0x00008121: /* LPC812M101JD20 - 16K Flash 4K SRAM */
	case 0x00008122: /* LPC812M101JDH20 / LPC812M101JTB16 - 16K Flash 4K SRAM */
		t->driver = "LPC81x";
		target_add_ram(t, 0x10000000, 0x1000);
		lpc11xx_add_flash(t, 0x00000000, 0x4000, 0x400, IAP_ENTRY_MOST, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC81x");
		return true;
	case 0x00008221: /* LPC822M101JHI33 - 16K Flash 4K SRAM */
	case 0x00008222: /* LPC822M101JDH20 */
	case 0x00008241: /* LPC824M201JHI33 - 32K Flash 8K SRAM */
	case 0x00008242: /* LPC824M201JDH20 */
		t->driver = "LPC82x";
		target_add_ram(t, 0x10000000, 0x2000);
		lpc11xx_add_flash(t, 0x00000000, 0x8000, 0x400, IAP_ENTRY_MOST, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC82x");
		return true;
	case 0x00008322: /* LPC832M101FDH20 - 16K Flash 4K SRAM */
		t->driver = "LPC832";
		target_add_ram(t, 0x10000000, 0x1000);
		lpc11xx_add_flash(t, 0x00000000, 0x4000, 0x400, IAP_ENTRY_MOST, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC832");
		return true;
	case 0x00008341: /* LPC834M101FHI33 - 32K Flash 4K SRAM */
		t->driver = "LPC834";
		target_add_ram(t, 0x10000000, 0x1000);
		lpc11xx_add_flash(t, 0x00000000, 0x8000, 0x400, IAP_ENTRY_MOST, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC834");
		return true;
	case 0x00008441: /* LPC844M201JBD64 - 64K Flash 8K SRAM */
	case 0x00008442: /* LPC844M201JBD48 */
	case 0x00008443: /* LPC844M201JHI48, note UM11029 Rev.1.4 table 29 is wrong, see table 174 (in same manual) */
	case 0x00008444: /* LPC844M201JHI33 */
		t->driver = "LPC844";
		target_add_ram(t, 0x10000000, 0x2000);
		lpc11xx_add_flash(t, 0x00000000, 0x10000, 0x400, IAP_ENTRY_84x, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC844");
		return true;
	case 0x00008451: /* LPC845M301JBD64 - 64K Flash 16K SRAM */
	case 0x00008452: /* LPC845M301JBD48 */
	case 0x00008453: /* LPC845M301JHI48 */
	case 0x00008454: /* LPC845M301JHI33 */
		t->driver = "LPC845";
		target_add_ram(t, 0x10000000, 0x4000);
		lpc11xx_add_flash(t, 0x00000000, 0x10000, 0x400, IAP_ENTRY_84x, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC845");
		return true;
	case 0x0003D440: /* LPC11U34/311 - 40K Flash 8K SRAM */
	case 0x0001cc40: /* LPC11U34/421 - 48K Flash 8K SRAM */
	case 0x0001BC40: /* LPC11U35/401 - 64K Flash 8K SRAM */
	case 0x0000BC40: /* LPC11U35/501 - 64K Flash 8K SRAM */
	case 0x00019C40: /* LPC11U36/401 - 96K Flash 8K SRAM */
	case 0x00017C40: /* LPC11U37FBD48/401 - 128K Flash 8K SRAM */
	case 0x00007C44: /* LPC11U37HFBD64/401 */
	case 0x00007C40: /* LPC11U37FBD64/501 */
		t->driver = "LPC11U3x";
		target_add_ram(t, 0x10000000, 0x2000);
		lpc11xx_add_flash(t, 0x00000000, 0x20000, 0x1000, IAP_ENTRY_MOST, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC11U3x");
		return true;
	case 0x00010013: /* LPC1111/103 - 8K Flash 2K SRAM */
	case 0x00010012: /* LPC1111/203 - 8K Flash 4K SRAM */
	case 0x00020023: /* LPC1112/103 - 16K Flash 2K SRAM */
	case 0x00020022: /* LPC1112/203 - 16K Flash 4K SRAM */
	case 0x00030030: /* LPC1113/303 - 24K Flash 8K SRAM */
	case 0x00030032: /* LPC1113/203 - 24K Flash 4K SRAM */
	case 0x00040040: /* LPC1114/303 - 32K Flash 8K SRAM */
	case 0x00040042: /* LPC1114/203 - 32K Flash 4K SRAM */
	case 0x00040060: /* LPC1114/323 - 48K Flash 8K SRAM */
	case 0x00040070: /* LPC1114/333 - 56K Flash 8K SRAM */
	case 0x00050080: /* LPC1115/303 - 64K Flash 8K SRAM */
		t->driver = "LPC11xx-XL";
		target_add_ram(t, 0x10000000, 0x2000);
		lpc11xx_add_flash(t, 0x00000000, 0x20000, 0x1000, IAP_ENTRY_MOST, 0);
		target_add_commands(t, lpc11xx_cmd_list, "LPC11xx-XL");
		return true;
	}
	if (device_id) {
		DEBUG_INFO("LPC8xx: Unknown Device ID 0x%08" PRIx32 "\n", device_id);
	}

	return false;
}

static bool lpc11xx_read_uid(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	struct lpc_flash *f = (struct lpc_flash *)t->flash;
	uint8_t uid[16];
	if (lpc_iap_call(f, uid, IAP_CMD_READUID))
		return false;
	tc_printf(t, "UID: 0x");
	for (size_t i = 0; i < sizeof(uid); ++i)
		tc_printf(t, "%02x", uid[i]);
	tc_printf(t, "\n");
	return true;
}
