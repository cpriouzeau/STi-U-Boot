/*
 * (C) Copyright 2006
 * Pierre Morel, WYPLAY, pmorel@wyplay.com
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

#include <common.h>
#include <command.h>

extern int do_st_memory_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);

int do_st_mtest(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong i, count;

	if (argc !=4)
		return cmd_usage(cmdtp);	/* bad command usage */

	count = simple_strtoul(argv[3], NULL, 10);

	for (;;) {
		do_st_memory_test(NULL, 0, 3, argv);

		printf("Type CTRL-C to Abort...");
		/* delay for <count> ms... */
		for (i = 0; i < count; i++)
		{
			udelay(1000);	/* 1 ms */
			/* check for ctrl-c to abort... */
			if (ctrlc()) {
				puts("Abort\n");
				return 0;
			}
		}
	}

	return 0;
}

U_BOOT_CMD(
	st_mtest, 4, 1, do_st_mtest,
	"STMicro Memory tests",
	"start length delay - STMicro memory tests\n"
	"      all values in decimal (prepend '0x' for hex)\n"
	"      delay specified in ms"
);
