/*
 * (C) Copyright 2013-2014 STMicroelectronics
 * Youssef TRIKI <youssef.triki@st.com>
 * Sean McGoogan <Sean.McGoogan@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <common.h>
#include <malloc.h>
#include <sdhci.h>
#include <stm/soc.h>
#include <stm/sdhci.h>


	/*
	 * Pointer to SoC-specific mmc_getcd() function.
	 */
typedef int (*const stm_getcd_fn)(const int port);
static stm_getcd_fn getcd =
	stm_mmc_getcd;


static char * const sdhci_name[] =
{
	"stm-sdhci0",	/* MMC #0 */
	"stm-sdhci1",	/* MMC #1 */
	"stm-sdhci2",	/* MMC #2 */
	"stm-sdhci3",	/* MMC #3 */
};

extern int board_mmc_getcd(struct mmc * const mmc)
{
	if (!strcmp(mmc->cfg->name,sdhci_name[0]))
	{
		return !(*getcd)(0);	/* MMC #0 */
	}
	else if (!strcmp(mmc->cfg->name,sdhci_name[1]))
	{
		return !(*getcd)(1);	/* MMC #1 */
	}
	else if (!strcmp(mmc->cfg->name,sdhci_name[2]))
	{
		return !(*getcd)(2);	/* MMC #2 */
	}
	else if (!strcmp(mmc->cfg->name,sdhci_name[3]))
	{
		return !(*getcd)(3);	/* MMC #3 */
	}
	else
	{
		BUG();		/* should never get here! */
	}
}

extern int stm_sdhci_init(const int port, struct stm_sdhci_info *info)
{
	struct sdhci_host *host;

	/* paranoia */
	BUG_ON(port >= ARRAY_SIZE(sdhci_name));

	host = (struct sdhci_host *)malloc(sizeof(struct sdhci_host));
	if (!host)
	{
		printf("stm_sdhci_init() malloc fail for host structure!\n");
		return -1;	/* return an error */
	}
		/* fill in the newly allocated host structure */
	memset(host, 0, sizeof(struct sdhci_host));
	host->name      = sdhci_name[port];
	host->ioaddr    = info->regbase;
	host->quirks    = info->quirks;
	host->version   = sdhci_readw(host, SDHCI_HOST_VERSION);
	host->host_caps = info->host_caps;

	return add_sdhci(host, info->max_speed, info->min_speed);
}
