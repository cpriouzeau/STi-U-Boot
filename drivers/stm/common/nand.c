/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#include <common.h>
#include <stm/soc.h>

extern void
stm_configure_nand (void)
{
  /*
   * We will set up the PIO pins correctly for NAND
   *
   * We want to use NAND on the FlashSS IP block
   *
   * With the FlashSS, there is no need for software
   * to configure the OE (output-enable) or the OD
   * (open-drain) control lines, as the FlashSS IP
   * (the NANDi) controller will take care of them,
   * as long as the alternate-function is right.
   * The PU/PD are S/W controllable, but are correct
   * following a reset, so again, we should be able
   * to rely on them being correct for U-Boot.
   * So, we only need to set the alternate-functions!
   *
   * Route PIO for NAND (alternate #3 in FlashSS)
   */
//  stm_pioalt_select (40, 4, 3);	/* NAND_CSn2 */
//  stm_pioalt_select (40, 5, 3);	/* NAND_CSn3 */
//  stm_pioalt_select (40, 6, 3);	/* NAND_CSn1 */

  stm_pioalt_select (40, 7, 3);	/* NAND_CSn0 */

  stm_pioalt_select (41, 0, 3);	/* NAND_DQ0 */
  stm_pioalt_select (41, 1, 3);	/* NAND_DQ1 */
  stm_pioalt_select (41, 2, 3);	/* NAND_DQ2 */
  stm_pioalt_select (41, 3, 3);	/* NAND_DQ3 */
  stm_pioalt_select (41, 4, 3);	/* NAND_DQ4 */
  stm_pioalt_select (41, 5, 3);	/* NAND_DQ5 */
  stm_pioalt_select (41, 6, 3);	/* NAND_DQ6 */
  stm_pioalt_select (41, 7, 3);	/* NAND_DQ7 */

  stm_pioalt_select (42, 0, 3);	/* NAND_WEn */
  stm_pioalt_select (42, 2, 3);	/* NAND_ALE */
  stm_pioalt_select (42, 3, 3);	/* NAND_CLE */
  stm_pioalt_select (42, 4, 3);	/* NAND_RnB */
  stm_pioalt_select (42, 5, 3);	/* NAND_REn */
}
