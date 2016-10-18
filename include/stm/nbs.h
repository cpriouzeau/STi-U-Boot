/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

/*----------------------------------------------------------------------------*/
#define CONFIG_SKIP_TZL

#define BLIT_BASE_ADDRESS             STM_BLIT_BASE_ADDRESS
#define DMU_BASE_ADDRESS              STM_DMU_BASE_ADDRESS
#define ESRAM_SECURE_BASE_ADDRESS     STM_ESRAM_SECURE_BASE_ADDRESS
#define HVA_BASE_ADDRESS              STM_HVA_BASE_ADDRESS

#ifndef CONFIG_SKIP_TZL
#ifndef TZL_CODE_ADDRESS /* workaround value, esram issue with official value */
#define TZL_CODE_ADDRESS            (BLIT_BASE_ADDRESS + 0x7000)
#endif

#ifndef TZL_PTR_NEXT_APP /* at this address TZloader finds address of next application (U-Boot in our case) */
#define TZL_PTR_NEXT_APP            (ESRAM_SECURE_BASE_ADDRESS+ 0x20000 + 0x20)
#endif
#endif

#ifndef BFR_DATA_ADDRESS   /* from CAF Config documentation: Offset in SCS Params structure */
#define BFR_DATA_ADDRESS            (HVA_BASE_ADDRESS)
#endif
#ifndef BFR_CODE_ADDRESS   /* from CAF Config documentation : Offset in Auxiliary structure */
#define BFR_CODE_ADDRESS            (HVA_BASE_ADDRESS+0x2000)
#endif
#ifndef BFR_GLOBAL_VAR_SIZE
#define BFR_GLOBAL_VAR_SIZE (4*1024 )
#endif
#ifndef BFR_GLOBAL_VAR_ADDRESS     /* from CAF Config documentation : area for BFR C global variables, only not statically initialization (i.e. initialized at definition) are permitted */
#define BFR_GLOBAL_VAR_ADDRESS      (HVA_BASE_ADDRESS + 0x36fD0 - BFR_GLOBAL_VAR_SIZE )
#endif
#ifndef BFR_STACK_TOP_ADDRESS     /* from CAF Config documentation (stack shall not decrease below 0x6030000) */
#define BFR_STACK_TOP_ADDRESS       (BFR_GLOBAL_VAR_ADDRESS - 0x10)
#endif
/*----------------------------------------------------------------------------*/


