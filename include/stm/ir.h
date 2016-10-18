/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Amitabh Dutta <amitabh.dutta@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 * IR driver config
 */

#ifndef __IR_CONFIG_H__
#define __IR_CONFIG_H__

/* Choose if we want IR Remote Control Support */
#define CONFIG_STM_IR
#ifdef CONFIG_STM_IR
#      define CONFIG_CMD_IR

#ifdef CONFIG_STM_STXH410
#      define CONFIG_SYS_STM_IRB_BASE          0x09518000
#endif

#endif

#endif /*end of __IR_CONFIG_H__ */
