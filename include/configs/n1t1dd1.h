/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef _CONFIG_N1T1DD1_H
#define _CONFIG_N1T1DD1_H

/*
 * mv-common.h should be defined after CMD configs since it used them
 * to enable certain macros
 */
#include "mv-common.h"

/*
 * Ethernet Driver configuration
 */
#ifdef CONFIG_CMD_NET
#define CONFIG_MVGBE_PORTS	{1, 0}	/* enable port 0 only */
#define CONFIG_PHY_BASE_ADR	8
#endif /* CONFIG_CMD_NET */

#endif /* _CONFIG_N1T1DD1_H */
