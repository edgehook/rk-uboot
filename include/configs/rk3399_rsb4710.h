/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __RK3399_RSB4710_H
#define __RK3399_RSB4710_H

#include <configs/rk3399_common.h>

#ifndef CONFIG_SPL_BUILD
#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND RKIMG_BOOTCOMMAND
#endif

#define CONFIG_MMC_SDHCI_SDMA
#define CONFIG_SYS_MMC_ENV_DEV 0

#define SDRAM_BANK_SIZE			(2UL << 30)
#define CONFIG_MISC_INIT_R
#define CONFIG_SERIAL_TAG
#define CONFIG_ENV_OVERWRITE

#define CONFIG_HAS_ETH1
#define FDT_SEQ_MACADDR_FROM_ENV

#define CONFIG_BMP_16BPP
#define CONFIG_BMP_24BPP
#define CONFIG_BMP_32BPP

#define ROCKCHIP_DEVICE_SETTINGS \
		"stdout=serial,vidconsole\0" \
		"stderr=serial,vidconsole\0"

#ifdef CONFIG_DM_CHARGE_DISPLAY
/*#undef CONFIG_DM_CHARGE_DISPLAY*/
#endif

#ifdef CONFIG_USING_KERNEL_DTB
#undef CONFIG_USING_KERNEL_DTB
#endif

#define CONFIG_BOARD_EARLY_INIT_F

#endif
