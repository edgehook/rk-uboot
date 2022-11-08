/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef __RK3399_ITB201_H
#define __RK3399_ITB201_H

#include <configs/rk3399_common.h>

#ifndef CONFIG_SPL_BUILD
#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND RKIMG_BOOTCOMMAND
#endif

/*#define CONFIG_ADV_SYSTEM_BACKUP*/

#define CONFIG_MMC_SDHCI_SDMA
#define CONFIG_SYS_MMC_ENV_DEV 0

#define SDRAM_BANK_SIZE			(2UL << 30)
#define CONFIG_MISC_INIT_R
#define CONFIG_SERIAL_TAG
#define CONFIG_ENV_OVERWRITE

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

#ifdef CONFIG_ENV_SIZE
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_SIZE 0x1000
#endif

#ifdef CONFIG_ROCKCHIP_VENDOR_PARTITION
#undef CONFIG_ROCKCHIP_VENDOR_PARTITION
#endif

#define CONFIG_BOARD_EARLY_INIT_F
#define ENABLE_PCIERST_GPIO

#define AMP_MUTE_GPIO	126
#define M2B1_SEL_GPIO   153 //GPIO4_D1
#define M2B1_GNSS_DISABLE_GPIO  10 //GPIO0_B2
#define M2B1_DISABLE_GPIO  11 //GPIO0_B3

#endif
