/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015 Elesar Ltd. - http://www.elesar.co.uk
 *
 * Based on include/configs/am57xx_evm.h.
 *
 * (C) Copyright 2014 Texas Instruments Incorporated.
 *
 * Configuration settings for the Elesar Titanium board.
 * See ti_omap5_common.h for omap5 common settings.
 */

#ifndef __CONFIG_ELESAR_TITANIUM_H
#define __CONFIG_ELESAR_TITANIUM_H

#include <linux/sizes.h>

#define CONFIG_IODELAY_RECALIBRATION

#define CONSOLEDEV			"ttyS0"
#define CONFIG_SYS_NS16550_COM1		UART1_BASE	/* Titanium has UART1 */

/* ABE has sys_clk as source */
#define CONFIG_SYS_OMAP_ABE_SYSCK

/* Required for I2C access to the TFP410 to work */
#define CONFIG_I2C_REPEATED_START

#include <configs/ti_omap5_common.h>

/* Enhance our eMMC support / experience. */
#define CONFIG_HSMMC2_8BIT

/* CPSW Ethernet */
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_NET_RETRY_COUNT	10
#define PHY_ANEG_TIMEOUT	8000	/* PHY needs longer aneg time at 1G */

/* USB xHCI HOST */
#if 0
#define CONFIG_USB_XHCI_OMAP
#define CONFIG_OMAP_USB3PHY1_HOST
#endif

/* SATA */
#if 0
#define CONFIG_SCSI_AHCI_PLAT
#define CONFIG_SYS_SCSI_MAX_SCSI_ID	1
#define CONFIG_SYS_SCSI_MAX_LUN		1
#define CONFIG_SYS_SCSI_MAX_DEVICE	(CONFIG_SYS_SCSI_MAX_SCSI_ID * \
						CONFIG_SYS_SCSI_MAX_LUN)
#endif

/*
 * Default to using SPI for environment, etc.
 * 0x000000 - 0x040000 : QSPI.SPL (256KiB)
 * 0x040000 - 0x140000 : QSPI.u-boot (1MiB)
 * 0x140000 - 0x1C0000 : QSPI.u-boot-spl-os (512KiB)
 * 0x1C0000 - 0x1D0000 : QSPI.u-boot-env (64KiB)
 * 0x1D0000 - 0x1E0000 : QSPI.u-boot-env.backup1 (64KiB)
 * 0x1E0000 - 0x800000 : QSPI.data
 */
#define CONFIG_SYS_SPI_KERNEL_OFFS	0x1E0000
#define CONFIG_SYS_SPI_ARGS_OFFS	0x140000
#define CONFIG_SYS_SPI_ARGS_SIZE	0x80000

#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_SIZE			SZ_64K
#define CONFIG_ENV_SECT_SIZE		SZ_64K
#define CONFIG_ENV_OFFSET		0x1C0000
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)

/* SPI SPL */
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x40000

#endif /* __CONFIG_ELESAR_TITANIUM_H */
