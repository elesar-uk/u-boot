/*
 * Copyright (C) 2015 Elesar - http://www.elesar.co.uk
 *
 * Based on board/ti/beagle_x15/mux_data.h
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _MUX_DATA_ELESAR_TITANIUM_H_
#define _MUX_DATA_ELESAR_TITANIUM_H_

#include <asm/arch/mux_dra7xx.h>

#define GPIO_OTP_CLOCK	((0 << 5) + 6)	/* GPIO1_6 */
#define GPIO_OTP_DATA	((0 << 5) + 7)	/* GPIO1_7 */
#define GPIO_MMC_PWROFF	((6 << 5) + 0)	/* GPIO7_0 */
#define GPIO_SOFTOFF	((6 << 5) + 7)	/* GPIO7_7 */

const struct pad_conf_entry core_padconf_array_essential[] = {
	{GPMC_AD0, (M14 | PIN_OUTPUT)},		/* gpmc_ad0.gpio1_6 (OTP_CLOCK) */
	{GPMC_AD1, (M14 | PIN_INPUT)},		/* gpmc_ad0.gpio1_7 (OTP_DATA) */

	{GPMC_A0, (M7 | PIN_INPUT)},		/* gpmc_a0.i2c4_scl (IIC4_7_SCL) */
	{GPMC_A1, (M7 | PIN_INPUT)},		/* gpmc_a1.i2c4_sda (IIC4_7_SDA) */

	{GPMC_A13, (M1 | PIN_INPUT)},		/* gpmc_a13.qspi1_rtclk (QSPI_1_RTCLK) */
	{GPMC_A14, (M1 | PIN_INPUT)},		/* gpmc_a14.qspi1_d3 (QSPI_1_D3) */
	{GPMC_A15, (M1 | PIN_INPUT)},		/* gpmc_a15.qspi1_d2 (QSPI_1_D2) */
	{GPMC_A16, (M1 | PIN_INPUT)},		/* gpmc_a16.qspi1_d1 (QSPI_1_D1) */
	{GPMC_A17, (M1 | PIN_INPUT)},		/* gpmc_a17.qspi1_d0 (QSPI_1_D0) */
	{GPMC_A18, (M1 | PIN_OUTPUT)},		/* gpmc_a18.qspi1_sclk (QSPI_1_SCLK) */
	{GPMC_CS2, (M1 | PIN_OUTPUT)},		/* gpmc_cs2.qspi1_cs0 (QSPI_1_nCS) */

	{GPMC_A26, (M14 | PIN_INPUT_PULLUP)},	/* gpmc_a26.gpio2_16 (NET1_nINT) */
	{GPMC_A27, (M14 | PIN_INPUT_PULLUP)},	/* gpmc_a27.gpio2_17 (NET2_nINT) */

	{GPMC_WAIT0, (M14 | PIN_INPUT)},	/* gpmc_wait0.gpio2_28 (CMOS lock) */

	{VIN2A_FLD0, (M14 | PIN_INPUT)},	/* vin2a_fld0.gpio3_30 (VID1_E_HOTP) */
	{VIN2A_HSYNC0, (M14 | PIN_INPUT)},	/* vin2a_hsync0.gpio3_31 (VID2_E_HOTP) */

	{VIN2A_VSYNC0, (M14 | PIN_INPUT_PULLUP)},	/* vin2a_vsync0.gpio4_0 (GPIO4_E_B0) */
	{VIN2A_D0, (M14 | PIN_INPUT_PULLUP)},		/* vin2a_d0.gpio4_1 (GPIO4_E_B1) */
	{VIN2A_D1, (M14 | PIN_INPUT_PULLUP)},		/* vin2a_d1.gpio4_2 (GPIO4_E_B2) */
	{VIN2A_D2, (M14 | PIN_INPUT_PULLUP)},		/* vin2a_d2.gpio4_3 (GPIO4_E_B3) */
	{VIN2A_D3, (M14 | PIN_INPUT_PULLUP)},		/* vin2a_d3.gpio4_4 (GPIO4_E_B4) */
	{VIN2A_D4, (M14 | PIN_INPUT_PULLUP)},		/* vin2a_d4.gpio4_5 (GPIO4_E_B5) */
	{VIN2A_D5, (M14 | PIN_INPUT_PULLUP)},		/* vin2a_d5.gpio4_6 (GPIO4_E_B6) */
	{VIN2A_D6, (M14 | PIN_INPUT_PULLUP)},		/* vin2a_d6.gpio4_7 (GPIO4_E_B7) */

	{VIN2A_D10, (M3 | PIN_OUTPUT)},		/* vin2a_d10.mdio_mclk (NET_3_MDC) */
	{VIN2A_D11, (M3 | PIN_INPUT)},		/* vin2a_d11.mdio_d (NET_3_MDIO) */

	{VIN2A_D12, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d12.rgmii1_txc (NET2_3_TXC) */
	{VIN2A_D13, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d13.rgmii1_txctl (NET2_3_TXCTL) */
	{VIN2A_D14, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d14.rgmii1_txd3 (NET2_3_TXD3) */
	{VIN2A_D15, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d15.rgmii1_txd2 (NET2_3_TXD2) */
	{VIN2A_D16, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d16.rgmii1_txd1 (NET2_3_TXD1) */
	{VIN2A_D17, (M3 | PIN_OUTPUT | MANUAL_MODE)},	/* vin2a_d17.rgmii1_txd0 (NET2_3_TXD0) */
	{VIN2A_D18, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d18.rgmii1_rxc (NET2_3_RXC) */
	{VIN2A_D19, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d19.rgmii1_rxctl (NET2_3_RXCTL) */
	{VIN2A_D20, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d20.rgmii1_rxd3 (NET2_3_RXD3) */
	{VIN2A_D21, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d21.rgmii1_rxd2 (NET2_3_RXD2) */
	{VIN2A_D22, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d22.rgmii1_rxd1 (NET2_3_RXD1) */
	{VIN2A_D23, (M3 | PIN_INPUT | MANUAL_MODE)},	/* vin2a_d23.rgmii1_rxd0 (NET2_3_RXD0) */

	{VOUT1_CLK, (M0 | PIN_OUTPUT)},		/* vout1_clk.vout1_clk (VDCK) */
	{VOUT1_DE, (M0 | PIN_OUTPUT)},		/* vout1_de.vout1_de (VDDE) */
	{VOUT1_HSYNC, (M0 | PIN_OUTPUT)},	/* vout1_hsync.vout1_hsync (VDHS) */
	{VOUT1_VSYNC, (M0 | PIN_OUTPUT)},	/* vout1_vsync.vout1_vsync (VDVS) */
	{VOUT1_D0, (M0 | PIN_OUTPUT)},		/* vout1_d0.vout1_d0 (VD0) */
	{VOUT1_D1, (M0 | PIN_OUTPUT)},		/* vout1_d1.vout1_d1 (VD1) */
	{VOUT1_D2, (M0 | PIN_OUTPUT)},		/* vout1_d2.vout1_d2 (VD2) */
	{VOUT1_D3, (M0 | PIN_OUTPUT)},		/* vout1_d3.vout1_d3 (VD3) */
	{VOUT1_D4, (M0 | PIN_OUTPUT)},		/* vout1_d4.vout1_d4 (VD4) */
	{VOUT1_D5, (M0 | PIN_OUTPUT)},		/* vout1_d5.vout1_d5 (VD5) */
	{VOUT1_D6, (M0 | PIN_OUTPUT)},		/* vout1_d6.vout1_d6 (VD6) */
	{VOUT1_D7, (M0 | PIN_OUTPUT)},		/* vout1_d7.vout1_d7 (VD7) */
	{VOUT1_D8, (M0 | PIN_OUTPUT)},		/* vout1_d8.vout1_d8 (VD8) */
	{VOUT1_D9, (M0 | PIN_OUTPUT)},		/* vout1_d9.vout1_d9 (VD9) */
	{VOUT1_D10, (M0 | PIN_OUTPUT)},		/* vout1_d10.vout1_d10 (VD10) */
	{VOUT1_D11, (M0 | PIN_OUTPUT)},		/* vout1_d11.vout1_d11 (VD11) */
	{VOUT1_D12, (M0 | PIN_OUTPUT)},		/* vout1_d12.vout1_d12 (VD12) */
	{VOUT1_D13, (M0 | PIN_OUTPUT)},		/* vout1_d13.vout1_d13 (VD13) */
	{VOUT1_D14, (M0 | PIN_OUTPUT)},		/* vout1_d14.vout1_d14 (VD14) */
	{VOUT1_D15, (M0 | PIN_OUTPUT)},		/* vout1_d15.vout1_d15 (VD15) */
	{VOUT1_D16, (M0 | PIN_OUTPUT)},		/* vout1_d16.vout1_d16 (VD16) */
	{VOUT1_D17, (M0 | PIN_OUTPUT)},		/* vout1_d17.vout1_d17 (VD17) */
	{VOUT1_D18, (M0 | PIN_OUTPUT)},		/* vout1_d18.vout1_d18 (VD18) */
	{VOUT1_D19, (M0 | PIN_OUTPUT)},		/* vout1_d19.vout1_d19 (VD19) */
	{VOUT1_D20, (M0 | PIN_OUTPUT)},		/* vout1_d20.vout1_d20 (VD20) */
	{VOUT1_D21, (M0 | PIN_OUTPUT)},		/* vout1_d21.vout1_d21 (VD21) */
	{VOUT1_D22, (M0 | PIN_OUTPUT)},		/* vout1_d22.vout1_d22 (VD22) */
	{VOUT1_D23, (M0 | PIN_OUTPUT)},		/* vout1_d23.vout1_d23 (VD23) */

	{RGMII0_TXC, (M0 | PIN_OUTPUT | MANUAL_MODE)},	/* rgmii0_txc.rgmii0_txc (NET1_0_TXC) */
	{RGMII0_TXCTL, (M0 | PIN_OUTPUT | MANUAL_MODE)},/* rgmii0_txctl.rgmii0_txctl (NET1_0_TXCTL) */
	{RGMII0_TXD3, (M0 | PIN_OUTPUT | MANUAL_MODE)},	/* rgmii0_txd3.rgmii0_txd3 (NET1_0_TXD3) */
	{RGMII0_TXD2, (M0 | PIN_OUTPUT | MANUAL_MODE)},	/* rgmii0_txd2.rgmii0_txd2 (NET1_0_TXD2) */
	{RGMII0_TXD1, (M0 | PIN_OUTPUT | MANUAL_MODE)},	/* rgmii0_txd1.rgmii0_txd1 (NET1_0_TXD1) */
	{RGMII0_TXD0, (M0 | PIN_OUTPUT | MANUAL_MODE)},	/* rgmii0_txd0.rgmii0_txd0 (NET1_0_TXD0) */
	{RGMII0_RXC, (M0 | PIN_INPUT | MANUAL_MODE)},	/* rgmii0_rxc.rgmii0_rxc (NET1_0_RXC) */
	{RGMII0_RXCTL, (M0 | PIN_INPUT | MANUAL_MODE)},	/* rgmii0_rxctl.rgmii0_rxctl (NET1_0_RXCTL) */
	{RGMII0_RXD3, (M0 | PIN_INPUT | MANUAL_MODE)},	/* rgmii0_rxd3.rgmii0_rxd3 (NET1_0_RXD3) */
	{RGMII0_RXD2, (M0 | PIN_INPUT | MANUAL_MODE)},	/* rgmii0_rxd2.rgmii0_rxd2 (NET1_0_RXD2) */
	{RGMII0_RXD1, (M0 | PIN_INPUT | MANUAL_MODE)},	/* rgmii0_rxd1.rgmii0_rxd1 (NET1_0_RXD1) */
	{RGMII0_RXD0, (M0 | PIN_INPUT | MANUAL_MODE)},	/* rgmii0_rxd0.rgmii0_rxd0 (NET1_0_RXD0) */

	{USB1_DRVVBUS, (M14 | PIN_INPUT)},	/* usb1_drvvbus.gpio6_12 (USB2_E_nOC3) */
	{USB2_DRVVBUS, (M14 | PIN_INPUT)},	/* usb2_drvvbus.gpio6_13 (USB2_E_nOC2) */

	{XREF_CLK2, (M3 | PIN_OUTPUT)},		/* xref_clk2.mcasp3_ahclkx (AUD_3_MCLK) */

	{MCASP3_ACLKX, (M0 | PIN_INPUT)},	/* mcasp3_aclkx.mcasp3_aclkx (AUD_0_BCLK) */
	{MCASP3_FSX, (M0 | PIN_OUTPUT)},	/* mcasp3_fsx.mcasp3_fsx (AUD_0_WCLK) */
	{MCASP3_AXR0, (M0 | PIN_OUTPUT)},	/* mcasp3_axr0.mcasp3_axr0 (AUD_0_DIN) */
	{MCASP3_AXR1, (M0 | PIN_INPUT)},	/* mcasp3_axr1.mcasp3_axr1 (AUD_0_DOUT) */

	{MCASP4_ACLKX, (M3 | PIN_INPUT_PULLUP)},	/* mcasp4_aclkx.uart8_rxd (UART8_3_RXD) */
	{MCASP4_FSX, (M3 | PIN_OUTPUT)},		/* mcasp4_fsx.uart8_txd (UART8_3_TXD) */
	{MCASP4_AXR0, (M3 | PIN_INPUT_PULLUP)},		/* mcasp4_axr0.uart8_ctsn (UART8_3_nCTS) */
	{MCASP4_AXR1, (M3 | PIN_OUTPUT)},		/* mcasp4_axr1.uart8_rtsn (UART8_3_nRTS) */

	{MCASP5_ACLKX, (M4 | PIN_INPUT)},	/* mcasp5_aclkx.i2c5_sda (IIC5_4_SDA) */
	{MCASP5_FSX, (M4 | PIN_INPUT)},		/* mcasp5_fsx.i2c5_scl (IIC5_4_SCL) */

	{MMC1_CLK, (M0 | PIN_OUTPUT)},		/* mmc1_clk.mmc1_clk (MMC1_CLK) */
	{MMC1_CMD, (M0 | PIN_OUTPUT)},		/* mmc1_cmd.mmc1_cmd (MMC1_CMD) */
	{MMC1_DAT0, (M0 | PIN_INPUT)},		/* mmc1_dat0.mmc1_dat0 (MMC1_DAT0) */
	{MMC1_DAT1, (M0 | PIN_INPUT)},		/* mmc1_dat1.mmc1_dat1 (MMC1_DAT1) */
	{MMC1_DAT2, (M0 | PIN_INPUT)},		/* mmc1_dat2.mmc1_dat2 (MMC1_DAT2) */
	{MMC1_DAT3, (M0 | PIN_INPUT)},		/* mmc1_dat3.mmc1_dat3 (MMC1_DAT3) */
	{MMC1_SDCD, (M0 | PIN_INPUT)},		/* mmc1_sdcd.mmc1_sdcd (MMC1_SDCD) */

	{MMC1_SDWP, (M14 | PIN_INPUT)},		/* mmc1_sdwp.gpio6_28 (USB1_E_nOC2) */
	{GPIO6_10, (M0 | PIN_INPUT)},		/* gpio6_10.gpio6_10 (USB1_E_nOC0) */
	{GPIO6_11, (M0 | PIN_INPUT)},		/* gpio6_11.gpio6_11 (USB1_E_nOC1) */
	{MMC3_CLK, (M14 | PIN_INPUT)},		/* mmc3_clk.gpio6_29 (USB2_E_nOC1) */
	{MMC3_CMD, (M14 | PIN_INPUT)},		/* mmc3_cmd.gpio6_30 (USB2_E_nOC0) */
	{MMC3_DAT0, (M14 | PIN_INPUT)},		/* mmc3_dat0.gpio6_31 (USB1_E_nOC3) */

	{MMC3_DAT1, (M14 | PIN_OUTPUT)},	/* mmc3_dat1.gpio7_0 (MMC1_E_PWROFF) */
	{MMC3_DAT2, (M14 | PIN_OUTPUT)},	/* mmc3_dat2.gpio7_1 (unconnected) */
	{MMC3_DAT3, (M14 | PIN_OUTPUT)},	/* mmc3_dat3.gpio7_2 (unconnected) */

	{SPI1_SCLK, (M14 | PIN_OUTPUT)},	/* spi1_sclk.gpio7_7 (GPIO_E_SOFTOFF) */

	{SPI1_CS0, (M14 | PIN_INPUT)},		/* spi1_cs0.gpio7_10 (PMIC_E_INT) */

	{SPI2_SCLK, (M14 | PIN_INPUT_PULLUP)},	/* spi2_sclk.gpio7_14 (UART8_E_nDCD) */
	{SPI2_D1, (M14 | PIN_INPUT_PULLUP)},	/* spi2_d1.gpio7_15 (UART8_E_nDSR) */
	{SPI2_D0, (M14 | PIN_OUTPUT)},		/* spi2_d0.gpio7_16 (UART8_E_nDTR) */
	{SPI2_CS0, (M14 | PIN_INPUT_PULLUP)},	/* spi2_cs0.gpio7_17 (UART8_E_nRI) */

	{DCAN1_TX, (M15 | PULL_UP)},		/* dcan1_tx.safe (DCAN1_TX) */
	{DCAN1_RX, (M15 | PULL_UP)},		/* dcan1_rx.safe (DCAN1_RX) */

	{UART1_RXD, (M0 | PIN_INPUT_PULLUP)},	/* uart1_rxd.uart1_rxd (UART1_0_RXD) */
	{UART1_TXD, (M0 | PIN_OUTPUT)},		/* uart1_txd.uart1_txd (UART1_0_TXD) */
	{UART1_CTSN, (M0 | PIN_INPUT_PULLUP)},	/* uart1_ctsn.uart1_ctsn (UART1_0_nCTS) */
	{UART1_RTSN, (M0 | PIN_OUTPUT)},	/* uart1_rtsn.uart1_rtsn (UART1_0_nRTS) */
	{UART2_RXD, (M5 | PIN_INPUT_PULLUP)},	/* uart2_rxd.uart1_dcdn (UART1_5_nDCD) */
	{UART2_TXD, (M5 | PIN_INPUT_PULLUP)},	/* uart2_txd.uart1_dsrn (UART1_5_nDSR) */
	{UART2_CTSN, (M5 | PIN_OUTPUT)},	/* uart2_ctsn.uart1_dtrn (UART1_5_nDTR) */
	{UART2_RTSN, (M5 | PIN_INPUT_PULLUP)},	/* uart2_rtsn.uart1_rin (UART1_5_nRI) */

	{I2C1_SDA, (M0 | PIN_INPUT)},		/* i2c1_sda.i2c1_sda (IIC1_0_SDA) */
	{I2C1_SCL, (M0 | PIN_INPUT)},		/* i2c1_scl.i2c1_scl (IIC1_0_SCL) */

	{WAKEUP2, (M1 | PIN_INPUT)},		/* wakeup2.sys_nirq2 */
	{WAKEUP3, (M1 | PIN_INPUT)},		/* wakeup3.sys_nirq1 */
};

const struct pad_conf_entry early_padconf[] = {
	{UART1_RXD, (M0 | PIN_INPUT_PULLUP)},	/* uart1_rxd.uart1_rxd (UART1_0_RXD) */
	{UART1_TXD, (M0 | PIN_OUTPUT)},		/* uart1_txd.uart1_txd (UART1_0_TXD) */
	{I2C1_SDA, (M0 | PIN_INPUT)},		/* i2c1_sda.i2c1_sda (IIC1_0_SDA) */
	{I2C1_SCL, (M0 | PIN_INPUT)},		/* i2c1_scl.i2c1_scl (IIC1_0_SCL) */
};

#ifdef CONFIG_IODELAY_RECALIBRATION
const struct iodelay_cfg_entry iodelay_cfg_array[] = {
	{0x06F0, 480, 0},	/* CFG_RGMII0_RXC_IN */
	{0x06FC, 111, 1641},	/* CFG_RGMII0_RXCTL_IN */
	{0x0708, 272, 1116},	/* CFG_RGMII0_RXD0_IN */
	{0x0714, 243, 1260},	/* CFG_RGMII0_RXD1_IN */
	{0x0720, 0, 1614},	/* CFG_RGMII0_RXD2_IN */
	{0x072C, 105, 1673},	/* CFG_RGMII0_RXD3_IN */
	{0x0740, 531, 120},	/* CFG_RGMII0_TXC_OUT */
	{0x074C, 11, 60},	/* CFG_RGMII0_TXCTL_OUT */
	{0x0758, 7, 120},	/* CFG_RGMII0_TXD0_OUT */
	{0x0764, 0, 0},		/* CFG_RGMII0_TXD1_OUT */
	{0x0770, 276, 120},	/* CFG_RGMII0_TXD2_OUT */
	{0x077C, 440, 120},	/* CFG_RGMII0_TXD3_OUT */
	{0x0A70, 1551, 115},	/* CFG_VIN2A_D12_OUT */
	{0x0A7C, 816, 0},	/* CFG_VIN2A_D13_OUT */
	{0x0A88, 876, 0},	/* CFG_VIN2A_D14_OUT */
	{0x0A94, 312, 0},	/* CFG_VIN2A_D15_OUT */
	{0x0AA0, 58, 0},	/* CFG_VIN2A_D16_OUT */
	{0x0AAC, 0, 0},		/* CFG_VIN2A_D17_OUT */
	{0x0AB0, 702, 0},	/* CFG_VIN2A_D18_IN */
	{0x0ABC, 136, 976},	/* CFG_VIN2A_D19_IN */
	{0x0AD4, 210, 1357},	/* CFG_VIN2A_D20_IN */
	{0x0AE0, 189, 1462},	/* CFG_VIN2A_D21_IN */
	{0x0AEC, 232, 1278},	/* CFG_VIN2A_D22_IN */
	{0x0AF8, 0, 1397},	/* CFG_VIN2A_D23_IN */
};
#endif

#endif /* _MUX_DATA_ELESAR_TITANIUM_H_ */
