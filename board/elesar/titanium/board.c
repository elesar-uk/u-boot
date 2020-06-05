/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015 Elesar Ltd. - http://www.elesar.co.uk
 *
 * Based on board/ti/am57xx/board.c
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 */

#include <common.h>
#include <palmas.h>
#include <miiphy.h>
#include <asm/omap_common.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/dra7xx_iodelay.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/gpio.h>
#include <environment.h>
#include <mmc.h>

#include "mux_data.h"

#if defined(CONFIG_PHYLIB) && defined(CONFIG_PHY_MICREL)
#include <micrel.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

const struct omap_sysinfo sysinfo = {
	"Board: Elesar Titanium\n"
};

static const struct dmm_lisa_map_regs titanium_lisa_regs = {
	.dmm_lisa_map_3 = 0x80740300,
	.is_ma_present  = 0x1
};

void emif_get_dmm_regs(const struct dmm_lisa_map_regs **dmm_lisa_regs)
{
	*dmm_lisa_regs = &titanium_lisa_regs;
}

static const struct emif_regs titanium_emif1_ddr3_532mhz_emif_regs = {
	.sdram_config_init	= 0x61851b32,
	.sdram_config		= 0x61851b32,
	.sdram_config2		= 0x00000000,
	.ref_ctrl		= 0x000040F1,
	.ref_ctrl_final		= 0x00001035,
	.sdram_tim1		= 0xceef266b,
	.sdram_tim2		= 0x328f7fda,
	.sdram_tim3		= 0x027f88a8,
	.read_idle_ctrl		= 0x00050000,
	.zq_config		= 0x0007190b,
	.temp_alert_config	= 0x00000000,
	.emif_ddr_phy_ctlr_1_init = 0x0024400b,
	.emif_ddr_phy_ctlr_1	= 0x0e24400b,
	.emif_ddr_ext_phy_ctrl_1 = 0x10040100,
	.emif_ddr_ext_phy_ctrl_2 = 0x00740074,
	.emif_ddr_ext_phy_ctrl_3 = 0x00780078,
	.emif_ddr_ext_phy_ctrl_4 = 0x007c007c,
	.emif_ddr_ext_phy_ctrl_5 = 0x007b007b,
	.emif_rd_wr_lvl_rmp_win	= 0x00000000,
	.emif_rd_wr_lvl_rmp_ctl	= 0x80000000,
	.emif_rd_wr_lvl_ctl	= 0x00000000,
	.emif_rd_wr_exec_thresh	= 0x00000305
};

/* Ext phy ctrl regs 1-35 */
static const u32 titanium_emif1_ddr3_ext_phy_ctrl_const_regs[] = {
	0x10040100,
	0x00740074,
	0x00780078,
	0x007c007c,
	0x007b007b,
	0x00800080,
	0x00360036,
	0x00340034,
	0x00360036,
	0x00350035,
	0x00350035,

	0x01ff01ff,
	0x01ff01ff,
	0x01ff01ff,
	0x01ff01ff,
	0x01ff01ff,

	0x00430043,
	0x003e003e,
	0x004a004a,
	0x00470047,
	0x00400040,

	0x00000000,
	0x00600020,
	0x40011080,
	0x08102040,

	0x00400040,
	0x00400040,
	0x00400040,
	0x00400040,
	0x00400040,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0
};

static const struct emif_regs titanium_emif2_ddr3_532mhz_emif_regs = {
	.sdram_config_init	= 0x61851b32,
	.sdram_config		= 0x61851b32,
	.sdram_config2		= 0x00000000,
	.ref_ctrl		= 0x000040F1,
	.ref_ctrl_final		= 0x00001035,
	.sdram_tim1		= 0xceef266b,
	.sdram_tim2		= 0x328f7fda,
	.sdram_tim3		= 0x027f88a8,
	.read_idle_ctrl		= 0x00050000,
	.zq_config		= 0x0007190b,
	.temp_alert_config	= 0x00000000,
	.emif_ddr_phy_ctlr_1_init = 0x0024400b,
	.emif_ddr_phy_ctlr_1	= 0x0e24400b,
	.emif_ddr_ext_phy_ctrl_1 = 0x10040100,
	.emif_ddr_ext_phy_ctrl_2 = 0x00820082,
	.emif_ddr_ext_phy_ctrl_3 = 0x008b008b,
	.emif_ddr_ext_phy_ctrl_4 = 0x00800080,
	.emif_ddr_ext_phy_ctrl_5 = 0x007e007e,
	.emif_rd_wr_lvl_rmp_win	= 0x00000000,
	.emif_rd_wr_lvl_rmp_ctl	= 0x80000000,
	.emif_rd_wr_lvl_ctl	= 0x00000000,
	.emif_rd_wr_exec_thresh	= 0x00000305
};

static const u32 titanium_emif2_ddr3_ext_phy_ctrl_const_regs[] = {
	0x10040100,
	0x00820082,
	0x008b008b,
	0x00800080,
	0x007e007e,
	0x00800080,
	0x00370037,
	0x00390039,
	0x00360036,
	0x00370037,
	0x00350035,
	0x01ff01ff,
	0x01ff01ff,
	0x01ff01ff,
	0x01ff01ff,
	0x01ff01ff,
	0x00540054,
	0x00540054,
	0x004e004e,
	0x004c004c,
	0x00400040,

	0x00000000,
	0x00600020,
	0x40011080,
	0x08102040,

	0x00400040,
	0x00400040,
	0x00400040,
	0x00400040,
	0x00400040,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0
};

void emif_get_reg_dump(u32 emif_nr, const struct emif_regs **regs)
{
	switch (emif_nr) {
	case 1:
		*regs = &titanium_emif1_ddr3_532mhz_emif_regs;
		break;
	case 2:
		*regs = &titanium_emif2_ddr3_532mhz_emif_regs;
		break;
	}
}

void emif_get_ext_phy_ctrl_const_regs(u32 emif_nr, const u32 **regs, u32 *size)
{
	switch (emif_nr) {
	case 1:
		*regs = titanium_emif1_ddr3_ext_phy_ctrl_const_regs;
		*size = ARRAY_SIZE(titanium_emif1_ddr3_ext_phy_ctrl_const_regs);
		break;
	case 2:
		*regs = titanium_emif2_ddr3_ext_phy_ctrl_const_regs;
		*size = ARRAY_SIZE(titanium_emif2_ddr3_ext_phy_ctrl_const_regs);
		break;
	}
}

struct vcores_data titanium_volts = {
	.mpu.value[OPP_NOM]	= VDD_MPU_DRA7_NOM,
	.mpu.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_MPU_NOM,
	.mpu.efuse.reg_bits     = DRA752_EFUSE_REGBITS,
	.mpu.addr		= TPS659038_REG_ADDR_SMPS12,
	.mpu.pmic		= &tps659038,
	.mpu.abb_tx_done_mask	= OMAP_ABB_MPU_TXDONE_MASK,

	.eve.value[OPP_NOM]	= VDD_EVE_DRA7_NOM,
	.eve.value[OPP_OD]	= VDD_EVE_DRA7_OD,
	.eve.value[OPP_HIGH]	= VDD_EVE_DRA7_HIGH,
	.eve.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_DSPEVE_NOM,
	.eve.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_DSPEVE_OD,
	.eve.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_DSPEVE_HIGH,
	.eve.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.eve.addr		= TPS659038_REG_ADDR_SMPS45,
	.eve.pmic		= &tps659038,
	.eve.abb_tx_done_mask	= OMAP_ABB_EVE_TXDONE_MASK,

	.gpu.value[OPP_NOM]	= VDD_GPU_DRA7_NOM,
	.gpu.value[OPP_OD]	= VDD_GPU_DRA7_OD,
	.gpu.value[OPP_HIGH]	= VDD_GPU_DRA7_HIGH,
	.gpu.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_GPU_NOM,
	.gpu.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_GPU_OD,
	.gpu.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_GPU_HIGH,
	.gpu.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.gpu.addr		= TPS659038_REG_ADDR_SMPS45,
	.gpu.pmic		= &tps659038,
	.gpu.abb_tx_done_mask	= OMAP_ABB_GPU_TXDONE_MASK,

	.core.value[OPP_NOM]	= VDD_CORE_DRA7_NOM,
	.core.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_CORE_NOM,
	.core.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.core.addr		= TPS659038_REG_ADDR_SMPS6,
	.core.pmic		= &tps659038,

	.iva.value[OPP_NOM]	= VDD_IVA_DRA7_NOM,
	.iva.value[OPP_OD]	= VDD_IVA_DRA7_OD,
	.iva.value[OPP_HIGH]	= VDD_IVA_DRA7_HIGH,
	.iva.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_IVA_NOM,
	.iva.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_IVA_OD,
	.iva.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_IVA_HIGH,
	.iva.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.iva.addr		= TPS659038_REG_ADDR_SMPS45,
	.iva.pmic		= &tps659038,
	.iva.abb_tx_done_mask	= OMAP_ABB_IVA_TXDONE_MASK,
};

int get_voltrail_opp(int rail_offset)
{
	int opp;

	switch (rail_offset) {
	case VOLT_MPU:
		opp = DRA7_MPU_OPP;
		break;
	case VOLT_CORE:
		opp = DRA7_CORE_OPP;
		break;
	case VOLT_GPU:
		opp = DRA7_GPU_OPP;
		break;
	case VOLT_EVE:
		opp = DRA7_DSPEVE_OPP;
		break;
	case VOLT_IVA:
		opp = DRA7_IVA_OPP;
		break;
	default:
		opp = OPP_NOM;
	}

	return opp;
}

void vcores_init(void)
{
	*omap_vcores = &titanium_volts;
}

void hw_data_init(void)
{
	*prcm = &dra7xx_prcm;
	*dplls_data = &dra7xx_dplls;
	*ctrl = &dra7xx_ctrl;
}

int board_init(void)
{
#if !defined(CONFIG_SPL_BUILD)
	uint32_t ctrl_val;

	/* Switch PCIe to two root complexes of 1 lane, set
	 * the physical coding sublayer delay to nominal, output
	 * the reference clock on the low jitter clock buffer.
	 */
	ctrl_val = readl((*ctrl)->control_pcie_control);
	ctrl_val = ctrl_val & ~0x5; /* 1 lane */
	writel(ctrl_val, (*ctrl)->control_pcie_control);
	ctrl_val = readl((*ctrl)->control_core_control_io2);
	ctrl_val = ctrl_val & ~(1 << 13); /* 1 lane */
	writel(ctrl_val, (*ctrl)->control_core_control_io2);

	/* Set PCIESS_PCS_RC_DELAY_COUNT to nominal */
	writel(0x96 << 16, (*ctrl)->control_pcie_pcs);

	/* Set PCIE_TX_RX_CONTROL to ACSPCIe TX Mode */
	ctrl_val = readl((*ctrl)->control_sma_sw_6);
	ctrl_val = (ctrl_val & ~0x30000) | 0x10000;
	writel(ctrl_val, (*ctrl)->control_sma_sw_6);

	/*
	 * Repeat the padconf setup for a subset of pins to cover the case
	 * where U-Boot has been started from RISC OS, when the SPL stage is
	 * skipped and so recalibrate_iodelay() has not been called.
	 */
	do_set_mux32((*ctrl)->control_padconf_core_base,
		     late_padconf, ARRAY_SIZE(late_padconf));
#endif

	gpmc_init();
	gd->bd->bi_boot_params = (CONFIG_SYS_SDRAM_BASE + 0x100);

	return 0;
}

#if !defined(CONFIG_SPL_BUILD)

#define BitClkSync	230
#define BitClkHigh	15
#define BitClkLow	15

#define PROM_SIZE	64
#define OTP_TAG_EUI_48	0xf7

static bool read_mac_address(uint8_t *mac)
{
	uint8_t UniquePROM[PROM_SIZE];
	unsigned int i;
	uint8_t mask;
	uint8_t v = 0;
	uint32_t d;
	bool found_id;
	size_t offset, size;
	uint8_t type;

	/* Read the PROM */
	gpio_request(GPIO_OTP_CLOCK, "otp_clock");
	gpio_direction_output(GPIO_OTP_CLOCK, 1);

	gpio_request(GPIO_OTP_DATA, "otp_data");
	gpio_direction_input(GPIO_OTP_DATA);

	udelay(BitClkSync);

	for (i = 0; i < sizeof(UniquePROM); i++) {
		for (mask = 0x80; mask != 0; mask >>= 1) {
			gpio_set_value(GPIO_OTP_CLOCK, 0);
			udelay(BitClkLow);
			/* Capture on rising edge */
			gpio_set_value(GPIO_OTP_CLOCK, 1);
			d = gpio_get_value(GPIO_OTP_DATA);

			v = (d == 0) ? v & ~mask : v | mask;
			udelay(BitClkHigh);
		}
		UniquePROM[i] = v;
	}

	/* Scan the PROM for an EUI48 */
	found_id = false;
	for (i = 0; i < sizeof(UniquePROM) / 4 && !found_id; i++)
	{
		d = ((uint32_t *)UniquePROM)[i];
		if (d == 0)
		{
			/* End of directory */
			break;
		}
		offset = d >> 16;
		size = (d >> 8) & 0xff;
		type = d & 0xff;
		if ((offset + size) > sizeof(UniquePROM))
		{
			/* Junk length or offset */
			break;
		}

		if (type == OTP_TAG_EUI_48 && size == 6)
		{
			memcpy(mac, UniquePROM + offset, size);
			found_id = true;
		}
	}

	return found_id;
}

static void board_set_ethaddr(void)
{
	uint8_t mac_addr[6];
	bool mac_found;

	/* Try reading mac address from PROM */
	mac_found = read_mac_address(mac_addr);

	if (!env_get("ethaddr")) {
		if (mac_found && is_valid_ethaddr(mac_addr)) {
			eth_env_set_enetaddr("ethaddr", mac_addr);
		}
	}

	if (!env_get("eth1addr")) {
		mac_addr[5] ^= 1;
		if (mac_found && is_valid_ethaddr(mac_addr))
			eth_env_set_enetaddr("eth1addr", mac_addr);
	}
}
#endif

#define TFP410_I2C_ADDR		0x38
#define TFP410_CTL_1_MODE	8
#define TFP410_CTL_3_MODE	10

static int update_tfp410(struct udevice *dev, uint reg, uint8_t mask, uint8_t set)
{
	uint8_t val;
	int ret;

	ret = dm_i2c_read(dev, reg, &val, 1);
	if (ret < 0) {
		return ret;
	}
	val = (val & ~mask) | set;
	return dm_i2c_write(dev, reg, &val, 1);
}

static void enable_tfp410(void)
{
	struct udevice *dev;
	int ret;

	ret = i2c_get_chip_for_busnum(0, TFP410_I2C_ADDR, 1, &dev);
	if (ret == 0) {
		/* Configure 24b, and turn on power */
		ret = update_tfp410(dev, TFP410_CTL_1_MODE, 0x40, 0x37);
	}
	if (ret == 0) {
		/* Set skew to 13 */
		ret = update_tfp410(dev, TFP410_CTL_3_MODE, 0xF0, 0xD0);
	}
	if (ret != 0) {
		pr_err("Failed to enable TFP410 (%d)\n", ret);
	}
}

int board_late_init(void)
{
	u8 smps_ctrl;

	/*
	 * As there's always a load from the MPU, set SMPS12 to always run
	 * in double mode to even the component stress.
	 */
	palmas_i2c_read_u8(TPS65903X_CHIP_P1, SMPS_CTRL, &smps_ctrl);
	smps_ctrl = (smps_ctrl & ~SMPS_CTRL_123_PHASE(SMPS_CTRL_PHASE_MASK)) |
	       SMPS_CTRL_123_PHASE(SMPS_CTRL_PHASE_DOUBLE);
	palmas_i2c_write_u8(TPS65903X_CHIP_P1, SMPS_CTRL, smps_ctrl);

	/* Enable the TFP410 encoder */
	enable_tfp410();

#if !defined(CONFIG_SPL_BUILD)
	/* Set the MAC addresses */
	board_set_ethaddr();
#endif

	return 0;
}

void gpi2c_init(void)
{
}

void set_muxconf_regs(void)
{
	do_set_mux32((*ctrl)->control_padconf_core_base,
		     early_padconf, ARRAY_SIZE(early_padconf));
}

#ifdef CONFIG_IODELAY_RECALIBRATION
void recalibrate_iodelay(void)
{
	/* Ensure these GPIO output pin states are set before pad config. */
	gpio_request(GPIO_MMC_PWROFF, "mmc_pwroff");
	gpio_direction_output(GPIO_MMC_PWROFF, 0);
	gpio_request(GPIO_SOFTOFF, "softoff");
	gpio_direction_output(GPIO_SOFTOFF, 0);

	__recalibrate_iodelay(core_padconf_array_essential,
			      ARRAY_SIZE(core_padconf_array_essential),
			      iodelay_cfg_array, ARRAY_SIZE(iodelay_cfg_array));
}
#endif

#if defined(CONFIG_MMC)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0, -1, -1);
	return 0;
}

static const struct mmc_platform_fixups am57x_es1_1_mmc1_fixups = {
	.hw_rev = "rev11",
	.unsupported_caps = MMC_CAP(MMC_HS_200) |
			    MMC_CAP(UHS_SDR104),
	.max_freq = 96000000,
};

static const struct mmc_platform_fixups am57x_es1_1_mmc23_fixups = {
	.hw_rev = "rev11",
	.unsupported_caps = MMC_CAP(MMC_HS_200) |
			    MMC_CAP(UHS_SDR104) |
			    MMC_CAP(UHS_SDR50),
	.max_freq = 48000000,
};

const struct mmc_platform_fixups *platform_fixups_mmc(uint32_t addr)
{
	switch (omap_revision()) {
	case DRA752_ES1_0:
	case DRA752_ES1_1:
		if (addr == OMAP_HSMMC1_BASE)
			return &am57x_es1_1_mmc1_fixups;
		else
			return &am57x_es1_1_mmc23_fixups;
	default:
		return NULL;
	}
}
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_OS_BOOT)
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_load();
	if (env_get_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, bd_t *bd)
{
	ft_cpu_setup(blob, bd);

	return 0;
}
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if (!strcmp(name, "am57xx-titanium")) {
		return 0;
	}

	return -1;
}
#endif

#if defined(CONFIG_PHYLIB) && defined(CONFIG_PHY_MICREL)

#define MII_KSZ9031_EXT_COMMON_CTRL	0x0
#define CC_LEDOVRD	(1<<4)

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config) {
		int cc;

		/* Standard PHY configuration */
		phydev->drv->config(phydev);

		/* Set the correct LED behaviour */
		cc = ksz9031_phy_extended_read(phydev, 2,
					       MII_KSZ9031_EXT_COMMON_CTRL,
					       MII_KSZ9031_MOD_DATA_NO_POST_INC);
		cc |= CC_LEDOVRD;
		ksz9031_phy_extended_write(phydev, 2,
					   MII_KSZ9031_EXT_COMMON_CTRL,
					   MII_KSZ9031_MOD_DATA_NO_POST_INC,
					   cc);
	}

	return 0;
}

#endif
