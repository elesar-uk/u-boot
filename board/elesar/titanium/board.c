/*
 * Copyright (C) 2015 Elesar Ltd. - http://www.elesar.co.uk
 *
 * Based on board/ti/beagle_x15/board.c
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <palmas.h>
#include <sata.h>
#include <usb.h>
#include <miiphy.h>
#include <asm/omap_common.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/dra7xx_iodelay.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sata.h>
#include <asm/arch/gpio.h>
#include <environment.h>

#include "mux_data.h"

#ifdef CONFIG_DRIVER_TI_CPSW
#include <cpsw.h>
#endif

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
	.mpu.value		= VDD_MPU_DRA752,
	.mpu.efuse.reg		= STD_FUSE_OPP_VMIN_MPU_NOM,
	.mpu.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.mpu.addr		= TPS659038_REG_ADDR_SMPS12,
	.mpu.pmic		= &tps659038,

	.eve.value		= VDD_EVE_DRA752,
	.eve.efuse.reg		= STD_FUSE_OPP_VMIN_DSPEVE_NOM,
	.eve.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.eve.addr		= TPS659038_REG_ADDR_SMPS45,
	.eve.pmic		= &tps659038,

	.gpu.value		= VDD_GPU_DRA752,
	.gpu.efuse.reg		= STD_FUSE_OPP_VMIN_GPU_NOM,
	.gpu.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.gpu.addr		= TPS659038_REG_ADDR_SMPS45,
	.gpu.pmic		= &tps659038,

	.core.value		= VDD_CORE_DRA752,
	.core.efuse.reg		= STD_FUSE_OPP_VMIN_CORE_NOM,
	.core.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.core.addr		= TPS659038_REG_ADDR_SMPS6,
	.core.pmic		= &tps659038,

	.iva.value		= VDD_IVA_DRA752,
	.iva.efuse.reg		= STD_FUSE_OPP_VMIN_IVA_NOM,
	.iva.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.iva.addr		= TPS659038_REG_ADDR_SMPS45,
	.iva.pmic		= &tps659038,
};

void hw_data_init(void)
{
	*prcm = &dra7xx_prcm;
	*dplls_data = &dra7xx_dplls;
	*omap_vcores = &titanium_volts;
	*ctrl = &dra7xx_ctrl;
}

int board_init(void)
{
	gpmc_init();
	gd->bd->bi_boot_params = (CONFIG_SYS_SDRAM_BASE + 0x100);

	return 0;
}

int board_late_init(void)
{
	init_sata(0);
	/*
	 * DEV_CTRL.DEV_ON = 1 please - else palmas switches off in 8 seconds
	 * This is the POWERHOLD-in-Low behavior.
	 */
	palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0xA0, 0x1);
	return 0;
}

void set_muxconf_regs_essential(void)
{
	uint32_t ctrl_val;

	do_set_mux32((*ctrl)->control_padconf_core_base,
		     early_padconf, ARRAY_SIZE(early_padconf));

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

	writel(0x96 << 16, (*ctrl)->control_pcie_pcs);

	ctrl_val = readl((*ctrl)->control_sma_sw_6);
	ctrl_val = (ctrl_val & ~0x30000) | 0x10000;
	writel(ctrl_val, (*ctrl)->control_sma_sw_6);
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

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_GENERIC_MMC)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0, -1, -1);
	return 0;
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
	env_relocate_spec();
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

#ifdef CONFIG_DRIVER_TI_CPSW

static void cpsw_control(int enabled)
{
	/* VTP can be added here */
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 7,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 3,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 2,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};

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

int board_eth_init(bd_t *bis)
{
	int ret;
	uint8_t mac_addr[6];
	bool mac_found;
	uint32_t ctrl_val;

	/* Try reading mac address from PROM */
	mac_found = read_mac_address(mac_addr);

	if (!getenv("ethaddr")) {
		if (mac_found && is_valid_ethaddr(mac_addr)) {
			eth_setenv_enetaddr("ethaddr", mac_addr);
		}
	}

	if (!getenv("eth1addr")) {
		mac_addr[5] ^= 1;
		if (mac_found && is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}

	ctrl_val = readl((*ctrl)->control_core_control_io1) & (~0x33);
	ctrl_val |= 0x22;
	writel(ctrl_val, (*ctrl)->control_core_control_io1);

	ret = cpsw_register(&cpsw_data);
	if (ret < 0)
		printf("Error %d registering CPSW switch\n", ret);

	return ret;
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
