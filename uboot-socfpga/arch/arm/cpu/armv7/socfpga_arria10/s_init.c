/*
 * Copyright (C) 2014-2017 Intel Corporation <www.intel.com>
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <common.h>
#include <asm/io.h>
#include <watchdog.h>
#include <asm/arch/fpga_manager.h>
#include <asm/arch/reset_manager.h>
#include <asm/arch/system_manager.h>
#include <asm/arch/clock_manager.h>
#include <asm/arch/ecc_ram.h>
#include <asm/arch/misc.h>
#include <asm/arch/sdram.h>
#include <asm/pl310.h>
#include <asm/sections.h>
#include <fdtdec.h>
#include <ns16550.h>

DECLARE_GLOBAL_DATA_PTR;

#define ALTERA_NONE 0
#define ALTERA_MMC 1
#define ALTERA_QSPI 2
#define ALTERA_EMMC 3
#define MMC_CLK_DIV 0x9
#define QSPI_CLK_DIV 0x384
#define ALTERA_PINMUX_OFFS 0xffd07200

#define ALTERA_GPIO_BASE 0xFFC02900
#define GPIO_DATA_OFFS 0x0
#define GPIO_DIR_OFFS 0x4
#define GPIO_OUT 0x1

void gpio_set(int gpio_chip, int gpio_nr, int value)
{
	u32 reg, offs;

	offs = ALTERA_GPIO_BASE + (gpio_chip * 0x100);
	reg = readl(offs + GPIO_DATA_OFFS);
	if (value)
		reg |= (1 << gpio_nr);
	else
		reg &= ~(1 << gpio_nr);

	writel(reg, offs + GPIO_DATA_OFFS);
}

void gpio_dir(int gpio_chip, int gpio_nr, int dir)
{
	u32 reg, offs;

	offs = ALTERA_GPIO_BASE + (gpio_chip * 0x100);
	reg = readl(offs + GPIO_DIR_OFFS);
	if (dir)
		reg |= (1 << gpio_nr);
	else
		reg &= ~(1 << gpio_nr);

	writel(reg, offs + GPIO_DIR_OFFS);
}

static void set_mux_mmc (void)
{
	u32 pinmux_arr[] = {0xc, 0x8, 0x10, 0x8, 0x14, 0x8, 0x18, 0x8, 0x1c, 0x8, 0x20, 0x8, 0x24, 0xf, 0x28, 0xf, 0x2c, 0xa, 0x30, 0xa, 0x34, 0xa, 0x38, 0xa, 0x110, 0xa282a};
	u32 len, i, offset, value;
	len = sizeof(pinmux_arr)/sizeof(u32);
	for (i=0; i<len; i+=2) {
		offset = pinmux_arr[i];
		value = pinmux_arr[i+1];
		writel(value, ALTERA_PINMUX_OFFS + offset);
	}
}

static void set_mux_emmc (void)
{
	u32 pinmux_arr[] = {0xc, 0x8, 0x10, 0x8, 0x14, 0x8, 0x18, 0x8, 0x1c, 0x8, 0x20, 0x8, 0x24, 0xf, 0x28, 0xf, 0x2c, 0x8, 0x30, 0x8, 0x34, 0x8, 0x38, 0x8, 0x110, 0xa282a, 0x130, 0xa282a, 0x134, 0xa282a, 0x138, 0xa282a, 0x13c, 0xa282a};
	u32 len, i, offset, value;
	len = sizeof(pinmux_arr)/sizeof(u32);
	for (i=0; i<len; i+=2) {
		offset = pinmux_arr[i];
		value = pinmux_arr[i+1];
		writel(value, ALTERA_PINMUX_OFFS + offset);
	}
}
static void set_mux_qspi (void)
{
	u32 pinmux_arr[] = {0xc, 0x4, 0x10, 0x4, 0x14, 0x4, 0x18, 0x4, 0x1c, 0x4, 0x20, 0x4, 0x24, 0xf, 0x28, 0xf, 0x2c, 0xa, 0x30, 0xa, 0x34, 0xa, 0x38, 0xa, 0x110, 0x8282a};
	u32 len, i, offset, value;
	len = sizeof(pinmux_arr)/sizeof(u32);
	for (i=0; i<len; i+=2) {
		offset = pinmux_arr[i];
		value = pinmux_arr[i+1];
		writel(value, ALTERA_PINMUX_OFFS + offset);
	}
}

static int altera_current_storage = ALTERA_NONE;
static const struct socfpga_clock_manager *clock_manager_base =
                (void *)SOCFPGA_CLKMGR_ADDRESS;

void altera_set_storage (int store)
{
	if (store == altera_current_storage)
		return;

	gpio_dir(2, 6, GPIO_OUT);
	gpio_dir(1, 5, GPIO_OUT);
	switch (store)
	{
		case ALTERA_MMC:
			set_mux_mmc();
			gpio_set(2, 6, 0);
			gpio_set(1, 5, 0);
			altera_current_storage = ALTERA_MMC;
			writel(MMC_CLK_DIV, &clock_manager_base->main_pll_cntr6clk);
			break;
		case ALTERA_EMMC:
			set_mux_emmc();
			gpio_set(2, 6, 1);
			gpio_set(1, 5, 1);
			altera_current_storage = ALTERA_EMMC;
			writel(MMC_CLK_DIV, &clock_manager_base->main_pll_cntr6clk);
			break;
		case ALTERA_QSPI:
			set_mux_qspi();
			gpio_set(2, 6, 1);
			gpio_set(1, 5, 0);
			altera_current_storage = ALTERA_QSPI;
			writel(QSPI_CLK_DIV, &clock_manager_base->main_pll_cntr6clk);
			break;
		default:
			altera_current_storage = ALTERA_NONE;
			break;
	}
}

int altera_set_storage_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if(argc != 2)
		return CMD_RET_USAGE;
	if(!strcmp(argv[1], "MMC"))
		altera_set_storage(ALTERA_MMC);
	else if (!strcmp(argv[1], "QSPI"))
		altera_set_storage(ALTERA_QSPI);
	else if (!strcmp(argv[1], "EMMC"))
		altera_set_storage(ALTERA_EMMC);
	else return CMD_RET_USAGE;

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(altera_set_storage, 2, 0, altera_set_storage_cmd,
		   "Set non volatile memory access",
		    "<MMC|QSPI|EMMC> - Set access for the selected memory device");

static int __do_pinctr_pins(const void *blob, int child, const char *node_name)
{
	int len;
	fdt_addr_t base_addr;
	fdt_size_t size;
	const u32 *cell;
	u32 offset, value;

	base_addr = fdtdec_get_addr_size(blob, child, "reg", &size);
	if (base_addr != FDT_ADDR_T_NONE) {
		debug("subnode %s %x:%x\n",
			node_name, base_addr, size);

		cell = fdt_getprop(blob, child, "pinctrl-single,pins",
			&len);
		if (cell != NULL) {
			debug("%p %d\n", cell, len);
			for (;len > 0; len -= (2*sizeof(u32))) {
				offset = fdt32_to_cpu(*cell++);
				value = fdt32_to_cpu(*cell++);
				debug("<0x%x 0x%x>\n", offset, value);
				writel(value, base_addr + offset);
			}
			return 0;
		}
	}
	return 1;
}
static int do_pinctrl_pins(const void *blob, int node, const char *child_name)
{
	int child, len;
	const char *node_name;

	child = fdt_first_subnode(blob, node);

	if (child < 0)
		return 2;

	node_name = fdt_get_name(blob, child, &len);

	while (node_name) {
		if (!strcmp(child_name, node_name)) {
			__do_pinctr_pins(blob, child, node_name);
			return(0);
		}
		child = fdt_next_subnode(blob, child);

		if (child < 0)
			break;

		node_name = fdt_get_name(blob, child, &len);
	}

	return 1;
}

int config_dedicated_pins(const void *blob)
{
	int node;

	node = fdtdec_next_compatible(blob, 0, COMPAT_PINCTRL_SINGLE);

	if (node < 0)
		return 1;

	if (do_pinctrl_pins(blob, node, "dedicated_cfg"))
		return 2;

	if (do_pinctrl_pins(blob, node, "dedicated"))
		return 3;

	return 0;
}

int config_pins(const void *blob, const char *pin_grp)
{
	int node;

	node = fdtdec_next_compatible(blob, 0, COMPAT_PINCTRL_SINGLE);

	if (node < 0)
		return 1;

	if (do_pinctrl_pins(blob, node, pin_grp))
		return 2;

	return 0;
}

/* This function initializes security policies to be consistent across
 * all logic units in the Arria 10.
 *
 * The idea is to set all security policies to be normal, nonsecure
 * for all units. This is to avoid bringup issue for user who
 * doesn't need any security or firewall protection.
 */
void
arria10_initialize_security_policies(void)
{

	/* Put OCRAM in non-secure */
	writel(0x003f0000, 0xffd1320c);
	writel(0x1, 0xffd13200);

	/* Put DDR in non-secure */
	writel(0xffff0000, 0xffd1340c);
	writel(0x1, 0xffd13400);

	/* Enable priviledge and non priviledge access to L4 peripherals */
	writel(~0, ALT_NOC_L4_PRIV_FLT_OFST);

	/* Enable secure and non secure transaction to bridges */
	writel(~0, ALT_NOC_FW_H2F_SCR_OFST);
	writel(~0, ALT_NOC_FW_H2F_SCR_OFST + 4);
}

/* This function masks all the ECC errors. The next stage
 * (Linux or a different OS) will unmask the appropriate
 * module in the ECC enable routine.
 */
void
arria10_mask_ecc_errors(void)
{
	const struct socfpga_system_manager *system_manager_base =
		(void *)SOCFPGA_SYSMGR_ADDRESS;

	writel(0x0007FFFF, &system_manager_base->ecc_intmask_set);
}

/*
 * First C function to initialize the critical hardware early
 */
void s_init(void)
{
	unsigned int com_port;

	/*
	 * Configure Clock Manager to use intosc clock instead external osc to
	 * ensure success watchdog operation. We do it as early as possible.
	 */
	cm_use_intosc();

	watchdog_disable();

	arria10_initialize_security_policies();

	arria10_mask_ecc_errors();

	/* Clear fake OCRAM ECC first as might triggered during power on */
	clear_ecc_ocram_ecc_status();

	/* Configure the L2 controller to make SDRAM start at 0	*/
	writel(0x1, SOCFPGA_MPUL2_ADRFLTR_START);

#ifdef CONFIG_HW_WATCHDOG
	/* release osc1 watchdog timer 0 from reset */
	reset_deassert_osc1wd0();

	/* reconfigure and enable the watchdog */
	hw_watchdog_init();
	WATCHDOG_RESET();
#endif /* CONFIG_HW_WATCHDOG */

#ifdef CONFIG_OF_CONTROL
	/* We need to access to FDT as this stage */
	memset((void *)gd, 0, sizeof(gd_t));
	/* FDT is at end of image */
	gd->fdt_blob = (void *)(_end);
	/* Check whether we have a valid FDT or not. */
	if (fdtdec_prepare_fdt()) {
		panic("** CONFIG_OF_CONTROL defined but no FDT - please see "
			"doc/README.fdt-control");
	}
#endif /* CONFIG_OF_CONTROL */

	/* assert reset to all except L4WD0 and L4TIMER0 */
	reset_assert_all_peripherals_except_l4wd0_l4timer0();

	/* Initialize the timer */
	timer_init();

	/* configuring the clock based on handoff */
	cm_basic_init(gd->fdt_blob);
	WATCHDOG_RESET();

	/* Setting serial port based on 1st encounter UART in handoff */
	com_port = uart_com_port(gd->fdt_blob);

	if (com_port) {
#ifdef CONFIG_SYS_NS16550_COM1
		set_serial_port(1, com_port);
#elif CONFIG_SYS_NS16550_COM2
		set_serial_port(2, com_port);
#elif CONFIG_SYS_NS16550_COM3
		set_serial_port(3, com_port);
#elif CONFIG_SYS_NS16550_COM4
		set_serial_port(4, com_port);
#elif CONFIG_SYS_NS16550_COM5
		set_serial_port(5, com_port);
#elif CONFIG_SYS_NS16550_COM6
		set_serial_port(6, com_port);
#endif
	}

	config_dedicated_pins(gd->fdt_blob);
	WATCHDOG_RESET();

	/* configure the Reset Manager */
	reset_deassert_dedicated_peripherals();

	if (is_external_fpga_config(gd->fdt_blob) ||
		CONFIG_UBOOT_EXE_ON_FPGA) {
		while (!is_fpgamgr_user_mode())
			;

		if (is_regular_boot()) {
			set_regular_boot(false);
/* Skip RAM Boot when booting from FPGA */
#if (CONFIG_UBOOT_EXE_ON_FPGA == 0)
			/* Disable RAM boot */
			enable_ram_boot(0, CONFIG_SYS_TEXT_BASE);
#endif
		}
		else {
			set_regular_boot(true);
			udelay(10000);
/* Skip RAM Boot when booting from FPGA */
#if (CONFIG_UBOOT_EXE_ON_FPGA == 0)
			/* Enable RAM boot */
			enable_ram_boot(RAM_BOOT_EN_MAGIC,
				 CONFIG_SYS_TEXT_BASE);
#endif
#if defined(CONFIG_CADENCE_QSPI_CFF)
			qspi_software_reset();
#endif
			reset_cpu(0);
		}

		config_pins(gd->fdt_blob, "shared");
		config_pins(gd->fdt_blob, "fpga");
		reset_deassert_shared_connected_peripherals();
		reset_deassert_fpga_connected_peripherals();
	}

	v7_outer_cache_configuration();
}
