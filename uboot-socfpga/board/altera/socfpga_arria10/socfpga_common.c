/*
 * Copyright (C) 2014 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <watchdog.h>
#include <asm/arch/system_manager.h>
#include <asm/arch/reset_manager.h>
#include <phy.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include "../../../drivers/net/designware.h"
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define ATSHA204_COMMAND	0x03
#define ATSHA204_READ_CMD	0x02

#define ATSHA204_CONFIG_ZONE	0x00
#define ATSHA204_OTP_ZONE	0x01
#define ATSHA204_DATA_ZONE	0x02

#define ATSHA204_READ32_BYTES_FLAG	(1<<7)

/* i2c controller defines */
#define IC_SDA_HOLD_OFFSET	0x007c
#define IC_ENABLE_OFFSET	0x006c
#define IC_ENABLE		0x0001

u16 atsha204_crc16(const u8 *buf, const u8 len)
{
	u8 i;
	u16 crc16 = 0;

	for (i = 0; i < len; i++) {
		u8 shift;

		for (shift = 0x01; shift > 0x00; shift <<= 1) {
			u8 data_bit = (buf[i] & shift) ? 1 : 0;
			u8 crc_bit = crc16 >> 15;

			crc16 <<= 1;

			if ((data_bit ^ crc_bit) != 0)
				crc16 ^= 0x8005;
		}
	}

	return crc16;
}

struct __attribute__((packed, aligned(1))) atsha204_read_command {
	u8 count;
	u8 opcode;
	u8 param1;
	u16 param2;
	u16 checksum;
};

int atsha204_send_read_cmd(u8 i2c_address, u8 zone, u16 address, u8 read_block) {

	int ret;
	u16 crc;
	struct atsha204_read_command packet;

	packet.count = sizeof(struct atsha204_read_command);
	packet.opcode = ATSHA204_READ_CMD;
	packet.param1 = zone;
	if(read_block) packet.param1 |= ATSHA204_READ32_BYTES_FLAG;
	packet.param2 = address;

	crc = atsha204_crc16((u8*)(&packet), sizeof(struct atsha204_read_command) - 2);
	packet.checksum = crc;

	ret = i2c_write(i2c_address,
		  ATSHA204_COMMAND,
		  1,
		  (u8*)&packet,
		  sizeof(struct atsha204_read_command));
	if(ret) {
		printf("Writing failed \n");
		return ret;
	}
	/* reading may take up to 4 ms */
	udelay(4000);

	return 0;
}

int atsha204_wakeup(u8 i2c_address) {

	u8 wake_cmd = 0x0;
	int ret = 0;

	ret = i2c_write(i2c_address,
		  0,
		  0,
		  &wake_cmd,
		  1);

	if(ret) return ret;

	/* wait for the chip to wake up */
	udelay(2000);
	return 0;

}

int atsha204_read_data(u8 i2c_address, u8* buffer, u8 len) {

	int ret = 0;
	u8 first_word[8];
	u8 msg_len;
	u8 i;

	if (len < 8) return 12;
	/* read the first 4 bytes from the device*/
	ret = i2c_read(i2c_address,
			0,
			0,
			first_word,
			8);

	if(ret) return ret;

	/* the first transferred byte is total length of the msg */
	msg_len = first_word[0];

	for(i = 0; i < 4; i++) buffer[i] = first_word[i+1];

	msg_len -= 4;

	return msg_len;
}

int atsha204_read_otp_register(u8 i2c_address, u8 reg, u8* buffer) {

	u8 data[8];
	u8 i;
	int ret;

	ret = atsha204_wakeup(i2c_address);
	if(ret) return ret;

	ret = atsha204_send_read_cmd(i2c_address, ATSHA204_OTP_ZONE, reg, 0);
	if(ret) return ret;

	/* Attempt to read the register */

	ret = atsha204_read_data(i2c_address, data, 8);
	if(ret < 0) return ret;

	for(i = 0; i < 4; i++) buffer[i] = data[i];

	return 0;
}

int atsha204_get_mac(u8 i2c_address, u8* buffer) {

	int ret;
	u8 data[4];
	u8 i;

	ret = atsha204_read_otp_register(i2c_address, 4, data);
	if(ret) return ret;
	else for(i = 0; i < 4; i++) buffer[i] = data[i];

	ret = atsha204_read_otp_register(i2c_address, 5, data);
	if(ret) return ret;
	else {
		buffer[4] = data[0];
		buffer[5] = data[1];
	}
	return 0;
}

struct eeprom_mem {
	u8 i2c_addr;
	int (*mac_reader)(u8 i2c_address, u8* buffer);
	int (*wakeup)(u8 i2c_address);
};

static struct eeprom_mem eeproms[] = {
	{ .i2c_addr = 0x64,
	  .mac_reader = atsha204_get_mac,
	  .wakeup = atsha204_wakeup,
	},
};


/*
 * Initialization function which happen at early stage of c code
 */
int board_early_init_f(void)
{
	WATCHDOG_RESET();
	return 0;
}

/*
 * Miscellaneous platform dependent initialisations
 */
int board_init(void)
{
	/* adress of boot parameters for ATAG (if ATAG is used) */
	gd->bd->bi_boot_params = 0x00000100;
	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_I2C
	int i;
	u8 hwaddr[6] = {0, 0, 0, 0, 0, 0};
	u32 hwaddr_h;
	char hwaddr_str[16];
	bool hwaddr_set;

	hwaddr_set = false;

	i2c_init(0,0);
	if(getenv("ethaddr") == NULL) {
		for (i = 0; i < ARRAY_SIZE(eeproms); i++) {

			if(eeproms[i].wakeup)
				eeproms[i].wakeup(eeproms[i].i2c_addr);

			/* Probe the chip */
			if(i2c_probe(eeproms[i].i2c_addr))
				continue;

			if(eeproms[i].mac_reader(eeproms[i].i2c_addr, hwaddr))
				continue;

			sprintf(hwaddr_str,
				"%02X:%02X:%02X:%02X:%02X:%02X",
				hwaddr[0],
				hwaddr[1],
				hwaddr[2],
				hwaddr[3],
				hwaddr[4],
				hwaddr[5]);

			/* Check if the value is a valid mac registered for
			* Enclustra  GmbH */
			hwaddr_h = hwaddr[0] | hwaddr[1] << 8 | hwaddr[2] << 16;
			if ((hwaddr_h & 0xFFFFFF) != ENCLUSTRA_MAC)
				continue;

			/* Set the actual env variable */
			setenv("ethaddr", hwaddr_str);
			hwaddr_set = true;
			break;
		}
		if (!hwaddr_set)
			setenv("ethaddr", ENCLUSTRA_ETHADDR_DEFAULT);
	}
#endif
	return 0;
}
#endif

ulong
socfpga_get_emac_control(unsigned long emacbase)
{
	ulong base = 0;
	switch (emacbase) {
		case SOCFPGA_EMAC0_ADDRESS:
			base = CONFIG_SYSMGR_EMAC0_CTRL;
			break;
		case SOCFPGA_EMAC1_ADDRESS:
			base = CONFIG_SYSMGR_EMAC1_CTRL;
			break;
		case SOCFPGA_EMAC2_ADDRESS:
			base = CONFIG_SYSMGR_EMAC2_CTRL;
			break;
		default:
			error("bad emacbase %lx\n", emacbase);
			hang();
			break;
	}
	return base;
}

ulong
socfpga_get_phy_mode(ulong phymode)
{
	ulong val;
	switch (phymode) {
		case PHY_INTERFACE_MODE_GMII:
			val = SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_GMII_MII;
			break;
		case PHY_INTERFACE_MODE_MII:
			val = SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_GMII_MII;
			break;
		case PHY_INTERFACE_MODE_RGMII:
			val = SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RGMII;
			break;
		case PHY_INTERFACE_MODE_RMII:
			val = SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RMII;
			break;
		default:
			error("bad phymode %lx\n", phymode);
			hang();
			break;
	}
	return val;
}

int is_ksz9031(struct phy_device *phydev)
{
	unsigned short phyid1;
	unsigned short phyid2;

	phyid1 = phy_read(phydev, MDIO_DEVAD_NONE, MII_PHYSID1);
	phyid2 = phy_read(phydev, MDIO_DEVAD_NONE, MII_PHYSID2);

	phyid2 = phyid2 & MICREL_KSZ9031_PHYID2_REVISION_MASK;

	debug("phyid1 %04x, phyid2 %04x\n", phyid1, phyid2);

	if ((phyid1 == MICREL_KSZ9031_PHYID1) &&
	    (phyid2 == MICREL_KSZ9031_PHYID2))
		return 1;
	return 0;
}



int board_phy_config(struct phy_device *phydev)
{
	int reg;
	int devad = MDIO_DEVAD_NONE;

	reg = phy_read(phydev, devad, MII_BMCR);
	if (reg < 0) {
		debug("PHY status read failed\n");
		return -1;
	}

	if (reg & BMCR_PDOWN) {
		reg &= ~BMCR_PDOWN;
		if (phy_write(phydev, devad, MII_BMCR, reg) < 0) {
			debug("PHY power up failed\n");
			return -1;
		}
		udelay(1500);
	}


	if (is_ksz9031(phydev)) {
		unsigned short reg4;
		unsigned short reg5;
		unsigned short reg6;
		unsigned short reg8;

		reg4 = getenv_ulong("ksz9031-rgmii-ctrl-skew", 16, 0x77);
		reg5 = getenv_ulong("ksz9031-rgmii-rxd-skew", 16, 0x7777);
		reg6 = getenv_ulong("ksz9031-rgmii-txd-skew", 16, 0x7777);
		reg8 = getenv_ulong("ksz9031-rgmii-clock-skew", 16, 0x1ef);

		ksz9031_phy_extended_write(phydev, 2, 4,
					   MII_KSZ9031_MOD_DATA_NO_POST_INC,
					   reg4);
		ksz9031_phy_extended_write(phydev, 2, 5,
					   MII_KSZ9031_MOD_DATA_NO_POST_INC,
					   reg5);
		ksz9031_phy_extended_write(phydev, 2, 6,
					   MII_KSZ9031_MOD_DATA_NO_POST_INC,
					   reg6);
		ksz9031_phy_extended_write(phydev, 2, 8,
					   MII_KSZ9031_MOD_DATA_NO_POST_INC,
					   reg8);

		ksz9031_phy_extended_write(phydev, 0,
			MII_KSZ9031RN_FLP_BURST_TX_HI,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x0006);

		ksz9031_phy_extended_write(phydev, 0,
			MII_KSZ9031RN_FLP_BURST_TX_LO,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x1A80);
	}
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#ifdef CONFIG_DESIGNWARE_ETH
/* We know all the init functions have been run now */
int board_eth_init(bd_t *bis)
{
	ulong emacctrlreg;
	ulong reg32;

	emacctrlreg = socfpga_get_emac_control(CONFIG_EMAC_BASE);

	/* Put the emac we're using into reset. 
	 * This is required before configuring the PHY interface
	 */
	emac_manage_reset(CONFIG_EMAC_BASE, 1);

	reg32 = readl(emacctrlreg);
	reg32 &= ~SYSMGR_EMACGRP_CTRL_PHYSEL_MASK;

	reg32 |= socfpga_get_phy_mode(CONFIG_PHY_INTERFACE_MODE);

	writel(reg32, emacctrlreg);

	/* Now that the PHY interface is configured, release
	 * the EMAC from reset. Delay a little bit afterwards
	 * just to make sure reset is completed before first access
	 * to EMAC CSRs. 
	 */
	emac_manage_reset(CONFIG_EMAC_BASE, 0);

	/* initialize and register the emac */
	return designware_initialize(CONFIG_EMAC_BASE,
					CONFIG_PHY_INTERFACE_MODE);
}
#endif

