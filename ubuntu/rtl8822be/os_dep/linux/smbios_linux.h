/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#ifndef __SMBIOS_LINUX_H__
#define __SMBIOS_LINUX_H__

#ifdef CONFIG_TXPWR_LIMIT_SMBIOS_SAR
void smbios_init(_adapter *adapter);
void smbios_free(_adapter *adapter);
int smbios_info(_adapter *adapter, struct seq_file *m);
s8 smbios_get_tx_power_limit_sar(_adapter *adapter, u8 rfpath, BAND_TYPE band,
				 u8 cch);
#else
#define smbios_init(x)
#define smbios_free(x)
#endif

#endif /* __SMBIOS_LINUX_H__ */
