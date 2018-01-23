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

#include <drv_types.h>
#include <hal_data.h>
#include <linux/acpi.h>

#define TXPWR_LMT_SAR_CHAIN_NUM 2

enum txpwr_lmt_sar_sbs {
	TXPWR_LMT_SAR_CH1_14, /* 2.4G */
	TXPWR_LMT_SAR_CH36_64, /* 5.15~5.35G */
	TXPWR_LMT_SAR_UND1, /* 5.35~5.47G */
	TXPWR_LMT_SAR_CH100_144, /* 5.47~5.725G */
	TXPWR_LMT_SAR_CH149_165, /* 5.725~5.95G */
	TXPWR_LMT_SAR_SB_NUM,
};

struct txpwr_lmt_sar {
	s8 val[TXPWR_LMT_SAR_CHAIN_NUM][TXPWR_LMT_SAR_SB_NUM]; /* Q3 */
};

#define ACPI_WRDS_METHOD        "WRDS"
#define ACPI_WRDS_WIFI          (0x07)
#define ACPI_WRDS_TABLE_SIZE    (TXPWR_LMT_SAR_CHAIN_NUM * TXPWR_LMT_SAR_SB_NUM)

#ifdef CONFIG_ACPI
bool rtl_sar_get_wrds(_adapter *adapter, union acpi_object *wrds,
		      struct txpwr_lmt_sar *limit_sar)
{
	union acpi_object *data_pkg;
	u32 i;
	int path, sb, idx;

	if (wrds->type != ACPI_TYPE_PACKAGE ||
	    wrds->package.count < 2 ||
	    wrds->package.elements[0].type != ACPI_TYPE_INTEGER ||
	    wrds->package.elements[0].integer.value != 0) {
		RTW_INFO("SAR: Unsupported wrds structure\n");
		return false;
	}

	/* loop through all the packages to find the one for WiFi */
	for (i = 1; i < wrds->package.count; i++) {
		union acpi_object *domain;

		data_pkg = &wrds->package.elements[i];

		/* Skip anything that is not a package with the right
		 * amount of elements (i.e. domain_type,
		 * enabled/disabled plus the sar table size.
		 */
		if (data_pkg->type != ACPI_TYPE_PACKAGE ||
		    data_pkg->package.count != ACPI_WRDS_TABLE_SIZE + 2)
			continue;

		domain = &data_pkg->package.elements[0];
		if (domain->type == ACPI_TYPE_INTEGER &&
		    domain->integer.value == ACPI_WRDS_WIFI)
			break;

		data_pkg = NULL;
	}

	if (!data_pkg)
		return false;

	if (data_pkg->package.elements[1].type != ACPI_TYPE_INTEGER)
		return false;

	/* WiFiSarEnable 0: ignore BIOS config; 1: use BIOS config */
	if (data_pkg->package.elements[1].integer.value == 0)
		return false;

	/* read elements[2~11] */
	idx = 2;
	for (path = 0; path < TXPWR_LMT_SAR_CHAIN_NUM; path++)
		for (sb = 0; sb < TXPWR_LMT_SAR_SB_NUM; sb++) {
			union acpi_object *entry;

			entry = &data_pkg->package.elements[idx++];
			if ((entry->type != ACPI_TYPE_INTEGER) ||
			    (entry->integer.value > U8_MAX))
				return false;

			limit_sar->val[path][sb] = entry->integer.value;
		}

	return true;
}

static bool smbios_load_sar_limit_table(_adapter *adapter,
					struct txpwr_lmt_sar *limit_sar)
{
	struct dvobj_priv *dvobj = adapter_to_dvobj((adapter));
	struct device *dev = dvobj_to_dev(dvobj);
	acpi_handle root_handle;
	acpi_handle handle;
	acpi_status status;
	struct acpi_buffer wrds = {ACPI_ALLOCATE_BUFFER, NULL};
	s32 ret;

	/* Check device handler */
	root_handle = ACPI_HANDLE(dev);
	if (!root_handle) {
		RTW_INFO("SAR: Could not retireve root port ACPI handle\n");
		return false;
	}

	/* Get method's handler */
	status = acpi_get_handle(root_handle, (acpi_string)ACPI_WRDS_METHOD,
				 &handle);
	if (ACPI_FAILURE(status)) {
		RTW_INFO("SAR: WRDS method not found\n");
		return false;
	}

	/* Call WRDS with no argument */
	status = acpi_evaluate_object(handle, NULL, NULL, &wrds);
	if (ACPI_FAILURE(status)) {
		RTW_INFO("SAR: WRDS invocation failed (0x%x)\n", status);
		return false;
	}

	/* Process WRDS returned wrapper */
	ret = rtl_sar_get_wrds(adapter, wrds.pointer, limit_sar);
	kfree(wrds.pointer);

	return true;
}
#else
static bool smbios_load_sar_limit_table(_adapter *adapter,
					struct txpwr_lmt_sar *limit_sar)
{
	return false;
}
#endif /* CONFIG_ACPI */

void smbios_init(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	struct txpwr_lmt_sar *limit_sar;

	limit_sar = (struct txpwr_lmt_sar *)rtw_zvmalloc(sizeof(*limit_sar));

	if (!limit_sar)
		return;

	if (!smbios_load_sar_limit_table(adapter, limit_sar)) {
		rtw_vmfree((u8 *)limit_sar, sizeof(*limit_sar));
		return;
	}

	rfctl->txpwr_lmt_sar = limit_sar;
}

void smbios_free(_adapter *adapter)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	struct txpwr_lmt_sar *limit_sar = rfctl->txpwr_lmt_sar;

	if (!limit_sar)
		return;

	rtw_vmfree((u8 *)limit_sar, sizeof(*limit_sar));
	limit_sar = NULL;
}

int smbios_info(_adapter *adapter, struct seq_file *m)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	struct txpwr_lmt_sar *limit_sar = rfctl->txpwr_lmt_sar;
	int i, j;

	seq_printf(m, "SAR table: %p\n", limit_sar);

	if (!limit_sar)
		goto done;

	for (i = 0; i < TXPWR_LMT_SAR_CHAIN_NUM; i++) {
		seq_printf(m, "Chain %d: ", i);
		for (j = 0; j < TXPWR_LMT_SAR_SB_NUM; j++)
			seq_printf(m, "%#02x ", limit_sar->val[i][j]);
		seq_puts(m, "\n");
	}

done:
	return 0;
}

s8 smbios_get_tx_power_limit_sar(_adapter *adapter, u8 rfpath, BAND_TYPE band,
				 u8 cch)
{
	struct rf_ctl_t *rfctl = adapter_to_rfctl(adapter);
	struct txpwr_lmt_sar *limit_sar = rfctl->txpwr_lmt_sar;
	enum txpwr_lmt_sar_sbs sb;
	s8 limit_q3;

	if (!limit_sar)
		return MAX_POWER_INDEX;

	if (rfpath >= TXPWR_LMT_SAR_CHAIN_NUM)
		rfpath = 0;

	if (band == BAND_ON_2_4G) {
		sb = TXPWR_LMT_SAR_CH1_14;
	} else if (band == BAND_ON_5G) {
		if (cch <= 64)
			sb = TXPWR_LMT_SAR_CH36_64;
		else if (cch >= 100 && cch <= 144)
			sb = TXPWR_LMT_SAR_CH100_144;
		else if (cch >= 149)
			sb = TXPWR_LMT_SAR_CH149_165;
		else
			return MAX_POWER_INDEX;
	} else {
		return MAX_POWER_INDEX;
	}

	limit_q3 = limit_sar->val[rfpath][sb];

	return limit_q3 >> 2;	/* Q3->Q0 and dBm->pwr_idx */
}
