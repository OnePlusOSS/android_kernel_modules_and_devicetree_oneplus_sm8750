// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[STRATEGY_PPS_UFCS_V2]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/string.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_strategy.h>

enum puc_soc_range {
	PUC_BATT_CURVE_SOC_RANGE_MIN = 0,
	PUC_BATT_CURVE_SOC_RANGE_LOW,
	PUC_BATT_CURVE_SOC_RANGE_MID_LOW,
	PUC_BATT_CURVE_SOC_RANGE_MID,
	PUC_BATT_CURVE_SOC_RANGE_MID_HIGH,
	PUC_BATT_CURVE_SOC_RANGE_HIGH,
	PUC_BATT_CURVE_SOC_RANGE_MAX,
	PUC_BATT_CURVE_SOC_RANGE_INVALID = PUC_BATT_CURVE_SOC_RANGE_MAX,
};

enum puc_temp_range {
	PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COLD = 0,
	PUC_BATT_CURVE_TEMP_RANGE_COOL,
	PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COOL,
	PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COOL_HIGH,
	PUC_BATT_CURVE_TEMP_RANGE_NORMAL_LOW_PRE,
	PUC_BATT_CURVE_TEMP_RANGE_NORMAL_LOW,
	PUC_BATT_CURVE_TEMP_RANGE_NORMAL_HIGH,
	PUC_BATT_CURVE_TEMP_RANGE_WARM,
	PUC_BATT_CURVE_TEMP_RANGE_MAX,
	PUC_BATT_CURVE_TEMP_RANGE_INVALID = PUC_BATT_CURVE_TEMP_RANGE_MAX,
};

struct puc_strategy_soc_curves {
	struct puc_strategy_temp_curves temp_curves[PUC_BATT_CURVE_TEMP_RANGE_MAX];
};

struct puc_strategy {
	struct oplus_chg_strategy strategy;
	struct puc_strategy_soc_curves soc_curves[PUC_BATT_CURVE_SOC_RANGE_MAX];
	uint32_t soc_range_data[PUC_BATT_CURVE_SOC_RANGE_MAX + 1];
	int32_t temp_range_data[PUC_BATT_CURVE_TEMP_RANGE_MAX + 1];
	uint32_t temp_type;
	int32_t iterm_data[PUC_BATT_CURVE_TEMP_RANGE_MAX];

	struct puc_strategy_temp_curves *curve;
	int curr_level;
	unsigned long timeout;
	unsigned long over_time;
	int temp_region;
	int allow_soc;
	int iterm;
	int temp_range_cnt;
	int temp_range_bound_cnt;
	int temp_range_map[PUC_BATT_CURVE_TEMP_RANGE_MAX];
	bool temp_range_map_inited;
};

#define PUC_DATA_SIZE	sizeof(struct puc_strategy_data)

static const char * const puc_strategy_soc[] = {
	[PUC_BATT_CURVE_SOC_RANGE_MIN]		= "strategy_soc_range_min",
	[PUC_BATT_CURVE_SOC_RANGE_LOW]		= "strategy_soc_range_low",
	[PUC_BATT_CURVE_SOC_RANGE_MID_LOW]	= "strategy_soc_range_mid_low",
	[PUC_BATT_CURVE_SOC_RANGE_MID]		= "strategy_soc_range_mid",
	[PUC_BATT_CURVE_SOC_RANGE_MID_HIGH]	= "strategy_soc_range_mid_high",
	[PUC_BATT_CURVE_SOC_RANGE_HIGH]		= "strategy_soc_range_high",
};

static const char * const puc_strategy_temp[] = {
	[PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COLD]	= "strategy_temp_little_cold",
	[PUC_BATT_CURVE_TEMP_RANGE_COOL]	= "strategy_temp_cool",
	[PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COOL]	= "strategy_temp_little_cool",
	[PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COOL_HIGH]	= "strategy_temp_little_cool_high",
	[PUC_BATT_CURVE_TEMP_RANGE_NORMAL_LOW_PRE]	= "strategy_temp_normal_low_pre",
	[PUC_BATT_CURVE_TEMP_RANGE_NORMAL_LOW]	= "strategy_temp_normal_low",
	[PUC_BATT_CURVE_TEMP_RANGE_NORMAL_HIGH]	= "strategy_temp_normal_high",
	[PUC_BATT_CURVE_TEMP_RANGE_WARM]	= "strategy_temp_warm",
};

static bool puc_temp_range_is_legacy_required(int temp_idx)
{
	switch (temp_idx) {
	case PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COLD:
	case PUC_BATT_CURVE_TEMP_RANGE_COOL:
	case PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COOL:
	case PUC_BATT_CURVE_TEMP_RANGE_LITTLE_COOL_HIGH:
	case PUC_BATT_CURVE_TEMP_RANGE_NORMAL_LOW:
	case PUC_BATT_CURVE_TEMP_RANGE_NORMAL_HIGH:
	case PUC_BATT_CURVE_TEMP_RANGE_WARM:
		return true;
	default:
		return false;
	}
}

static struct oplus_mms *comm_topic;
static struct oplus_mms *gauge_topic;

__maybe_unused static bool is_comm_topic_available(void)
{
	if (!comm_topic)
		comm_topic = oplus_mms_get_by_name("common");
	return !!comm_topic;
}

__maybe_unused static bool is_gauge_topic_available(void)
{
	if (!gauge_topic)
		gauge_topic = oplus_mms_get_by_name("gauge");
	return !!gauge_topic;
}

static int __read_unsigned_data_from_node(struct device_node *node,
					  const char *prop_str, u32 *addr,
					  int len_max)
{
	int rc = 0, length;

	if (!node || !prop_str || !addr) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		chg_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;

	if (length != len_max) {
		chg_err("entries(%d) num error, only %d allowed\n", length,
			len_max);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, prop_str, (u32 *)addr, length);
	if (rc < 0) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	return length;
}

static int puc_read_temp_range_by_node(struct device_node *node,
				       struct puc_strategy *puc,
				       int *temp_range_cnt)
{
	int rc;
	int temp_range_bound_cnt;

	temp_range_bound_cnt = of_property_count_elems_of_size(node, "oplus,temp_range", sizeof(s32));
	if (temp_range_bound_cnt <= 1 || temp_range_bound_cnt > PUC_BATT_CURVE_TEMP_RANGE_MAX + 1) {
		chg_err("oplus,temp_range entries(%d) num error, need >= 2 and <= %d\n",
			temp_range_bound_cnt, PUC_BATT_CURVE_TEMP_RANGE_MAX + 1);
		return -EINVAL;
	}
	rc = of_property_read_u32_array(node, "oplus,temp_range",
					(u32 *)puc->temp_range_data,
					temp_range_bound_cnt);
	if (rc < 0)
		return rc;

	puc->temp_range_bound_cnt = temp_range_bound_cnt;
	puc->temp_range_cnt = temp_range_bound_cnt - 1;
	*temp_range_cnt = puc->temp_range_cnt;

	return 0;
}

static int puc_read_iterm_by_node(struct device_node *node, int temp_range_cnt,
				  int32_t *iterm_buf, int *iterm_len)
{
	int length;
	int rc;

	*iterm_len = 0;
	length = of_property_count_elems_of_size(node, "oplus,iterm", sizeof(s32));
	if (length < 0) {
		chg_err("get oplus,iterm count error, rc=%d\n", length);
		return 0;
	}
	if (length < temp_range_cnt) {
		chg_err("oplus,iterm entries(%d) num error, need >= %d\n",
			length, temp_range_cnt);
		return -EINVAL;
	}

	*iterm_len = min(length, (int)PUC_BATT_CURVE_TEMP_RANGE_MAX);
	rc = of_property_read_u32_array(node, "oplus,iterm", (u32 *)iterm_buf, *iterm_len);
	if (rc < 0) {
		chg_err("get oplus,iterm property error, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int puc_build_temp_range_map(struct puc_strategy *puc,
				    struct device_node *soc_node,
				    int *temp_range_cnt)
{
	int j;
	int length;
	int temp_prop_cnt = 0;

	for (j = 0; j < PUC_BATT_CURVE_TEMP_RANGE_MAX; j++) {
		length = of_property_count_elems_of_size(
			soc_node, puc_strategy_temp[j], sizeof(u32));
		if (length < 0) {
			if (puc_temp_range_is_legacy_required(j)) {
				chg_err("can't find legacy %s property, rc=%d\n",
					puc_strategy_temp[j], length);
				return length;
			}
			continue;
		}
		if (temp_prop_cnt >= PUC_BATT_CURVE_TEMP_RANGE_MAX) {
			chg_err("temp_range_map overflow, cnt=%d\n", temp_prop_cnt);
			return -EINVAL;
		}
		puc->temp_range_map[temp_prop_cnt++] = j;
	}
	if (temp_prop_cnt <= 0)
		return -ENODEV;
	if (temp_prop_cnt != *temp_range_cnt) {
		chg_err("temp_range property num(%d) mismatch range num(%d)\n",
			temp_prop_cnt, *temp_range_cnt);
		return -EINVAL;
	}

	puc->temp_range_map_inited = true;
	return 0;
}

static int puc_read_temp_curve_by_node(struct device_node *soc_node, int temp_idx,
				       struct puc_strategy_temp_curves *curve)
{
	int length;
	int rc;

	length = of_property_count_elems_of_size(
		soc_node, puc_strategy_temp[temp_idx], sizeof(u32));
	if (length < 0) {
		chg_err("can't find %s property, rc=%d\n",
			puc_strategy_temp[temp_idx], length);
		return length;
	}
	rc = length * sizeof(u32);
	if (rc % PUC_DATA_SIZE != 0) {
		chg_err("buf size does not meet the requirements, size=%d\n", rc);
		return -EINVAL;
	}

	curve->num = rc / PUC_DATA_SIZE;
	curve->data = kzalloc(rc, GFP_KERNEL);
	if (curve->data == NULL) {
		chg_err("alloc strategy data memory error\n");
		return -ENOMEM;
	}

	rc = of_property_read_u32_array(
			soc_node, puc_strategy_temp[temp_idx],
			(u32 *)curve->data, length);
	if (rc < 0) {
		chg_err("read %s property error, rc=%d\n",
			puc_strategy_temp[temp_idx], rc);
		kfree(curve->data);
		curve->data = NULL;
		return rc;
	}

	return 0;
}

static void puc_read_temp_type_by_node(struct device_node *node, struct puc_strategy *puc)
{
	u32 data;
	int rc;

	rc = of_property_read_u32(node, "oplus,temp_type", &data);
	if (rc < 0) {
		chg_err("oplus,temp_type reading failed, rc=%d\n", rc);
		puc->temp_type = STRATEGY_USE_SHELL_TEMP;
		return;
	}
	puc->temp_type = (uint32_t)data;
}

static int puc_read_soc_range_by_node(struct device_node *node, struct puc_strategy *puc)
{
	return __read_unsigned_data_from_node(node, "oplus,soc_range",
					      (u32 *)puc->soc_range_data,
					      PUC_BATT_CURVE_SOC_RANGE_MAX + 1);
}

static int puc_fill_iterm_data(struct puc_strategy *puc, const int32_t *iterm_buf,
			       int iterm_len, int temp_range_cnt)
{
	int j;
	int temp_idx;

	if (iterm_len > 0 && iterm_len < temp_range_cnt) {
		chg_err("oplus,iterm entries(%d) num error, need >= %d\n",
			iterm_len, temp_range_cnt);
		return -EINVAL;
	}
	if (iterm_len == 0)
		return 0;

	for (j = 0; j < temp_range_cnt; j++) {
		temp_idx = puc->temp_range_map[j];
		if (temp_idx < 0 || temp_idx >= PUC_BATT_CURVE_TEMP_RANGE_MAX) {
			chg_err("invalid temp_idx=%d\n", temp_idx);
			return -EINVAL;
		}
		puc->iterm_data[temp_idx] = iterm_buf[j];
	}
	return 0;
}

static int puc_load_curves_by_node(struct puc_strategy *puc,
				   struct device_node *soc_node,
				   int soc_idx, int temp_range_cnt)
{
	int j;
	int rc;
	int temp_idx;

	for (j = 0; j < temp_range_cnt; j++) {
		temp_idx = puc->temp_range_map[j];
		rc = puc_read_temp_curve_by_node(
			soc_node, temp_idx, &puc->soc_curves[soc_idx].temp_curves[temp_idx]);
		if (rc < 0)
			return rc;
	}
	return 0;
}

static int puc_strategy_get_soc(struct puc_strategy *puc, int *soc)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_comm_topic_available()) {
		chg_err("common topic not found\n");
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_UI_SOC,
				     &data, false);
	if (rc < 0) {
		chg_err("can't get ui soc, rc=%d\n", rc);
		return rc;
	}
	*soc = data.intval;

	return 0;
}

static int puc_strategy_get_vbat(struct puc_strategy *puc, int *vbat)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_gauge_topic_available()) {
		chg_err("gauge topic not found\n");
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_VOL_MAX,
				     &data, true);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		return rc;
	}
	*vbat = data.intval;

	return 0;
}

static int puc_strategy_get_temp(struct puc_strategy *puc, int *temp)
{
	union mms_msg_data data = { 0 };
	int rc;

	switch (puc->temp_type) {
	case STRATEGY_USE_BATT_TEMP:
		if (!is_gauge_topic_available()) {
			chg_err("gauge topic not found\n");
			return -ENODEV;
		}
		rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_TEMP,
					     &data, true);
		if (rc < 0) {
			chg_err("can't get battery temp, rc=%d\n", rc);
			return rc;
		}

		*temp = data.intval;
		break;
	case STRATEGY_USE_SHELL_TEMP:
		if (!is_comm_topic_available()) {
			chg_err("common topic not found\n");
			return -ENODEV;
		}
		rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_SHELL_TEMP,
					     &data, false);
		if (rc < 0) {
			chg_err("can't get shell temp, rc=%d\n", rc);
			return rc;
		}

		*temp = data.intval;
		break;
	default:
		chg_err("not support temp type, type=%d\n", puc->temp_type);
		return -EINVAL;
	}

	return 0;
}

static enum puc_soc_range
puc_get_soc_region(struct puc_strategy *puc)
{
	int soc;
	enum puc_soc_range soc_region = PUC_BATT_CURVE_SOC_RANGE_INVALID;
	int i;
	int rc;

	rc = puc_strategy_get_soc(puc, &soc);
	if (rc < 0) {
		chg_err("can't get soc, rc=%d\n", rc);
		return PUC_BATT_CURVE_SOC_RANGE_INVALID;
	}

	for (i = 1; i < PUC_BATT_CURVE_SOC_RANGE_MAX + 1; i++) {
		if (soc <= puc->soc_range_data[i]) {
			soc_region = i - 1;
			break;
		}
	}

	return soc_region;
}

static enum puc_soc_range
puc_get_fastchg_allow_soc_region(struct puc_strategy *puc)
{
	int soc;
	enum puc_soc_range soc_region = PUC_BATT_CURVE_SOC_RANGE_INVALID;
	int i;
	int rc;

	rc = puc_strategy_get_soc(puc, &soc);
	if (rc < 0) {
		chg_err("can't get soc, rc=%d\n", rc);
		return PUC_BATT_CURVE_SOC_RANGE_INVALID;
	}

	/* To prevent the issue of SOC_RANGE_INVALID error when ui_soc suddenly rise, use soc_region of allow_soc */
	if (abs(puc->allow_soc - soc) > 1)
		return PUC_BATT_CURVE_SOC_RANGE_INVALID;

	for (i = 1; i < PUC_BATT_CURVE_SOC_RANGE_MAX + 1; i++) {
		if (puc->allow_soc <= puc->soc_range_data[i]) {
			soc_region = i - 1;
			break;
		}
	}
	chg_err("use allow_soc=%d soc_region=%d\n", puc->allow_soc, soc_region);
	return soc_region;
}

static enum puc_temp_range
puc_get_temp_region(struct puc_strategy *puc)
{
	int temp;
	enum puc_temp_range temp_region = PUC_BATT_CURVE_TEMP_RANGE_INVALID;
	int i;
	int rc;
	int temp_region_idx = -1;

	rc = puc_strategy_get_temp(puc, &temp);
	if (rc < 0) {
		chg_err("can't get temp, rc=%d\n", rc);
		return PUC_BATT_CURVE_TEMP_RANGE_INVALID;
	}

	for (i = 0; i < puc->temp_range_bound_cnt; i++) {
		if (temp < puc->temp_range_data[i]) {
			if (i != 0)
				temp_region_idx = i - 1;
			break;
		}
	}
	if (temp_region_idx >= 0 && temp_region_idx < puc->temp_range_cnt)
		temp_region = puc->temp_range_map[temp_region_idx];
	if((puc->temp_region != temp_region) && (puc->temp_region != PUC_BATT_CURVE_TEMP_RANGE_INVALID)) {
		chg_err("puc->temp_region != temp_region use puc->temp_region\n");
		return puc->temp_region;
	} else {
		return temp_region;
	}
}

static struct oplus_chg_strategy *
puc_strategy_alloc(unsigned char *buf, size_t size)
{
	return ERR_PTR(-ENOTSUPP);
}

static struct oplus_chg_strategy *
puc_strategy_alloc_by_node(struct device_node *node)
{
	struct puc_strategy *puc;
	int rc;
	int i;
	int j;
	struct device_node *soc_node;
	int temp_range_cnt;
	int iterm_len = 0;
	int32_t iterm_buf[PUC_BATT_CURVE_TEMP_RANGE_MAX];

	if (node == NULL) {
		chg_err("node is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	puc = kzalloc(sizeof(struct puc_strategy), GFP_KERNEL);
	if (puc == NULL) {
		chg_err("alloc strategy memory error\n");
		return ERR_PTR(-ENOMEM);
	}

	puc_read_temp_type_by_node(node, puc);
	rc = puc_read_soc_range_by_node(node, puc);
	if (rc < 0) {
		chg_err("get oplus,soc_range property error, rc=%d\n", rc);
		goto base_info_err;
	}
	rc = puc_read_temp_range_by_node(node, puc, &temp_range_cnt);
	if (rc < 0) {
		chg_err("get oplus,temp_range property error, rc=%d\n", rc);
		goto base_info_err;
	}
	rc = puc_read_iterm_by_node(node, temp_range_cnt, iterm_buf, &iterm_len);
	if (rc < 0)
		goto base_info_err;
	for (i = 0; i < PUC_BATT_CURVE_SOC_RANGE_MAX; i++) {
		soc_node = of_get_child_by_name(node, puc_strategy_soc[i]);
		if (!soc_node) {
			chg_err("can't find %s node\n", puc_strategy_soc[i]);
			rc = -ENODEV;
			goto data_err;
		}

		if (!puc->temp_range_map_inited) {
			rc = puc_build_temp_range_map(puc, soc_node, &temp_range_cnt);
			if (rc < 0) {
				chg_err("temp_range property not found\n");
				goto data_err;
			}
			rc = puc_fill_iterm_data(puc, iterm_buf, iterm_len, temp_range_cnt);
			if (rc < 0)
				goto data_err;
		}

		rc = puc_load_curves_by_node(puc, soc_node, i, temp_range_cnt);
		if (rc < 0)
			goto data_err;
	}

	return (struct oplus_chg_strategy *)puc;

data_err:
	for (i = 0; i < PUC_BATT_CURVE_SOC_RANGE_MAX; i++) {
		for (j = 0; j < PUC_BATT_CURVE_TEMP_RANGE_MAX; j++) {
			if (puc->soc_curves[i].temp_curves[j].data != NULL) {
				kfree(puc->soc_curves[i].temp_curves[j].data);
				puc->soc_curves[i].temp_curves[j].data = NULL;
			}
		}
	}
base_info_err:
	kfree(puc);
	return ERR_PTR(rc);
}

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#define TMP_BUF_SIZE 10
static int puc_cfg_build_name(char *str_buf, size_t buf_size, const char *node_name,
			      const char *prop1, const char *prop2)
{
	int index;
	size_t node_len;

	if (!str_buf || !node_name || !prop1 || buf_size == 0)
		return -EINVAL;
	node_len = strlen(node_name);
	if (node_len >= buf_size - 1)
		return -EINVAL;

	if (prop2)
		index = snprintf(str_buf, buf_size - 1, "%s:%s:%s", node_name, prop1, prop2);
	else
		index = snprintf(str_buf, buf_size - 1, "%s:%s", node_name, prop1);
	if (index < 0 || index >= (int)(buf_size - 1))
		return -EINVAL;
	str_buf[index] = 0;
	return 0;
}

static int puc_cfg_find_data_head(struct oplus_param_head *head, char *str_buf,
				  const char *node_name, const char *prop1, const char *prop2,
				  struct oplus_cfg_data_head **data_head)
{
	int rc;

	if (!data_head)
		return -EINVAL;
	rc = puc_cfg_build_name(str_buf, PAGE_SIZE, node_name, prop1, prop2);
	if (rc < 0)
		return rc;
	*data_head = oplus_cfg_find_param_by_name(head, str_buf);
	if (*data_head == NULL)
		return -ENODATA;
	return 0;
}

static int puc_cfg_read_u32_array(struct oplus_param_head *head, char *str_buf,
				  const char *node_name, const char *prop,
				  u32 *buf, size_t buf_len, int *count)
{
	struct oplus_cfg_data_head *data_head;
	ssize_t data_len;
	int rc;

	if (!buf || !count)
		return -EINVAL;
	rc = puc_cfg_find_data_head(head, str_buf, node_name, prop, NULL, &data_head);
	if (rc < 0)
		return rc;
	data_len = oplus_cfg_get_data_size(data_head);
	if (data_len % sizeof(buf[0]) != 0)
		return -EINVAL;
	*count = data_len / sizeof(buf[0]);
	if (*count > (int)buf_len)
		return -EINVAL;
	rc = oplus_cfg_get_data(data_head, (u8 *)buf, data_len);
	if (rc < 0)
		return rc;
	return 0;
}

static int puc_cfg_build_temp_range_map(struct oplus_param_head *head, char *str_buf,
					const char *node_name, const char *soc_name,
					struct puc_strategy *puc, int *temp_range_cnt)
{
	int j;
	int rc;
	int temp_prop_cnt = 0;
	struct oplus_cfg_data_head *data_head;

	for (j = 0; j < PUC_BATT_CURVE_TEMP_RANGE_MAX; j++) {
		rc = puc_cfg_find_data_head(head, str_buf, node_name, soc_name,
					    puc_strategy_temp[j], &data_head);
		if (rc < 0) {
			if (puc_temp_range_is_legacy_required(j)) {
				chg_err("can't find legacy %s:%s:%s data head, rc=%d\n",
					node_name, soc_name, puc_strategy_temp[j], rc);
				return rc;
			}
			continue;
		}
		if (temp_prop_cnt >= PUC_BATT_CURVE_TEMP_RANGE_MAX) {
			chg_err("temp_range_map overflow, cnt=%d\n", temp_prop_cnt);
			return -EINVAL;
		}
		puc->temp_range_map[temp_prop_cnt++] = j;
	}
	if (temp_prop_cnt <= 0)
		return -ENODATA;
	if (temp_prop_cnt != *temp_range_cnt) {
		chg_err("temp_range property num(%d) mismatch range num(%d)\n",
			temp_prop_cnt, *temp_range_cnt);
		return -EINVAL;
	}
	puc->temp_range_map_inited = true;
	return 0;
}

static int puc_cfg_read_temp_curve(struct oplus_param_head *head, char *str_buf,
				   const char *node_name, const char *soc_name,
				   const char *temp_name, struct puc_strategy_temp_curves *curve)
{
	int rc;
	int k;
	ssize_t data_len;
	struct oplus_cfg_data_head *data_head;

	rc = puc_cfg_find_data_head(head, str_buf, node_name, soc_name, temp_name, &data_head);
	if (rc < 0) {
		chg_err("get %s:%s:%s data head error\n", node_name, soc_name, temp_name);
		return rc;
	}
	data_len = oplus_cfg_get_data_size(data_head);
	if (data_len % PUC_DATA_SIZE != 0) {
		chg_err("%s:%s:%s: buf size does not meet the requirements, size=%ld\n",
			node_name, soc_name, temp_name, data_len);
		return -EINVAL;
	}
	curve->num = data_len / PUC_DATA_SIZE;
	curve->data = kzalloc(data_len, GFP_KERNEL);
	if (curve->data == NULL) {
		chg_err("alloc strategy data memory error\n");
		return -ENOMEM;
	}
	rc = oplus_cfg_get_data(data_head, (u8 *)curve->data, data_len);
	if (rc < 0) {
		chg_err("get %s:%s:%s data error, rc=%d\n", node_name, soc_name, temp_name, rc);
		kfree(curve->data);
		curve->data = NULL;
		return rc;
	}
	for (k = 0; k < curve->num; k++) {
		curve->data[k].target_vbus = le32_to_cpu(curve->data[k].target_vbus);
		curve->data[k].target_vbat = le32_to_cpu(curve->data[k].target_vbat);
		curve->data[k].target_ibus = le32_to_cpu(curve->data[k].target_ibus);
		curve->data[k].flags = le32_to_cpu(curve->data[k].flags);
		curve->data[k].target_time = le32_to_cpu(curve->data[k].target_time);
	}
	return 0;
}

static struct oplus_chg_strategy *puc_strategy_alloc_by_param_head(const char *node_name, struct oplus_param_head *head)
{
	struct puc_strategy *puc;
	int rc;
	int i;
	int j;
	int32_t buf[TMP_BUF_SIZE];
	char *str_buf;
	int temp_range_bound_cnt;
	int temp_range_cnt;
	int temp_idx;
	int count;

	if (node_name == NULL) {
		chg_err("node_name is NULL\n");
		return ERR_PTR(-EINVAL);
	}
	if (head == NULL) {
		chg_err("head is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	puc = kzalloc(sizeof(struct puc_strategy), GFP_KERNEL);
	if (puc == NULL) {
		chg_err("alloc strategy memory error\n");
		return ERR_PTR(-ENOMEM);
	}
	str_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (str_buf == NULL) {
		chg_err("alloc str_buf memory error\n");
		rc = -ENOMEM;
		goto str_buf_err;
	}

	rc = puc_cfg_read_u32_array(head, str_buf, node_name, "oplus,temp_type",
				    (u32 *)buf, TMP_BUF_SIZE, &count);
	if (rc < 0 || count < 1) {
		chg_err("get oplus,temp_type data head error\n");
		goto base_info_err;
	}
	puc->temp_type = (uint32_t)(le32_to_cpu(buf[0]));
	chg_info("[TEST]:oplus,temp_type = %u\n", puc->temp_type);

	rc = puc_cfg_read_u32_array(head, str_buf, node_name, "oplus,soc_range",
				    (u32 *)buf, TMP_BUF_SIZE, &count);
	if (rc < 0 || count != PUC_BATT_CURVE_SOC_RANGE_MAX + 1) {
		chg_err("get oplus,soc_range data head error\n");
		rc = -EINVAL;
		goto base_info_err;
	}
	for (i = 0; i < PUC_BATT_CURVE_SOC_RANGE_MAX + 1; i++)
		puc->soc_range_data[i] = (uint32_t)(le32_to_cpu(buf[i]));

	rc = puc_cfg_read_u32_array(head, str_buf, node_name, "oplus,temp_range",
				    (u32 *)buf, TMP_BUF_SIZE, &count);
	if (rc < 0) {
		chg_err("get oplus,temp_range data head error\n");
		goto base_info_err;
	}
	temp_range_bound_cnt = count;
	if (temp_range_bound_cnt <= 1 || temp_range_bound_cnt > PUC_BATT_CURVE_TEMP_RANGE_MAX + 1) {
		chg_err("oplus,temp_range entries(%d) num error, need >= 2 and <= %d\n",
			temp_range_bound_cnt, PUC_BATT_CURVE_TEMP_RANGE_MAX + 1);
		rc = -EINVAL;
		goto base_info_err;
	}
	for (i = 0; i < temp_range_bound_cnt; i++)
		puc->temp_range_data[i] = (uint32_t)le32_to_cpu(buf[i]);
	puc->temp_range_bound_cnt = temp_range_bound_cnt;
	puc->temp_range_cnt = temp_range_bound_cnt - 1;
	temp_range_cnt = puc->temp_range_cnt;

	for (i = 0; i < PUC_BATT_CURVE_SOC_RANGE_MAX; i++) {
		if (!puc->temp_range_map_inited) {
			rc = puc_cfg_build_temp_range_map(head, str_buf, node_name,
							  puc_strategy_soc[i], puc, &temp_range_cnt);
			if (rc < 0) {
				chg_err("temp_range property not found\n");
				goto data_err;
			}
		}

		for (j = 0; j < temp_range_cnt; j++) {
			temp_idx = puc->temp_range_map[j];
			if (temp_idx < 0 ||
			    temp_idx >= PUC_BATT_CURVE_TEMP_RANGE_MAX) {
				chg_err("invalid temp_idx=%d\n", temp_idx);
				rc = -EINVAL;
				goto data_err;
			}
			rc = puc_cfg_read_temp_curve(head, str_buf, node_name, puc_strategy_soc[i],
						     puc_strategy_temp[temp_idx],
						     &puc->soc_curves[i].temp_curves[temp_idx]);
			if (rc < 0)
				goto data_err;
		}
	}

	return (struct oplus_chg_strategy *)puc;

data_err:
	for (i = 0; i < PUC_BATT_CURVE_SOC_RANGE_MAX; i++) {
		for (j = 0; j < PUC_BATT_CURVE_TEMP_RANGE_MAX; j++) {
			if (puc->soc_curves[i].temp_curves[j].data != NULL) {
				kfree(puc->soc_curves[i].temp_curves[j].data);
				puc->soc_curves[i].temp_curves[j].data = NULL;
			}
		}
	}
base_info_err:
	kfree(str_buf);
str_buf_err:
	kfree(puc);
	return ERR_PTR(rc);
}
#endif /* CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER */

static int puc_strategy_release(struct oplus_chg_strategy *strategy)
{
	struct puc_strategy *puc;
	int i, j;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	puc = (struct puc_strategy *)strategy;

	for (i = 0; i < PUC_BATT_CURVE_SOC_RANGE_MAX; i++) {
		for (j = 0; j < PUC_BATT_CURVE_TEMP_RANGE_MAX; j++) {
			if (puc->soc_curves[i].temp_curves[j].data != NULL) {
				kfree(puc->soc_curves[i].temp_curves[j].data);
				puc->soc_curves[i].temp_curves[j].data = NULL;
			}
		}
	}
	kfree(puc);

	return 0;
}

static int puc_strategy_init(struct oplus_chg_strategy *strategy)
{
	struct puc_strategy *puc;
	enum puc_temp_range temp_range;
	enum puc_soc_range soc_range;
	int vbat;
	int i;
	int rc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	puc = (struct puc_strategy *)strategy;

	soc_range = puc_get_soc_region(puc);
	if (soc_range == PUC_BATT_CURVE_SOC_RANGE_INVALID)
		soc_range = puc_get_fastchg_allow_soc_region(puc);
	if (soc_range == PUC_BATT_CURVE_SOC_RANGE_INVALID)
		return -EFAULT;

	temp_range = puc_get_temp_region(puc);
	if (temp_range == PUC_BATT_CURVE_TEMP_RANGE_INVALID)
		return -EFAULT;

	rc = puc_strategy_get_vbat(puc, &vbat);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		return rc;
	}

	chg_info("uss %s:%s curve\n", puc_strategy_soc[soc_range], puc_strategy_temp[temp_range]);
	puc->curve = &puc->soc_curves[soc_range].temp_curves[temp_range];
	for (i = 0; i < puc->curve->num; i++) {
		if (vbat < puc->curve->data[i].target_vbat) {
			puc->curr_level = i;
			break;
		}
	}
	if (i >= puc->curve->num) {
		puc->curr_level = puc->curve->num - 1;
		chg_err("The battery voltage is too high, there is no suitable range, vbat=%d\n", vbat);
	}
	if (puc->curve->data[puc->curr_level].target_time > 0)
		puc->timeout = jiffies + msecs_to_jiffies(puc->curve->data[puc->curr_level].target_time * 1000);
	else
		puc->timeout = 0;
	puc->over_time = 0;
	puc->iterm = puc->iterm_data[temp_range];

	return 0;
}
static int puc_strategy_set_process_data(struct oplus_chg_strategy *strategy, const char *type, unsigned long arg)
{
	struct puc_strategy *puc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if(strcmp(type, "temp_region") == 0) {
		puc = (struct puc_strategy *)strategy;
		chg_info("type = %s", type);
		chg_info("arg = %lu", arg);
		if((arg < PUC_BATT_CURVE_TEMP_RANGE_COOL) || (arg > PUC_BATT_CURVE_TEMP_RANGE_MAX)) {
			chg_info("puc->temp_region out of range");
			puc->temp_region = PUC_BATT_CURVE_TEMP_RANGE_INVALID;
			return -EINVAL;
		}

		puc->temp_region = arg - 1;

		chg_info("puc->temp_region = %d", puc->temp_region);
		return 0;
	}

	if(strcmp(type, "allow_soc") == 0) {
		puc = (struct puc_strategy *)strategy;
		chg_info("type = %s", type);
		chg_info("arg = %lu", arg);

		if((arg < puc->soc_range_data[PUC_BATT_CURVE_SOC_RANGE_MIN]) ||
		   (arg > puc->soc_range_data[PUC_BATT_CURVE_SOC_RANGE_MAX])) {
			chg_info("puc->allow_soc out of range");
			puc->allow_soc = PUC_BATT_CURVE_SOC_RANGE_INVALID;
			return -EINVAL;
		}
		puc->allow_soc = (int)arg;
		chg_info("puc->allow_soc = %d", puc->allow_soc);
		return 0;
	}
	return -ENOTSUPP;
}

static int puc_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret)
{
	struct puc_strategy *puc;
	struct puc_strategy_ret_data *ret_data;
	struct puc_strategy_data *data;
	int vbat;
	int rc;
	bool curve_level_update = false;

#define VBAT_OVER_TIME_MS	2500
#define CV_VBAT_OVER_TIME_MS	10000

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (ret == NULL) {
		chg_err("ret is NULL\n");
		return -EINVAL;
	}
	puc = (struct puc_strategy *)strategy;
	if (puc->curve == NULL) {
		chg_err("curve is NULL\n");
		return -EINVAL;
	}
	if (puc->curr_level >= puc->curve->num)
		goto out;

	rc = puc_strategy_get_vbat(puc, &vbat);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		return rc;
	}

	data = &puc->curve->data[puc->curr_level];
	if (puc->timeout > 0 && time_is_before_jiffies(puc->timeout)) {
		puc->curr_level++;
		curve_level_update = true;
		chg_info("timeout, switch to next level(=%d)\n", puc->curr_level);
		goto out;
	}
	if (vbat > data->target_vbat) {
		if (data->support_cv && data->exit)
			goto out;
		if (puc->over_time == 0) {
			if (data->support_cv)
				puc->over_time = msecs_to_jiffies(CV_VBAT_OVER_TIME_MS) + jiffies;
			else
				puc->over_time = msecs_to_jiffies(VBAT_OVER_TIME_MS) + jiffies;
		} else if (time_is_before_jiffies(puc->over_time)) {
			puc->over_time = 0;
			puc->curr_level++;
			curve_level_update = true;
			chg_info("switch to next level(=%d)\n", puc->curr_level);
			goto out;
		}
	} else {
		puc->over_time = 0;
	}

out:
	ret_data = (struct puc_strategy_ret_data *)ret;
	if (puc->curr_level >= puc->curve->num) {
		ret_data->target_vbus = 0;
		ret_data->target_vbat = 0;
		ret_data->target_ibus = 0;
		ret_data->index = 0;
		ret_data->last_gear = false;
		ret_data->exit = true;
		ret_data->support_cv = false;
		chg_info("curve exit\n");
		return 0;
	}

	if (curve_level_update) {
		data = &puc->curve->data[puc->curr_level];
		if (data->target_time > 0)
			puc->timeout = jiffies + msecs_to_jiffies(data->target_time * 1000);
		else
			puc->timeout = 0;
		puc->over_time = 0;
		chg_info("level[%d]: %d %d %d %d %d %d 0x%x\n", puc->curr_level,
			 data->target_vbus, data->target_vbat,
			 data->target_ibus, data->exit, data->target_time,
			 data->support_cv, data->reserve_flags);
	}

	ret_data->target_vbus = data->target_vbus;
	ret_data->support_cv = data->support_cv;
	ret_data->target_vbat = data->target_vbat;
	ret_data->target_ibus = data->target_ibus;
	ret_data->index = puc->curr_level;
	ret_data->last_gear = !!data->exit;
	ret_data->exit = false;
	ret_data->iterm = puc->iterm;

	return 0;
}

static int puc_strategy_get_metadata(struct oplus_chg_strategy *strategy, void *ret)
{
	struct puc_strategy *puc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (ret == NULL) {
		chg_err("ret is NULL\n");
		return -EINVAL;
	}

	puc = (struct puc_strategy *)strategy;

	memcpy(ret, puc->curve, sizeof(*puc->curve));
	return 0;
}

static struct oplus_chg_strategy_desc puc_strategy_desc = {
	.name = "pps_ufcs_curve_v2",
	.strategy_init = puc_strategy_init,
	.strategy_release = puc_strategy_release,
	.strategy_alloc = puc_strategy_alloc,
	.strategy_alloc_by_node = puc_strategy_alloc_by_node,
#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	.strategy_alloc_by_param_head = puc_strategy_alloc_by_param_head,
#endif
	.strategy_get_data = puc_strategy_get_data,
	.strategy_set_process_data = puc_strategy_set_process_data,
	.strategy_get_metadata = puc_strategy_get_metadata,
};

int puc2_strategy_register(void)
{
	return oplus_chg_strategy_register(&puc_strategy_desc);
}

