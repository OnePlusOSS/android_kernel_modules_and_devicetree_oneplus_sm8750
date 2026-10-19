// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2024 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[STRATEGY_LOW_CURR_FULL]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_strategy.h>

#define LOW_CURR_FULL_SYS_MAX		6
#define LCF_TEMP_STATUS_OVER_COUNTS	2
#define LCF_LOW_CURR_FULL_COUNTS	6
#define NAME_LENGTH			16
enum {
	STRATEGY_TEMP_RANGE_T0,
	STRATEGY_TEMP_RANGE_T1,
	STRATEGY_TEMP_RANGE_T2,
	STRATEGY_TEMP_RANGE_T3,
	STRATEGY_TEMP_RANGE_T4,
	STRATEGY_TEMP_RANGE_T5,
	STRATEGY_TEMP_RANGE_T6,
	STRATEGY_TEMP_RANGE_T7,
	STRATEGY_TEMP_RANGE_T8,
	STRATEGY_TEMP_RANGE_T9,
	STRATEGY_TEMP_RANGE_MAX,
	STRATEGY_TEMP_RANGE_INVALID = STRATEGY_TEMP_RANGE_MAX
};

static const char *const lcf_strategy_temp_range_name[] = {
	[STRATEGY_TEMP_RANGE_T0] = "strategy_temp_range_t0",
	[STRATEGY_TEMP_RANGE_T1] = "strategy_temp_range_t1",
	[STRATEGY_TEMP_RANGE_T2] = "strategy_temp_range_t2",
	[STRATEGY_TEMP_RANGE_T3] = "strategy_temp_range_t3",
	[STRATEGY_TEMP_RANGE_T4] = "strategy_temp_range_t4",
	[STRATEGY_TEMP_RANGE_T5] = "strategy_temp_range_t5",
	[STRATEGY_TEMP_RANGE_T6] = "strategy_temp_range_t6",
	[STRATEGY_TEMP_RANGE_T7] = "strategy_temp_range_t7",
	[STRATEGY_TEMP_RANGE_T8] = "strategy_temp_range_t8",
	[STRATEGY_TEMP_RANGE_T9] = "strategy_temp_range_t9"
};

struct low_curr_full_curve {
	unsigned int iterm;
	unsigned int vterm;
	bool exit;
};

struct low_curr_full_curves_temp_range {
	struct low_curr_full_curve curves[LOW_CURR_FULL_SYS_MAX];
	int num;
};

struct low_curr_full_temp_range {
	int range[STRATEGY_TEMP_RANGE_MAX+1];
	int range_num;  /* Actual number of temperature thresholds configured */
	int index;
	int status;
};

struct lcf_strategy {
	struct oplus_chg_strategy strategy;
	unsigned int temp_type;
	struct low_curr_full_temp_range temp_range;
	struct low_curr_full_curves_temp_range curves_temp_range[STRATEGY_TEMP_RANGE_MAX];
	int low_curr_full_conut;
	char topic_name[NAME_LENGTH];
	struct oplus_mms *topic;
	int alarm_offset;
};

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

__maybe_unused static bool is_gauge_topic_available_by_lcy(struct lcf_strategy *lcy)
{
	if (!lcy->topic)
		lcy->topic = oplus_mms_get_by_name(lcy->topic_name);
	return !!lcy->topic;
}

static int __read_signed_data_from_node(struct device_node *node,
					const char *prop_str,
					s32 *addr, int len_max)
{
	int rc = 0, length;

	if (!node || !prop_str || !addr) {
		chg_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(s32));
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
	if (rc) {
		chg_err("Read %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	return rc;
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

static int lcf_strategy_get_temp(struct lcf_strategy *lcf, int *temp)
{
	union mms_msg_data data = { 0 };
	int rc;

	switch (lcf->temp_type) {
	case STRATEGY_USE_BATT_TEMP:
		if (!is_gauge_topic_available()) {
			chg_err("gauge topic not found\n");
			return -ENODEV;
		}
		rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_TEMP,
					     &data, false);
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
		chg_err("not support temp type, type=%d\n", lcf->temp_type);
		return -EINVAL;
	}

	return 0;
}

static int lcf_strategy_get_vbat(struct lcf_strategy *lcf, int *vbat)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_gauge_topic_available_by_lcy(lcf)) {
		chg_err("gauge topic not found\n");
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(lcf->topic, GAUGE_ITEM_VOL_MAX,
				     &data, false);
	if (rc < 0) {
		chg_err("can't get vbat, rc=%d\n", rc);
		return rc;
	}
	*vbat = data.intval;

	return 0;
}

static int lcf_strategy_get_ibat(struct lcf_strategy *lcf, int *ibat)
{
	union mms_msg_data data = { 0 };
	int rc;

	if (!is_gauge_topic_available_by_lcy(lcf)) {
		chg_err("gauge topic not found\n");
		return -ENODEV;
	}
	rc = oplus_mms_get_item_data(lcf->topic, GAUGE_ITEM_CURR,
				     &data, false);
	if (rc < 0) {
		chg_err("can't get ibat, rc=%d\n", rc);
		return rc;
	}
	*ibat = -data.intval;

	return 0;
}

static int lcf_get_temp_region(struct lcf_strategy *lcf)
{
	int i, temp;
	int temp_region = STRATEGY_TEMP_RANGE_INVALID;
	int rc;
	int temp_range_num = lcf->temp_range.range_num;

	rc = lcf_strategy_get_temp(lcf, &temp);
	if (rc < 0) {
		chg_err("can't get temp, rc=%d\n", rc);
		return STRATEGY_TEMP_RANGE_INVALID;
	}

	for (i = 0; i < temp_range_num - 1; i++) {
		if ((temp >= lcf->temp_range.range[i]) &&
		    (temp < lcf->temp_range.range[i+1])) {
			temp_region = i;
			break;
		}
	}
	if (i == temp_range_num - 1) {
		chg_err("over max temp range, i=%d, temp_range_num=%d\n",
			i, temp_range_num);
		return STRATEGY_TEMP_RANGE_INVALID;
	}
	return temp_region;
}

static struct oplus_chg_strategy *
lcf_strategy_alloc(unsigned char *buf, size_t size)
{
	return ERR_PTR(-ENOTSUPP);
}

static struct oplus_chg_strategy *
lcf_strategy_alloc_by_node(struct device_node *node)
{
	struct lcf_strategy *lcf;
	u32 data;
	int rc;
	int i, j;
	int length;
	struct device_node *curves_node;
	const char *topic_name;

	if (node == NULL) {
		chg_err("node is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	lcf = kzalloc(sizeof(struct lcf_strategy), GFP_KERNEL);
	if (lcf == NULL) {
		chg_err("alloc strategy memory error\n");
		return ERR_PTR(-ENOMEM);
	}

	memset(lcf->topic_name, 0, sizeof(lcf->topic_name));
	rc = of_property_read_string(node, "oplus,gauge_topic_name", &topic_name);
	if (rc < 0) {
		chg_err("oplus,gauge_topic_name reading failed, rc=%d\n", rc);
		snprintf(lcf->topic_name, sizeof(lcf->topic_name), "gauge");
	} else {
		snprintf(lcf->topic_name, sizeof(lcf->topic_name), "%s", topic_name);
		chg_err("lcf gauge topic name: %s\n", lcf->topic_name);
	}

	rc = of_property_read_u32(node, "oplus,temp_type", &data);
	if (rc < 0) {
		chg_err("oplus,temp_type reading failed, rc=%d\n", rc);
		lcf->temp_type = STRATEGY_USE_SHELL_TEMP;
	} else {
		lcf->temp_type = (uint32_t)data;
		chg_info("lcf->temp_type = %d\n", lcf->temp_type);
	}

	rc = of_property_read_u32(node, "oplus,alarm_offset", &data);
	if (rc < 0) {
		chg_err("oplus,alarm_offset reading failed, rc=%d\n", rc);
		lcf->alarm_offset = 0;
	} else {
		lcf->alarm_offset = (uint32_t)data;
		chg_info("lcf->alarm_offset = %d\n", lcf->alarm_offset);
	}

	/* Dynamically read temp_range count */
	rc = of_property_count_elems_of_size(node, "oplus,temp_range", sizeof(s32));
	if (rc < 0) {
		chg_err("get oplus,temp_range count error, rc=%d\n", rc);
		goto base_info_err;
	}
	if (rc > STRATEGY_TEMP_RANGE_MAX) {
		chg_err("oplus,temp_range has %d elements, exceeds max %d\n",
			rc, STRATEGY_TEMP_RANGE_MAX);
		rc = -EINVAL;
		goto base_info_err;
	}
	lcf->temp_range.range_num = rc;

	rc = __read_signed_data_from_node(node, "oplus,temp_range",
					  (s32 *)lcf->temp_range.range,
					  lcf->temp_range.range_num);
	if (rc < 0) {
		chg_err("get oplus,temp_range property error, rc=%d\n", rc);
		goto base_info_err;
	}
	chg_debug("lcf->temp_range.range_num=%d, range: ", lcf->temp_range.range_num);
	for (i = 0; i < lcf->temp_range.range_num; i++)
		chg_debug("%d ", lcf->temp_range.range[i]);

	curves_node = of_get_child_by_name(node, "strategy_temp_range_curves");
	if (!curves_node) {
		chg_err("Can not find strategy_temp_range_curves node\n");
		rc = -ENODEV;
		goto base_info_err;
	}

	/* Only parse curves for configured temperature ranges */
	for (i = 0; i < lcf->temp_range.range_num - 1; i++) {
		rc = of_property_count_elems_of_size(curves_node, lcf_strategy_temp_range_name[i], sizeof(u32));
		if (rc < 0) {
			chg_err("Count lcf_strategy_temp_range_name %s failed, rc=%d\n",
				lcf_strategy_temp_range_name[i], rc);
			goto base_info_err;
		}
		length = rc;
		rc = __read_unsigned_data_from_node(curves_node, lcf_strategy_temp_range_name[i],
						(u32 *)lcf->curves_temp_range[i].curves, length);
		if (rc < 0) {
			chg_err("parse lcf->curves_temp_range[%d].curves failed, rc=%d\n", i, rc);
			goto base_info_err;
		}
		lcf->curves_temp_range[i].num = length / 3;
		for (j = 0; j < lcf->curves_temp_range[i].num; j++)
			chg_debug("curves_temp_range[%d].curves[%d] iterm=%d vterm=%d exit=%d", i, j,
				 lcf->curves_temp_range[i].curves[j].iterm,
				 lcf->curves_temp_range[i].curves[j].vterm,
				 lcf->curves_temp_range[i].curves[j].exit);
	}

	return (struct oplus_chg_strategy *)lcf;

base_info_err:
	kfree(lcf);
	return ERR_PTR(rc);
}

#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
#define LCF_CURVES_DATA_SIZE	sizeof(struct low_curr_full_curve)
#define TMP_BUF_SIZE 		(STRATEGY_TEMP_RANGE_MAX + 1)
static struct oplus_chg_strategy *lcf_strategy_alloc_by_param_head(const char *node_name, struct oplus_param_head *head)
{
	struct lcf_strategy *lcf;
	int rc;
	int i, j;
	struct oplus_cfg_data_head *data_head;
	int32_t buf[TMP_BUF_SIZE];
	ssize_t data_len;
	char *str_buf;
	int index = 0;

	if (node_name == NULL) {
		chg_err("node_name is NULL\n");
		return ERR_PTR(-EINVAL);
	}
	if (head == NULL) {
		chg_err("head is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	lcf = kzalloc(sizeof(struct lcf_strategy), GFP_KERNEL);
	if (lcf == NULL) {
		chg_err("alloc strategy memory error\n");
		return ERR_PTR(-ENOMEM);
	}
	/* Initialize range_num to 0, will be set when parsing temp_range */
	lcf->temp_range.range_num = 0;
	str_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (str_buf == NULL) {
		chg_err("alloc str_buf memory error\n");
		rc = -ENOMEM;
		goto str_buf_err;
	}

	index = snprintf(str_buf, PAGE_SIZE - 1, "%s:oplus,temp_type", node_name);
	if (index < 0 || index >= PAGE_SIZE) {
		rc = -EFAULT;
		goto base_info_err;
	}
	str_buf[index] = 0;
	data_head = oplus_cfg_find_param_by_name(head, str_buf);
	if (data_head == NULL) {
		rc = -ENODATA;
		chg_err("get oplus,temp_type data head error\n");
		goto base_info_err;
	}
	rc = oplus_cfg_get_data(data_head, (u8 *)buf, sizeof(buf[0]));
	if (rc < 0) {
		chg_err("get oplus,temp_type data error, rc=%d\n", rc);
		goto base_info_err;
	}
	lcf->temp_type = (uint32_t)(le32_to_cpu(buf[0]));
	chg_info("[TEST]:oplus,temp_type = %u\n", lcf->temp_type);

	index = snprintf(str_buf, PAGE_SIZE - 1, "%s:oplus,temp_range", node_name);
	if (index < 0 || index >= PAGE_SIZE) {
		rc = -EFAULT;
		goto base_info_err;
	}
	str_buf[index] = 0;
	data_head = oplus_cfg_find_param_by_name(head, str_buf);
	if (data_head == NULL) {
		rc = -ENODATA;
		chg_err("get oplus,temp_range data head error\n");
		goto base_info_err;
	}
	data_len = oplus_cfg_get_data_size(data_head);
	if (data_len / sizeof(buf[0]) > STRATEGY_TEMP_RANGE_MAX + 1) {
		rc = -EINVAL;
		chg_err("configuration data size error, data_len=%ld exceeds max %d\n",
			data_len / sizeof(buf[0]), STRATEGY_TEMP_RANGE_MAX + 1);
		goto base_info_err;
	}
	lcf->temp_range.range_num = data_len / sizeof(buf[0]);
	rc = oplus_cfg_get_data(data_head, (u8 *)buf, data_len);
	if (rc < 0) {
		chg_err("get oplus,temp_range data error, rc=%d\n", rc);
		goto base_info_err;
	}
	for (i = 0; i < lcf->temp_range.range_num; i++)
		lcf->temp_range.range[i] = (uint32_t)(le32_to_cpu(buf[i]));
	chg_info("oplus,temp_range_num = %d\n", lcf->temp_range.range_num);

	/* Only parse curves for configured temperature ranges */
	for (i = 0; i < lcf->temp_range.range_num - 1; i++) {
		index = snprintf(str_buf, PAGE_SIZE - 1, "%s:strategy_temp_range_curves:%s", node_name, lcf_strategy_temp_range_name[i]);
		if (index < 0 || index >= PAGE_SIZE) {
			rc = -EINVAL;
			goto base_info_err;
		}
		str_buf[index] = 0;
		data_head = oplus_cfg_find_param_by_name(head, str_buf);
		if (data_head == NULL) {
			rc = -ENODATA;
			chg_err("get %s:%s data head error\n", node_name, lcf_strategy_temp_range_name[i]);
			goto base_info_err;
		}
		data_len = oplus_cfg_get_data_size(data_head);
		if (data_len % LCF_CURVES_DATA_SIZE != 0) {
			chg_err("%s:strategy_temp_range_curves:%s: buf size does not meet the requirements, size=%ld\n",
				node_name, lcf_strategy_temp_range_name[i], data_len);
			rc = -EINVAL;
			goto base_info_err;
		}
		lcf->curves_temp_range[i].num = data_len / LCF_CURVES_DATA_SIZE;
		rc = oplus_cfg_get_data(data_head, (u8 *)lcf->curves_temp_range[i].curves, data_len);
		if (rc < 0) {
			chg_err("get %s:%s data error, rc=%d\n", node_name, lcf_strategy_temp_range_name[i], rc);
			goto base_info_err;
		}
		for (j = 0; j < lcf->curves_temp_range[i].num; j++) {
			lcf->curves_temp_range[i].curves[j].iterm =
				le32_to_cpu(lcf->curves_temp_range[i].curves[j].iterm);
			lcf->curves_temp_range[i].curves[j].vterm =
				le32_to_cpu(lcf->curves_temp_range[i].curves[j].vterm);
			lcf->curves_temp_range[i].curves[j].exit =
				le32_to_cpu(lcf->curves_temp_range[i].curves[j].exit);
		}
	}

	return (struct oplus_chg_strategy *)lcf;

base_info_err:
	kfree(str_buf);
str_buf_err:
	kfree(lcf);
	return ERR_PTR(rc);
}
#endif /* CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER */

static int lcf_strategy_init(struct oplus_chg_strategy *strategy)
{
	struct lcf_strategy *lcf;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	lcf = (struct lcf_strategy *)strategy;

	lcf->temp_range.status = lcf_get_temp_region(lcf);
	lcf->temp_range.index = lcf->temp_range.status;
	lcf->low_curr_full_conut = 0;
	return 0;
}

static int lcf_strategy_release(struct oplus_chg_strategy *strategy)
{
	struct lcf_strategy *lcf;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	lcf = (struct lcf_strategy *)strategy;
	kfree(lcf);

	return 0;
}

static void update_low_curr_temp_status(struct lcf_strategy *lcf)
{
	static int t_cnts = 0;
	static int pre_index = STRATEGY_TEMP_RANGE_INVALID;

	lcf->temp_range.index = lcf_get_temp_region(lcf);

	if (lcf->temp_range.index != pre_index) {
		t_cnts++;
		if (t_cnts >= LCF_TEMP_STATUS_OVER_COUNTS) {
			pre_index = lcf->temp_range.index;
			lcf->temp_range.status =
				lcf->temp_range.index;
			t_cnts = 0;
			chg_info("update low curr full temp range, index=%d\n",
				lcf->temp_range.index);
		}
	}
}

static int __lcf_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret, int offset)
{
	struct lcf_strategy *lcf;
	int i, vbatt, ibatt, temp_status, iterm, vterm;
	bool low_curr = false;
	int rc, *ret_val;

	if ((strategy == NULL) || (ret == NULL)) {
		chg_err("strategy or ret is NULL\n");
		return -EINVAL;
	}
	lcf = (struct lcf_strategy *)strategy;
	ret_val = (int *)ret;
	*ret_val = 0;

	update_low_curr_temp_status(lcf);
	temp_status = lcf->temp_range.status;
	if (temp_status >= lcf->temp_range.range_num - 1) {
		chg_debug("temp_status is %d, INVALID (max=%d, range_num=%d)\n",
			  temp_status, STRATEGY_TEMP_RANGE_MAX, lcf->temp_range.range_num);
		return -EINVAL;
	}
	rc = lcf_strategy_get_vbat(lcf, &vbatt);
	if (rc < 0) {
		chg_err("can't get vbatt, rc=%d\n", rc);
		return rc;
	}

	rc = lcf_strategy_get_ibat(lcf, &ibatt);
	if (rc < 0) {
		chg_err("can't get ibatt, rc=%d\n", rc);
		return rc;
	}

	for (i = 0; i < lcf->curves_temp_range[temp_status].num; i++) {
		iterm = lcf->curves_temp_range[temp_status].curves[i].iterm + offset;
		vterm = lcf->curves_temp_range[temp_status].curves[i].vterm;

		if ((ibatt <= iterm) && (vbatt >= vterm)) {
			low_curr = true;
			chg_info("low_curr = %d\n", low_curr);
			break;
		}
	}
	if (low_curr) {
		lcf->low_curr_full_conut++;
		if (lcf->low_curr_full_conut > LCF_LOW_CURR_FULL_COUNTS) {
			lcf->low_curr_full_conut = 0;
			*ret_val = 1;
			chg_info("*ret_val = %d\n", *ret_val);
		}
	} else {
		lcf->low_curr_full_conut = 0;
	}

	return 0;
}


static int lcf_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret)
{
	return __lcf_strategy_get_data(strategy, ret, 0);
}

static int lcf_strategy_get_custom_data(struct oplus_chg_strategy *strategy, const char *type, void *ret)
{
	struct lcf_strategy *lcf;

	if (strategy == NULL || !type) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	if (strcmp(type, "alarm") != 0)
		return -ENOTSUPP;
	lcf = (struct lcf_strategy *)strategy;

	if (!lcf->alarm_offset)
		return -EINVAL;

	return __lcf_strategy_get_data(strategy, ret, lcf->alarm_offset);
}

static struct oplus_chg_strategy_desc lcf_strategy_desc = {
	.name = "low_curr_full_strategy",
	.strategy_init = lcf_strategy_init,
	.strategy_release = lcf_strategy_release,
	.strategy_alloc = lcf_strategy_alloc,
	.strategy_alloc_by_node = lcf_strategy_alloc_by_node,
#if IS_ENABLED(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER)
	.strategy_alloc_by_param_head = lcf_strategy_alloc_by_param_head,
#endif
	.strategy_get_data = lcf_strategy_get_data,
	.strategy_get_custom_data = lcf_strategy_get_custom_data,
};

int lcf_strategy_register(void)
{
	return oplus_chg_strategy_register(&lcf_strategy_desc);
}
