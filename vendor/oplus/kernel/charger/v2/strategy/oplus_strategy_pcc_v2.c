// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2024 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[STRATEGY_PCC_V2]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_chg_comm.h>
#include <oplus_strategy.h>

#define GAUGE_IC_NUM_MAX 2

#define IBUS_INIT_MA	800
#define IBUS_MIN_MA	800

#define IMP_IBAT_UP_MA	1000
#define IBAT_STEP_MIN_MA	600
#define IBAT_STEP_MAX_MA	2000
#define VBAT_FULL_DONW_IBUS_MA	600
#define VBAT_FULL_MARGIN_THRESHOLD	50

#define VBAT_MV_VALID_MIN	2000
#define VBAT_MV_VALID_MAX	5500
#define T2_IBAT_MA_VALID_MIN	500
#define T2_TO_T1_DIFF_IBAT_VALID_MIN	200

#define CURRENT_DOWN_TIME_LIMIT_MS	100

struct battery_info {
	int vbat_mv;
	int ibat_ma;
};

struct pcc_strategy {
	struct oplus_chg_strategy strategy;

	struct oplus_mms *comm_topic;
	struct oplus_mms *gauge_topic;
	int battery_info_num;
	enum batt_connect_type batt_connect_type;
	int gauge_ratio;
	struct oplus_mms *gauge_topic_parallel[GAUGE_IC_NUM_MAX];

	struct battery_info curr_info[GAUGE_IC_NUM_MAX];
	struct battery_info t1_info[GAUGE_IC_NUM_MAX];
	struct battery_info t2_info[GAUGE_IC_NUM_MAX];
	bool t1_info_updated;
	bool t2_info_updated;
	int r_mohm[GAUGE_IC_NUM_MAX];
	int final_r_mohm;

	int cp_ratio;
	int ibus_full_limit;
	int one_time_full_volt;
	int ibus_req;
	int ibus_target;

	int vbat_mv;
	int shell_temp;

	unsigned long curr_down_moment;
	struct mutex lock;
};

static bool is_imp_in_range(int32_t temp_deci_c, int32_t impedance_mohm)
{
	struct imp_range {
		int32_t temp_deci_c_min;
		int32_t temp_deci_c_max;
		int32_t impedance_mohm_min;
		int32_t impedance_mohm_max;
	} const imp_ranges[] = {
		{ -50, 120, 50, 600 },
		{ 120, 200, 30, 400 },
		{ 200, 550, 20, 300 },
	};
	size_t i = 0;

	for (i = 0; i < ARRAY_SIZE(imp_ranges); i++) {
		if (temp_deci_c >= imp_ranges[i].temp_deci_c_min && temp_deci_c < imp_ranges[i].temp_deci_c_max)
			return impedance_mohm >= imp_ranges[i].impedance_mohm_min &&
			       impedance_mohm < imp_ranges[i].impedance_mohm_max;
	}

	return false;
}

int32_t get_ibus_by_volt_diff(int32_t volt_diff_mv, int cp_ratio)
{
	struct vbat_ibat_diff_map {
		int32_t vbat_diff_mv;
		int32_t ibat_diff_ma;
	} const vbat_ibat_diff_maps[] = {
		{ 400, 800 },
		{ 300, 600 },
		{ 200, 400 },
		{ 120, 300 },
		{ 0, 200 },
	};
	size_t i = 0;

	if (cp_ratio <= 0)
		return 0;

	for (i = 0; i < ARRAY_SIZE(vbat_ibat_diff_maps); i++) {
		if (volt_diff_mv >= vbat_ibat_diff_maps[i].vbat_diff_mv)
			return vbat_ibat_diff_maps[i].ibat_diff_ma / cp_ratio;
	}

	return 0;
}

static struct oplus_chg_strategy *pcc_strategy_alloc(unsigned char *buf, size_t size)
{
	return ERR_PTR(-ENOTSUPP);
}

static struct oplus_chg_strategy *pcc_strategy_alloc_by_node(struct device_node *node)
{
	struct pcc_strategy *pcc;

	if (node == NULL) {
		chg_err("node is NULL\n");
		return ERR_PTR(-EINVAL);
	}

	pcc = kzalloc(sizeof(struct pcc_strategy), GFP_KERNEL);
	if (pcc == NULL) {
		chg_err("alloc strategy memory error\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_init(&pcc->lock);
	return (struct oplus_chg_strategy *)pcc;
}

static int pcc_strategy_release(struct oplus_chg_strategy *strategy)
{
	struct pcc_strategy *pcc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	pcc = (struct pcc_strategy *)strategy;

	if (pcc) {
		mutex_destroy(&pcc->lock);
		kfree(pcc);
		pcc = NULL;
	}

	return 0;
}

static int pcc_strategy_init(struct oplus_chg_strategy *strategy)
{
	struct pcc_strategy *pcc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}
	pcc = (struct pcc_strategy *)strategy;

	pcc->comm_topic = oplus_mms_get_by_name("common");
	pcc->gauge_topic = oplus_mms_get_by_name("gauge");
	pcc->batt_connect_type = is_support_parallel_battery(pcc->gauge_topic);
	pcc->gauge_ratio = (pcc->batt_connect_type == PARALLEL_CONNECT_TYPE) ? 2 : 1;
	pcc->battery_info_num = (pcc->batt_connect_type == DEFAULT_CONNECT_TYPE) ? 1 : 2;
	pcc->gauge_topic_parallel[0] = oplus_mms_get_by_name("gauge:0");
	pcc->gauge_topic_parallel[1] = oplus_mms_get_by_name("gauge:1");

	mutex_lock(&pcc->lock);
	memset(pcc->curr_info, 0, sizeof(pcc->curr_info));
	memset(pcc->t1_info, 0, sizeof(pcc->t1_info));
	memset(pcc->t2_info, 0, sizeof(pcc->t2_info));
	memset(pcc->r_mohm, 0, sizeof(pcc->r_mohm));
	pcc->final_r_mohm = 0;
	pcc->t1_info_updated = false;
	pcc->t2_info_updated = false;

	pcc->ibus_req = IBUS_INIT_MA;
	pcc->ibus_full_limit = 0;
	pcc->ibus_target = 0;
	pcc->curr_down_moment = 0;
	pcc->cp_ratio = 1;
	mutex_unlock(&pcc->lock);

	return 0;
}

static int pcc_strategy_update_ibus_target(struct pcc_strategy *pcc, int ibus_target)
{
	mutex_lock(&pcc->lock);
	if (pcc->ibus_full_limit && ibus_target > pcc->ibus_full_limit)
		ibus_target = pcc->ibus_full_limit;
	pcc->ibus_target = ibus_target;
	if (pcc->ibus_req > ibus_target) {
		pcc->ibus_req = ibus_target;
		pcc->curr_down_moment = jiffies;
	}
	chg_info("ibus_target=%d, ibus_req=%d\n", ibus_target, pcc->ibus_req);
	mutex_unlock(&pcc->lock);

	return 0;
}

static int pcc_strategy_update_one_time_full_volt(struct pcc_strategy *pcc, int volt)
{
	mutex_lock(&pcc->lock);
	pcc->one_time_full_volt = volt;
	chg_info("one_time_full_volt:%d\n", pcc->one_time_full_volt);
	mutex_unlock(&pcc->lock);

	return 0;
}

static int pcc_strategy_update_ibus_full_limit(struct pcc_strategy *pcc, int i_req, bool step1)
{
	int ibus_full_limit;

	mutex_lock(&pcc->lock);
	if (step1)
		ibus_full_limit = i_req - VBAT_FULL_DONW_IBUS_MA;
	else
		ibus_full_limit = i_req;
	pcc->ibus_full_limit = max(ibus_full_limit / 100 * 100, IBUS_MIN_MA);
	chg_info("i_req:%d, cp_ratio:%d, ibus_full_limit:%d\n", i_req, pcc->cp_ratio, pcc->ibus_full_limit);
	mutex_unlock(&pcc->lock);

	return 0;
}

static int pcc_strategy_update_cp_ratio(struct pcc_strategy *pcc, int cp_ratio)
{
	mutex_lock(&pcc->lock);
	pcc->cp_ratio = cp_ratio;
	chg_info("cp_ratio:%d\n", cp_ratio);
	mutex_unlock(&pcc->lock);

	return 0;
}

static int pcc_strategy_cal_step_curr(struct pcc_strategy *pcc)
{
	int ibus_delta = pcc->ibus_target - pcc->ibus_req;
	int ibat_step_ma = 0;

	if (pcc->cp_ratio == 0) {
		chg_err("Invalid cp_ratio: 0\n");
		return -EINVAL;
	}

	if (pcc->ibus_req >= pcc->ibus_target) {
		pcc->ibus_req = pcc->ibus_target;
		return 0;
	}

	if (!pcc->t2_info_updated) {
		pcc->ibus_req += IMP_IBAT_UP_MA / pcc->cp_ratio;
		goto done;
	}

	if (pcc->final_r_mohm && ibus_delta >= IBAT_STEP_MIN_MA / pcc->cp_ratio) {
		ibat_step_ma = (pcc->one_time_full_volt - VBAT_FULL_MARGIN_THRESHOLD - pcc->vbat_mv) * 1000 /
			pcc->final_r_mohm / 100 * 100;
		if (ibat_step_ma >= IBAT_STEP_MIN_MA / pcc->cp_ratio) {
			pcc->ibus_req += min3(ibat_step_ma / pcc->cp_ratio, ibus_delta, IBAT_STEP_MAX_MA / pcc->cp_ratio);
			goto done;
		}
	}

	pcc->ibus_req += get_ibus_by_volt_diff(pcc->one_time_full_volt - pcc->vbat_mv, pcc->cp_ratio);
done:
	if (pcc->ibus_req > pcc->ibus_target)
		pcc->ibus_req = pcc->ibus_target;
	return 0;
}

static int update_batt_volt_curr(struct oplus_mms *gauge_topic, struct battery_info *info)
{
	int rc = 0;
	union mms_msg_data data = { 0 };

	rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	if (rc < 0) {
		chg_err("can't get batt volt, rc=%d\n", rc);
		return rc;
	}
	info->vbat_mv = data.intval;

	rc = oplus_mms_get_item_data(gauge_topic, GAUGE_ITEM_CURR, &data, true);
	if (rc < 0) {
		chg_err("can't get batt volt, rc=%d\n", rc);
		return rc;
	}
	info->ibat_ma = -data.intval;
	return 0;
}

static int pcc_strategy_update_batt_volt_curr(struct pcc_strategy *pcc)
{
	int rc;
	union mms_msg_data data = { 0 };

	rc = oplus_mms_get_item_data(pcc->comm_topic, COMM_ITEM_SHELL_TEMP, &data, true);
	if (rc < 0) {
		chg_err("can't get shell temp, rc=%d\n", rc);
		return rc;
	}
	pcc->shell_temp = data.intval;

	switch (pcc->batt_connect_type) {
	case PARALLEL_CONNECT_TYPE:
	case SERIAL_CONNECT_TYPE:
		update_batt_volt_curr(pcc->gauge_topic_parallel[0], &pcc->curr_info[0]);
		update_batt_volt_curr(pcc->gauge_topic_parallel[1], &pcc->curr_info[1]);
		break;
	case DEFAULT_CONNECT_TYPE:
	default:
		update_batt_volt_curr(pcc->gauge_topic, &pcc->curr_info[0]);
		break;
	}
	pcc->vbat_mv = max(pcc->curr_info[0].vbat_mv, pcc->curr_info[1].vbat_mv);

	if (!pcc->t1_info_updated) {
		memcpy(pcc->t1_info, pcc->curr_info, sizeof(pcc->t1_info));
		pcc->t1_info_updated = true;
	} else if (!pcc->t2_info_updated) {
		memcpy(pcc->t2_info, pcc->curr_info, sizeof(pcc->t2_info));
		pcc->t2_info_updated = true;
	} else {
		memcpy(pcc->t1_info, pcc->t2_info, sizeof(pcc->t1_info));
		memcpy(pcc->t2_info, pcc->curr_info, sizeof(pcc->t2_info));
	}
	return 0;
}

static int pcc_cal_batt_fastchg_imp_r(struct pcc_strategy *pcc)
{
	int i = 0;
	int r_mohm = 0;

	if (pcc->final_r_mohm)
		return 0;

	if (!pcc->t1_info_updated || !pcc->t2_info_updated) {
		chg_info("t1_info or t2_info not updated\n");
		return 0;
	}

	for (i = 0; i < pcc->battery_info_num; i++) {
		if (pcc->t1_info[i].vbat_mv <= VBAT_MV_VALID_MIN ||
		    pcc->t1_info[i].vbat_mv >= VBAT_MV_VALID_MAX ||
		    pcc->t2_info[i].vbat_mv <= VBAT_MV_VALID_MIN ||
		    pcc->t2_info[i].vbat_mv >= VBAT_MV_VALID_MAX ||
		    pcc->t2_info[i].ibat_ma <= T2_IBAT_MA_VALID_MIN ||
		    pcc->t2_info[i].ibat_ma - pcc->t1_info[i].ibat_ma <= T2_TO_T1_DIFF_IBAT_VALID_MIN) {
			chg_err("invalid parameters for battery%d t1[%dmV %dmA] t2[%dmV %dmA]\n", i,
				pcc->t1_info[i].vbat_mv, pcc->t1_info[i].ibat_ma,
				pcc->t2_info[i].vbat_mv, pcc->t2_info[i].ibat_ma);
			pcc->r_mohm[i] = 0;
			continue;
		}
		pcc->r_mohm[i] = (pcc->t2_info[i].vbat_mv - pcc->t1_info[i].vbat_mv) * 1000 /
				(pcc->t2_info[i].ibat_ma - pcc->t1_info[i].ibat_ma);

		chg_info("battery%d t1[%dmV %dmA] t2[%dmV %dmA] temp=%d r=%dmohm\n", i,
			pcc->t1_info[i].vbat_mv, pcc->t1_info[i].ibat_ma,
			pcc->t2_info[i].vbat_mv, pcc->t2_info[i].ibat_ma,
			pcc->shell_temp, pcc->r_mohm[i]);

		if (!is_imp_in_range(pcc->shell_temp, pcc->r_mohm[i])) {
			chg_err("r=%d invalid battery%d\n", pcc->r_mohm[i], i);
			pcc->r_mohm[i] = 0;
			continue;
		}
		if (pcc->r_mohm[i] > r_mohm)
			r_mohm = pcc->r_mohm[i];
	}

	pcc->final_r_mohm = r_mohm;
	return 0;
}

static int pcc_strategy_update_ibus_req(struct pcc_strategy *pcc)
{
	mutex_lock(&pcc->lock);

	pcc_strategy_update_batt_volt_curr(pcc);
	pcc_cal_batt_fastchg_imp_r(pcc);

	if (pcc->ibus_target == pcc->ibus_req) {
		mutex_unlock(&pcc->lock);
		return 0;
	}

	if (pcc->curr_down_moment &&
	    time_is_after_jiffies(pcc->curr_down_moment + msecs_to_jiffies(CURRENT_DOWN_TIME_LIMIT_MS))) {
		chg_info("recalculation of a cycle just after downflow\n");
		mutex_unlock(&pcc->lock);
		return 0;
	}

	pcc_strategy_cal_step_curr(pcc);
	chg_info("after adjust target curr:%d, request curr:%d\n", pcc->ibus_target, pcc->ibus_req);

	mutex_unlock(&pcc->lock);
	return 0;
}

static int pcc_strategy_set_process_data(struct oplus_chg_strategy *strategy, const char *type, unsigned long arg)
{
	struct pcc_strategy *pcc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}

	if (type == NULL) {
		chg_err("type is NULL\n");
		return -EINVAL;
	}

	pcc = (struct pcc_strategy *)strategy;
	chg_info("type = %s, arg = %lu\n", type, arg);

	if (sysfs_streq(type, "curr_target")) {
		pcc_strategy_update_ibus_target(pcc, arg * 100);
	} else if (sysfs_streq(type, "curr_pcc_cycle_t")) {
		pcc_strategy_update_ibus_req(pcc);
	} else if (sysfs_streq(type, "1time_full_voltage")) {
		pcc_strategy_update_one_time_full_volt(pcc, (int)arg);
	} else if (sysfs_streq(type, "1time_full_happen")) {
		pcc_strategy_update_ibus_full_limit(pcc, (int)arg * 100, true);
	} else if (sysfs_streq(type, "1time_full_step_happen")) {
		pcc_strategy_update_ibus_full_limit(pcc, (int)arg * 100, false);
	} else if (sysfs_streq(type, "cp_ratio")) {
		pcc_strategy_update_cp_ratio(pcc, (int)arg);
	} else {
		return -ENOTSUPP;
	}

	return 0;
}

static int pcc_strategy_get_data(struct oplus_chg_strategy *strategy, void *ret)
{
	struct pcc_strategy *pcc;

	if (strategy == NULL) {
		chg_err("strategy is NULL\n");
		return -EINVAL;
	}

	if (ret == NULL) {
		chg_err("ret is NULL\n");
		return -EINVAL;
	}

	pcc = (struct pcc_strategy *)strategy;

	mutex_lock(&pcc->lock);
	*((char *)ret) = pcc->ibus_req / 100;
	mutex_unlock(&pcc->lock);

	return 0;
}

static struct oplus_chg_strategy_desc pcc_strategy_desc = {
	.name = "pcc_strategy_v2",
	.strategy_init = pcc_strategy_init,
	.strategy_release = pcc_strategy_release,
	.strategy_alloc = pcc_strategy_alloc,
	.strategy_alloc_by_node = pcc_strategy_alloc_by_node,
	.strategy_get_data = pcc_strategy_get_data,
	.strategy_set_process_data = pcc_strategy_set_process_data,
	.strategy_get_metadata = NULL,
};

int pcc_v2_strategy_register(void)
{
	return oplus_chg_strategy_register(&pcc_strategy_desc);
}

