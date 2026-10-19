// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 * UFCS power regulation (PR)
 */

#ifndef __OPLUS_CHG_UFCS_PR_H__
#define __OPLUS_CHG_UFCS_PR_H__

#define UFCS_PR_PDO_STEP_MV		20

#define UFCS_PR_DEFAULT_R_INPUT_MOHM	100
#define UFCS_PR_DEFAULT_R_BTB_MOHM	10
#define UFCS_PR_CHANGE_THRES_MA		200
#define UFCS_PR_DELAY_MS		200
#define UFCS_PR_FAST_DELAY_MS		20
#define UFCS_PR_FULL_DELAY_MS		1000
#define UFCS_PR_PDO_ERR_DELAY_MS	1000

#define UFCS_PR_VSTEP1			40
#define UFCS_PR_VSTEP2			20
#define UFCS_PR_RISING_STEP_MAX_MV	300
#define UFCS_PR_RISING_VFULL_TOLERANCE	20
#define UFCS_PR_ADJUST_TIMEOUT_MS	1150
#define UFCS_PR_CC_PLC_IBUS_TOLERANCE	50
#define UFCS_PR_CC_IBUS_TOLERANCE	100
#define UFCS_PR_RISING_ADJUST_CHECK_CNT	5
#define UFCS_PR_RISING_ADJUST_CHECK_PCT	30

#define UFCS_PR_FALLING_IBUS_DELAT_MAX	1000
#define UFCS_PR_FALLING_VBUS_DELAT_MAX	300
#define UFCS_PR_FALLING_VSTEP_MAX	200
#define UFCS_PR_FALLING_IBUS_TOLERANCE	200

#define UFCS_PR_CC2CV_VBAT_TOLERANCE	10
#define UFCS_PR_CC2CV_TIMEOUT_MS	3000
#define UFCS_PR_CC_CHECK_CNT		5
#define UFCS_PR_CC_VBUS_REQ_TOLERANCE	20

#define UFCS_PR_CV_FULL_CHECK_CNT	10
#define UFCS_PR_CV_DOWN_CHECK_CNT	5
#define UFCS_PR_CV_DOWN_IBUS_TOLERANCE	50
#define UFCS_PR_CV_VBAT_TOLERANCE	10
#define UFCS_PR_CV_LEDOFF_IBAT_DIFF_MA	1000
#define UFCS_PR_CV_IBUS_LARGET_TOLERANCE	200
#define UFCS_PR_CV_IBUS_LARGE_TIMEOUT_MS	1000
#define UFCS_PR_CV_IBUS_CHECK_MIN_MA	1800
#define UFCS_PR_CV_IBUS_CHECK_CNT	5
#define UFCS_PR_CV_IBUS_CHECK_INC_MA	200
#define UFCS_PR_BCC_DOWN_STEP_MA	1000
#define UFCS_PR_BCC_DOWN_HOLD_MS	10000

#define UFCS_PR_PDO_ERR_MAX_STEP_MA	1000
#define UFCS_PR_PDO_ERR_MIN_CURR_MA	1000
#define UFCS_PR_PDO_ERR_PRECISION	50

#define UFCS_PR_FULL_COMP_VOL_MV	20

#define UFCS_PR_MIN_REQ_CURRENT		500

#define UFCS_START_DEF_CURR_MA      2000

static void ufcs_pr_parse_dt(struct oplus_ufcs *chip, struct device_node *node)
{
	struct ufcs_pr_config *pr_config = &chip->pr_config;
	int rc = 0;

	pr_config->support_pr = of_property_read_bool(node, "oplus,support_pr");

	if (!pr_config->support_pr)
		return;

	rc = of_property_read_u32(node, "oplus,pr_r_input_mohm", &pr_config->pr_r_input_mohm);
	if (rc < 0)
		pr_config->pr_r_input_mohm = UFCS_PR_DEFAULT_R_INPUT_MOHM;

	rc = of_property_read_u32(node, "oplus,pr_r_btb_mohm", &pr_config->pr_r_btb_mohm);
	if (rc < 0)
		pr_config->pr_r_btb_mohm = UFCS_PR_DEFAULT_R_BTB_MOHM;
}

static void ufcs_pr_deal_plc(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (chip->plc_status == PLC_STATUS_ENABLE && pr_data->target_current_ma < PLC_IBUS_MAX) {
		pr_data->plc_current_ma = PLC_IBUS_MAX;
		oplus_ufcs_cp_set_ucp_disable(chip, true);
	} else {
		pr_data->plc_current_ma = -EINVAL;
	}
}

static void ufcs_pr_deal_falling(struct oplus_ufcs *chip, bool falling)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (falling)
		pr_data->falling_current_ma = pr_data->target_current_ma;
	else
		pr_data->falling_current_ma = 0;
}

static void ufcs_pr_update_request_current(struct oplus_ufcs *chip, bool falling)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int *current_vars[] = {
		&pr_data->emark_current_ma,
		&pr_data->adapter_current_ma,
		&pr_data->pdo_current_ma,
		&pr_data->curve_current_ma,
		&pr_data->plc_current_ma,
		&pr_data->falling_current_ma,
		&pr_data->err_current_ma,
	};
	int current_ma = chip->config.curr_max_ma;
	size_t i;

	ufcs_pr_deal_plc(chip);
	ufcs_pr_deal_falling(chip, falling);

	for (i = 0; i < ARRAY_SIZE(current_vars); i++) {
		if (current_vars[i] && *current_vars[i] > 0 && *current_vars[i] < current_ma)
			current_ma = *current_vars[i];
	}

	pr_data->request_current_ma = max(current_ma, UFCS_PR_MIN_REQ_CURRENT);
}

static void ufcs_pr_update_request_voltage(struct oplus_ufcs *chip, int voltage, bool falling)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int vol = falling ? max(pr_data->pmic_vbus_mv, voltage) : voltage;

	pr_data->request_voltage_mv = rounddown(min(pr_data->req_vbus_max_mv, vol), UFCS_PR_PDO_STEP_MV);
}

static void ufcs_pr_align_pdo(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int i;
	u64 pdo;
	int vol_mv = pr_data->request_voltage_mv;
	int curr_ma = pr_data->request_current_ma;

	for (i = 0; i < chip->pdo_num; i++) {
		pdo = chip->pdo[i];

		if (vol_mv < UFCS_OUTPUT_MODE_VOL_MIN(pdo))
			continue;
		if (vol_mv > UFCS_OUTPUT_MODE_VOL_MAX(pdo))
			continue;
		if (curr_ma < UFCS_OUTPUT_MODE_CURR_MIN(pdo))
			continue;
		if (curr_ma > UFCS_OUTPUT_MODE_CURR_MAX(pdo))
			continue;

		pr_data->request_voltage_mv = UFCS_OUTPUT_MODE_VOL_MIN(pdo) +
			rounddown(vol_mv - UFCS_OUTPUT_MODE_VOL_MIN(pdo), UFCS_OUTPUT_MODE_VOL_STEP(pdo));
		pr_data->request_current_ma = UFCS_OUTPUT_MODE_CURR_MIN(pdo) +
			rounddown(curr_ma - UFCS_OUTPUT_MODE_CURR_MIN(pdo), UFCS_OUTPUT_MODE_CURR_STEP(pdo));
		if (pr_data->request_voltage_mv != vol_mv || pr_data->request_current_ma != curr_ma)
			chg_info("[%dmV %dmA] align to [%dmV %dmA]\n",
				vol_mv, curr_ma,
				pr_data->request_voltage_mv, pr_data->request_current_ma);
		break;
	}
}

static int ufcs_pr_set_pdo(struct oplus_ufcs *chip, int voltage, bool falling)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int rc = EALREADY;

	pr_data->falling_cc_adjust = falling;
	ufcs_pr_update_request_current(chip, falling);
	ufcs_pr_update_request_voltage(chip, voltage, falling);
	ufcs_pr_align_pdo(chip);

	if (pr_data->request_voltage_mv != chip->vol_set_mv || pr_data->request_current_ma != chip->curr_set_ma) {
		rc = oplus_ufcs_cp_set_iin(chip, pr_data->target_current_ma);
		if (rc < 0) {
			chg_err("set cp input current error, rc=%d\n", rc);
			return rc;
		}

		if (oplus_wired_get_bcc_curr_done_status(chip->wired_topic) == BCC_CURR_DONE_REQUEST)
			oplus_wired_check_bcc_curr_done(chip->wired_topic);
		rc = oplus_ufcs_pdo_set(chip, pr_data->request_voltage_mv, pr_data->request_current_ma);
		if (rc < 0) {
			chg_err("pdo set error, rc=%d\n", rc);
			pr_data->pdo_err = true;
			return rc;
		}
		pr_data->adjust = true;
		pr_data->adjust_jiffies = jiffies;
		pr_data->pdo_err = false;
	}

	return rc;
}

static void ufcs_pr_init(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int rc = 0;

	memset(pr_data, 0, sizeof(struct ufcs_pr_data));

	pr_data->curr_state = UFCS_PR_IDLE;
	pr_data->prev_state = UFCS_PR_IDLE;

	pr_data->bcc_target_update_jiffies = jiffies;

	pr_data->target_current_ma = UFCS_START_DEF_CURR_MA;
	pr_data->req_vbus_max_mv = chip->config.target_vbus_mv;
	pr_data->request_voltage_mv = min(pr_data->req_vbus_max_mv, chip->vol_set_mv);
	pr_data->request_current_ma = chip->curr_set_ma;

	pr_data->emark_current_ma = get_client_vote(chip->ufcs_curr_votable, CABLE_MAX_VOTER);
	pr_data->adapter_current_ma = get_client_vote(chip->ufcs_curr_votable, ADAPTER_IMAX_VOTER);
	pr_data->pdo_current_ma = get_client_vote(chip->ufcs_curr_votable, BASE_MAX_VOTER);

	rc = oplus_chg_strategy_get_metadata(chip->strategy, &pr_data->curve);
	if (rc < 0 || !pr_data->curve.data || pr_data->curve.num < 1) {
		chg_err("can't get charger curve %d 0x%p %d\n", rc, pr_data->curve.data, pr_data->curve.num);
		pr_data->curve_current_ma = -EINVAL;
		pr_data->vbat_full_mv = 4500;
	} else {
		pr_data->curve_current_ma = pr_data->curve.data[0].target_ibus;
		pr_data->vbat_full_mv = pr_data->curve.data[pr_data->curve.num - 1].target_vbat;
	}
	ufcs_pr_deal_plc(chip);

	chg_info("emark:%d adapter%d pdo:%d curve:%d plc:%d max:%d\n",
		pr_data->emark_current_ma,
		pr_data->adapter_current_ma,
		pr_data->pdo_current_ma,
		pr_data->curve_current_ma,
		pr_data->plc_current_ma,
		pr_data->max_current_ma);
}

static void ufcs_pr_set_state(struct oplus_ufcs *chip, enum ufcs_pr_state state)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (state < 0 || state >= UFCS_PR_MAX) {
		chg_err("Invalid state: %d (current: %d)\n", state, pr_data->curr_state);
		return;
	}

	if (state == pr_data->curr_state)
		return;

	if (pr_data->prev_state == UFCS_PR_PDO_ERR)
		pr_data->adjust_rising_cnt = 0;

	if (pr_data->prev_state == UFCS_PR_CV) {
		pr_data->cv_ibus_larget_jiffies = jiffies;
		pr_data->cv_delta_cnt = 0;
		pr_data->cv_full_cnt = 0;
		pr_data->cv_iterm_cnt = 0;
		pr_data->cv_target_cnt = 0;
		pr_data->cv_ibus_cnt = 0;
	}

	pr_data->prev_state = pr_data->curr_state;
	pr_data->curr_state = state;

	if (pr_data->curr_state == UFCS_PR_CC) {
		pr_data->entry_cv_jiffies = jiffies;
		pr_data->cc_cnt = 0;
	}

	if (pr_data->curr_state == UFCS_PR_CV) {
		pr_data->adjust_rising_cnt = 0;
		pr_data->adjust_rising_adapter_vbus_mv = pr_data->adapter_vbus_mv;
		pr_data->adjust_rising_req_vbus_mv = chip->vol_set_mv;
	}

	if (pr_data->curr_state == UFCS_PR_PDO_ERR) {
		pr_data->pdo_err_high = pr_data->request_current_ma;
		pr_data->pdo_err_low = UFCS_PR_PDO_ERR_MIN_CURR_MA;
	}
}

struct ufcs_pr_handler {
	const char *name;
	int (*handle)(struct oplus_ufcs *chip);
};

static int ufcs_pr_handle_idle(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (pr_data->vbat_mv < pr_data->vbat_target_mv_by_ibus &&
	    (pr_data->adapter_ibus_ma + UFCS_PR_CHANGE_THRES_MA <= chip->target_curr_ma))
		ufcs_pr_set_state(chip, UFCS_PR_RISING);
	else if (pr_data->adapter_ibus_ma - UFCS_PR_CHANGE_THRES_MA >= chip->target_curr_ma)
		ufcs_pr_set_state(chip, UFCS_PR_FALLING);
	else
		ufcs_pr_set_state(chip, UFCS_PR_CC);

	pr_data->target_current_ma = chip->target_curr_ma;

	return UFCS_PR_FAST_DELAY_MS;
}

static bool ufcs_pr_check_adapter_limit(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (pr_data->adjust_rising_cnt == 0) {
		pr_data->adjust_rising_adapter_vbus_mv = pr_data->adapter_vbus_mv;
		pr_data->adjust_rising_req_vbus_mv = chip->vol_set_mv;
	}

	if (pr_data->adjust_rising_cnt >= UFCS_PR_RISING_ADJUST_CHECK_CNT) {
		pr_data->adjust_rising_cnt = 0;
		if ((pr_data->adapter_vbus_mv -pr_data->adjust_rising_adapter_vbus_mv) <=
			   ((chip->vol_set_mv - pr_data->adjust_rising_req_vbus_mv) *
			   UFCS_PR_RISING_ADJUST_CHECK_PCT / 100)) {
			chg_info("adapter limit req %d %d, adapter %d %d\n",
				chip->vol_set_mv, pr_data->adjust_rising_req_vbus_mv,
				pr_data->adapter_vbus_mv, pr_data->adjust_rising_adapter_vbus_mv);
			ufcs_pr_set_pdo(chip, pr_data->adapter_vbus_mv, false);
			pr_data->adjust_rising_adapter_vbus_mv = pr_data->adapter_vbus_mv;
			pr_data->adjust_rising_req_vbus_mv = chip->vol_set_mv;
			if (chip->plc_status != PLC_STATUS_ENABLE)
				vote(chip->ufcs_curr_votable, PR_VOTER, true, pr_data->adapter_ibus_ma, false);
			ufcs_pr_set_state(chip, UFCS_PR_CC);
			return true;
		}
		pr_data->adjust_rising_adapter_vbus_mv = pr_data->adapter_vbus_mv;
		pr_data->adjust_rising_req_vbus_mv = chip->vol_set_mv;
	}

	return false;
}

static int ufcs_pr_get_step_by_ibus_diff(int ibus_diff)
{
	struct ibus_diff_step_map {
		int ibus_diff;
		int step;
	} const maps[] = {
		{ 1000, 300 },
		{ 500, 100 },
		{ 100, 40 },
	};
	size_t i = 0;

	for (i = 0; i < ARRAY_SIZE(maps); i++) {
		if (ibus_diff >= maps[i].ibus_diff)
			return maps[i].step;
	}

	return 0;
}

static int ufcs_pr_rising_handle_small_step(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int delay_ms = UFCS_PR_DELAY_MS;
	int rc = 0;

	if (pr_data->pmic_vbat_mv > pr_data->vbat_full_comp_mv) {
		ufcs_pr_set_pdo(chip, chip->vol_set_mv - pr_data->pmic_vbat_mv + pr_data->vbat_full_comp_mv, false);
		pr_data->adjust_rising_cnt = 0;
		ufcs_pr_set_state(chip, UFCS_PR_CC);
		return delay_ms;
	}

	if (pr_data->vbat_mv >= (pr_data->vbat_full_mv - UFCS_PR_RISING_VFULL_TOLERANCE)) {
		if (chip->plc_status != PLC_STATUS_ENABLE)
			vote(chip->ufcs_curr_votable, PR_VOTER, true, pr_data->adapter_ibus_ma, false);
		pr_data->adjust_rising_cnt = 0;
		ufcs_pr_set_state(chip, UFCS_PR_CC);
		return UFCS_PR_FAST_DELAY_MS;
	}

	if (chip->vol_set_mv > pr_data->req_vbus_max_mv - UFCS_PR_VSTEP1) {
		pr_data->adjust_rising_cnt = 0;
		ufcs_pr_set_state(chip, UFCS_PR_CV);
		return UFCS_PR_FAST_DELAY_MS;
	}

	if (ufcs_pr_check_adapter_limit(chip))
		return UFCS_PR_FAST_DELAY_MS;

	rc = ufcs_pr_set_pdo(chip, chip->vol_set_mv + UFCS_PR_VSTEP1, false);
	if (!rc)
		pr_data->adjust_rising_cnt++;

	return delay_ms;
}

static int ufcs_pr_rising_handle_large_step(struct oplus_ufcs *chip, int step_mv)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int delay_ms = UFCS_PR_DELAY_MS;
	int rc;

	if (ufcs_pr_check_adapter_limit(chip))
		return UFCS_PR_FAST_DELAY_MS;

	rc = ufcs_pr_set_pdo(chip, chip->vol_set_mv + step_mv, false);
	if (!rc)
		pr_data->adjust_rising_cnt++;

	return delay_ms;
}

static int ufcs_pr_handle_rising_no_adjust(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int ibus_diff = pr_data->adapter_ibus_ma - pr_data->target_current_ma;
	int step_mv = rounddown(min3(pr_data->vbus_max_mv - pr_data->adapter_vbus_mv,
		UFCS_PR_RISING_STEP_MAX_MV, ufcs_pr_get_step_by_ibus_diff(-ibus_diff)), UFCS_PR_VSTEP1);

	if (pr_data->adapter_vbus_mv > pr_data->vbus_max_mv || step_mv <= UFCS_PR_VSTEP1)
		return ufcs_pr_rising_handle_small_step(chip);

	return ufcs_pr_rising_handle_large_step(chip, step_mv);
}

static int ufcs_pr_handle_rising_adjust(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int ibus_diff = pr_data->adapter_ibus_ma - pr_data->target_current_ma;
	int delay_ms = UFCS_PR_DELAY_MS;

	if (pr_data->pmic_vbat_mv > pr_data->vbat_full_comp_mv) {
		ufcs_pr_set_pdo(chip, chip->vol_set_mv - pr_data->pmic_vbat_mv + pr_data->vbat_full_comp_mv, false);
		pr_data->adjust_rising_cnt = 0;
		ufcs_pr_set_state(chip, UFCS_PR_CC);
	} else if (time_is_before_eq_jiffies(pr_data->adjust_jiffies + msecs_to_jiffies(UFCS_PR_ADJUST_TIMEOUT_MS))) {
		pr_data->adjust = false;
		delay_ms = UFCS_PR_FAST_DELAY_MS;
		if (chip->target_curr_ma != pr_data->target_current_ma) {
			ufcs_pr_set_state(chip, UFCS_PR_IDLE);
			delay_ms = UFCS_PR_FAST_DELAY_MS;
		} else if (ibus_diff >= 0) {
			pr_data->adjust_rising_cnt = 0;
			ufcs_pr_set_state(chip, UFCS_PR_IDLE);
			delay_ms = UFCS_PR_FAST_DELAY_MS;
		} else if (pr_data->vbat_mv >= pr_data->vbat_target_mv_by_ibus) {
			pr_data->adjust_rising_cnt = 0;
			vote(chip->ufcs_curr_votable, PR_VOTER, true, pr_data->adapter_ibus_ma, false);
			ufcs_pr_set_state(chip, UFCS_PR_IDLE);
			delay_ms = UFCS_PR_FAST_DELAY_MS;
		}
	}

	return delay_ms;
}

static int ufcs_pr_handle_rising(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (!pr_data->adjust)
		return ufcs_pr_handle_rising_no_adjust(chip);

	return ufcs_pr_handle_rising_adjust(chip);
}

static int ufcs_pr_handle_falling_no_adjust(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int ibus_delta = 0;
	int vbus_delta = 0;
	int delay_ms = UFCS_PR_DELAY_MS;

	ibus_delta = pr_data->adapter_ibus_ma - pr_data->target_current_ma;
	vbus_delta = chip->vol_set_mv - pr_data->vbus_to_vbat_ratio * pr_data->vbat_mv;
	if (chip->plc_status == PLC_STATUS_ENABLE && chip->oplus_ufcs_adapter) {
		ufcs_pr_set_pdo(chip, chip->vol_set_mv - oplus_ufcs_get_update_vstep(chip,
			abs(pr_data->adapter_ibus_ma - pr_data->target_current_ma)), true);
		pr_data->adjust_rising_cnt = 0;
	} else if (ibus_delta <= UFCS_PR_FALLING_IBUS_DELAT_MAX || vbus_delta < UFCS_PR_FALLING_VBUS_DELAT_MAX) {
		ufcs_pr_set_pdo(chip, chip->vol_set_mv - UFCS_PR_VSTEP1, false);
		pr_data->adjust_rising_cnt = 0;
	} else {
		ufcs_pr_set_pdo(chip, chip->vol_set_mv - min(UFCS_PR_FALLING_VSTEP_MAX,
			vbus_delta - UFCS_PR_FALLING_VBUS_DELAT_MAX), true);
		pr_data->adjust_rising_cnt = 0;
	}

	return delay_ms;
}

static int ufcs_pr_handle_falling_adjust(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int delay_ms = UFCS_PR_DELAY_MS;

	if (time_is_before_eq_jiffies(pr_data->adjust_jiffies + msecs_to_jiffies(UFCS_PR_ADJUST_TIMEOUT_MS))) {
		pr_data->adjust = false;
		delay_ms = UFCS_PR_FAST_DELAY_MS;
		if (chip->target_curr_ma != pr_data->target_current_ma) {
			if (pr_data->falling_cc_adjust) {
				ufcs_pr_set_pdo(chip, pr_data->adapter_vbus_mv, true);
				ufcs_pr_set_pdo(chip, pr_data->adapter_vbus_mv, false);
			}
			pr_data->adjust = false;
			ufcs_pr_set_state(chip, UFCS_PR_IDLE);
			delay_ms = UFCS_PR_FAST_DELAY_MS;
		} else if (pr_data->adapter_ibus_ma - UFCS_PR_FALLING_IBUS_TOLERANCE <= chip->target_curr_ma) {
			if (pr_data->falling_cc_adjust) {
				ufcs_pr_set_pdo(chip, pr_data->adapter_vbus_mv, true);
				ufcs_pr_set_pdo(chip, pr_data->adapter_vbus_mv, false);
			}
			pr_data->adjust = false;
			ufcs_pr_set_state(chip, UFCS_PR_CC);
			pr_data->adjust_rising_cnt = 0;
			delay_ms = UFCS_PR_FAST_DELAY_MS;
		}
	}

	return delay_ms;
}

static int ufcs_pr_handle_falling(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (!pr_data->adjust)
		return ufcs_pr_handle_falling_no_adjust(chip);

	return ufcs_pr_handle_falling_adjust(chip);
}

static bool ufcs_pr_cc_precheck(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int ibus_delta;
	int tolerance = UFCS_PR_CC_IBUS_TOLERANCE;

	if (chip->target_curr_ma != pr_data->target_current_ma) {
		ufcs_pr_set_state(chip, UFCS_PR_IDLE);
		*delay_ms = UFCS_PR_FAST_DELAY_MS;
		return true;
	}

	if (pr_data->pmic_vbat_mv > pr_data->vbat_full_comp_mv) {
		ufcs_pr_set_pdo(chip, chip->vol_set_mv - pr_data->pmic_vbat_mv + pr_data->vbat_full_comp_mv, false);
		pr_data->adjust_rising_cnt = 0;
		pr_data->cc_cnt = 0;
		return true;
	}

	ibus_delta = pr_data->adapter_ibus_ma - pr_data->target_current_ma;
	if (chip->plc_status == PLC_STATUS_ENABLE && chip->oplus_ufcs_adapter)
		tolerance = UFCS_PR_CC_PLC_IBUS_TOLERANCE;

	if (ibus_delta >= tolerance)
		pr_data->cc_cnt++;
	else if (ibus_delta <= -tolerance)
		pr_data->cc_cnt--;

	return false;
}

static bool ufcs_pr_cc_check_cv_curve(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (pr_data->cv_curve) {
		if ((pr_data->last_curve &&
		    pr_data->vbat_mv >= (pr_data->vbat_target_mv - UFCS_PR_CC2CV_VBAT_TOLERANCE)) ||
		    (!pr_data->last_curve && pr_data->vbat_mv >= pr_data->vbat_target_mv)) {
			if (time_is_before_eq_jiffies(pr_data->entry_cv_jiffies +
			    msecs_to_jiffies(UFCS_PR_CC2CV_TIMEOUT_MS))) {
				if (pr_data->last_curve) {
					pr_data->req_vbus_max_mv = chip->vol_set_mv;
					chg_info("entry last curve, vbus limit %dmV\n", pr_data->req_vbus_max_mv);
				}
				ufcs_pr_set_state(chip, UFCS_PR_CV);
				pr_data->entry_cv_jiffies = jiffies;
				pr_data->adjust_rising_cnt = 0;
				*delay_ms = UFCS_PR_FAST_DELAY_MS;
				return true;
			}
		} else {
			pr_data->entry_cv_jiffies = jiffies;
		}
	} else {
		pr_data->entry_cv_jiffies = jiffies;
	}

	return false;
}

static int ufcs_pr_cc_adjust_vbus(struct oplus_ufcs *chip, int delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int rc = 0;

	if (pr_data->cc_cnt >= UFCS_PR_CC_CHECK_CNT) {
		pr_data->cc_cnt = 0;
		if (chip->plc_status == PLC_STATUS_ENABLE && chip->oplus_ufcs_adapter) {
			ufcs_pr_set_pdo(chip, chip->vol_set_mv - oplus_ufcs_get_update_vstep(chip,
				abs(pr_data->adapter_ibus_ma - pr_data->target_current_ma)), false);
			pr_data->adjust_rising_cnt = 0;
		} else if ((chip->vol_set_mv - UFCS_PR_VSTEP2) >=
		    (pr_data->vbus_to_vbat_ratio * pr_data->vbat_mv + UFCS_PR_CC_VBUS_REQ_TOLERANCE)) {
			ufcs_pr_set_pdo(chip, chip->vol_set_mv - UFCS_PR_VSTEP2, false);
			pr_data->adjust_rising_cnt = 0;
		} else {
			ufcs_pr_set_state(chip, UFCS_PR_CV);
			pr_data->adjust_rising_cnt = 0;
			delay_ms = UFCS_PR_FAST_DELAY_MS;
		}
	} else if (pr_data->cc_cnt <= -UFCS_PR_CC_CHECK_CNT) {
		pr_data->cc_cnt = 0;
		if (ufcs_pr_check_adapter_limit(chip))
			return UFCS_PR_FAST_DELAY_MS;

		rc = ufcs_pr_set_pdo(chip, chip->vol_set_mv + UFCS_PR_VSTEP2, false);
		if (!rc)
			pr_data->adjust_rising_cnt++;
	}

	return delay_ms;
}

static int ufcs_pr_handle_cc(struct oplus_ufcs *chip)
{
	int delay_ms = UFCS_PR_DELAY_MS;

	if (ufcs_pr_cc_precheck(chip, &delay_ms))
		return delay_ms;

	if (ufcs_pr_cc_check_cv_curve(chip, &delay_ms))
		return delay_ms;

	return ufcs_pr_cc_adjust_vbus(chip, delay_ms);
}

static bool ufcs_pr_cv_basic_check(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (!pr_data->cv_curve) {
		chg_err("not cv curve\n");
		ufcs_pr_set_state(chip, UFCS_PR_IDLE);
		*delay_ms = UFCS_PR_FAST_DELAY_MS;
		return true;
	}

	if (chip->target_curr_ma != pr_data->target_current_ma) {
		ufcs_pr_set_state(chip, UFCS_PR_IDLE);
		*delay_ms = UFCS_PR_FAST_DELAY_MS;
		return true;
	}

	if (pr_data->adapter_ibus_ma >= pr_data->target_current_ma + UFCS_PR_CV_IBUS_LARGET_TOLERANCE) {
		if (time_is_before_eq_jiffies(pr_data->cv_ibus_larget_jiffies +
		    msecs_to_jiffies(UFCS_PR_CV_IBUS_LARGE_TIMEOUT_MS))) {
			chg_info("adapter_ibus_ma:%d too large\n", pr_data->adapter_ibus_ma);
			ufcs_pr_set_state(chip, UFCS_PR_IDLE);
			*delay_ms = UFCS_PR_FAST_DELAY_MS;
			return true;
		}
	} else {
		pr_data->cv_ibus_larget_jiffies = jiffies;
	}

	return false;
}

static void ufcs_pr_update_bcc_target(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int ibat_from_ibus;
	int ibat_ceiled;
	int next_curve_ibat;
	int new_target;
	int min_limit;

	if (pr_data->adapter_ibus_ma <= 0)
		return;

	ibat_from_ibus = pr_data->adapter_ibus_ma * pr_data->ibus_to_ibat_ratio;
	ibat_ceiled = DIV_ROUND_UP(ibat_from_ibus, 100) * 100;

	if (pr_data->bcc_target_ma <= 0 || pr_data->bcc_target_ma > pr_data->ibat_target_ma)
		pr_data->bcc_target_ma = pr_data->ibat_target_ma;

	if (pr_data->last_curve) {
		next_curve_ibat = pr_data->iterm_ma;
		min_limit = pr_data->iterm_ma;
	} else {
		next_curve_ibat = pr_data->next_ibus_target_ma * pr_data->ibus_to_ibat_ratio;
		min_limit = next_curve_ibat + 100;
		if (pr_data->bcc_target_ma - next_curve_ibat < UFCS_PR_BCC_DOWN_STEP_MA)
			return;
	}

	if (ibat_ceiled <= pr_data->bcc_target_ma - UFCS_PR_BCC_DOWN_STEP_MA) {
		if (time_before(jiffies, pr_data->bcc_target_update_jiffies +
		    msecs_to_jiffies(UFCS_PR_BCC_DOWN_HOLD_MS)))
			return;

		new_target = max(pr_data->bcc_target_ma - UFCS_PR_BCC_DOWN_STEP_MA, min_limit);
		pr_data->bcc_target_ma = min(pr_data->bcc_target_ma, new_target);
		pr_data->bcc_target_update_jiffies = jiffies;
	}
}

static void ufcs_pr_check_bcc_curr(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int bcc_min_curr, bcc_max_curr, bcc_exit_curr;
	int bcc_target_ma = pr_data->bcc_target_ma > 0 ? pr_data->bcc_target_ma : pr_data->ibat_target_ma;
	int bcc_min_raw;
	int bcc_min_floor;

	bcc_min_raw = bcc_target_ma / UFCS_BATT_CURR_TO_BCC_CURR;
	bcc_exit_curr = pr_data->iterm_ma;
	bcc_min_floor = ((max(bcc_min_raw - UFCS_BCC_CURRENT_MIN, 0)) / BCC_CURRENT_MIN_STEP) * BCC_CURRENT_MIN_STEP;
	bcc_min_curr = max_t(int, bcc_exit_curr / UFCS_BATT_CURR_TO_BCC_CURR, bcc_min_floor);
	bcc_max_curr = bcc_min_raw;

	chip->bcc.bcc_min_curr = bcc_min_curr;
	chip->bcc.bcc_max_curr = bcc_max_curr;
	chip->bcc.bcc_exit_curr = bcc_exit_curr;
}

static void ufcs_pr_cv_handle_vbat_ge_target_last(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (pr_data->ibat_ma <= pr_data->iterm_ma) {
		pr_data->cv_full_cnt++;
		pr_data->cv_iterm_cnt = 0;
		if (pr_data->cv_full_cnt >= UFCS_PR_CV_FULL_CHECK_CNT) {
			pr_data->full = true;
			chg_info("full\n");
			ufcs_pr_set_state(chip, UFCS_PR_FULL);
			*delay_ms = UFCS_PR_FAST_DELAY_MS;
			return;
		}
	} else {
		pr_data->cv_full_cnt = 0;
		pr_data->cv_iterm_cnt++;
		if (pr_data->adapter_ibus_ma <= UFCS_PR_CV_IBUS_CHECK_MIN_MA)
			pr_data->cv_ibus_cnt++;
		else
			pr_data->cv_ibus_cnt = 0;

		if (pr_data->cv_iterm_cnt >= UFCS_PR_CV_DOWN_CHECK_CNT) {
			pr_data->cv_iterm_cnt = 0;
			ufcs_pr_set_pdo(chip, chip->vol_set_mv - UFCS_PR_VSTEP2, false);
			if (pr_data->cv_ibus_cnt >= UFCS_PR_CV_IBUS_CHECK_CNT)
				pr_data->req_vbus_max_mv = min((int)chip->config.target_vbus_mv,
					chip->vol_set_mv + UFCS_PR_CV_IBUS_CHECK_INC_MA);
			else
				pr_data->req_vbus_max_mv = chip->vol_set_mv;
			pr_data->adjust_rising_cnt = 0;
			ufcs_pr_update_bcc_target(chip);
		}
	}
}

static void ufcs_pr_cv_handle_vbat_ge_target_nonlast(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	pr_data->cv_full_cnt = 0;
	pr_data->cv_ibus_cnt = 0;

	if (pr_data->cv_target_cnt >= UFCS_PR_CV_DOWN_CHECK_CNT) {
		pr_data->cv_target_cnt = 0;
		if (pr_data->adapter_ibus_ma <= pr_data->next_ibus_target_ma + UFCS_PR_CV_DOWN_IBUS_TOLERANCE) {
			chg_info("ibus < curve+50mA\n");
			if (pr_data->adapter_ibus_ma <= pr_data->target_current_ma - UFCS_PR_CC_IBUS_TOLERANCE) {
				ufcs_pr_set_state(chip, UFCS_PR_IDLE);
				*delay_ms = UFCS_PR_FAST_DELAY_MS;
			}
		} else {
			ufcs_pr_set_pdo(chip, chip->vol_set_mv - UFCS_PR_VSTEP2, false);
			pr_data->adjust_rising_cnt = 0;
			ufcs_pr_update_bcc_target(chip);
		}
	}
}

static void ufcs_pr_cv_handle_vbat_ge_target(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	pr_data->cv_delta_cnt = 0;
	if (pr_data->pmic_vbat_mv > pr_data->vbat_full_comp_mv) {
		ufcs_pr_set_pdo(chip, chip->vol_set_mv - pr_data->pmic_vbat_mv + pr_data->vbat_full_comp_mv, false);
		pr_data->adjust_rising_cnt = 0;
		pr_data->cv_full_cnt = 0;
		pr_data->cv_target_cnt = 0;
		pr_data->cv_ibus_cnt = 0;
		return;
	}

	pr_data->cv_target_cnt++;
	if (pr_data->last_curve)
		ufcs_pr_cv_handle_vbat_ge_target_last(chip, delay_ms);
	else
		ufcs_pr_cv_handle_vbat_ge_target_nonlast(chip, delay_ms);
}

static bool ufcs_pr_cv_handle_vbat_lt_led_off(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	if (pr_data->adapter_ibus_ma * pr_data->ibus_to_ibat_ratio - pr_data->ibat_ma <
	    UFCS_PR_CV_LEDOFF_IBAT_DIFF_MA) {
		pr_data->cv_delta_cnt++;
		if (pr_data->cv_delta_cnt >= 5) {
			pr_data->cv_delta_cnt = 0;
			if (pr_data->req_vbus_max_mv - chip->vol_set_mv >= UFCS_PR_VSTEP2) {
				if (ufcs_pr_check_adapter_limit(chip)) {
					*delay_ms = UFCS_PR_FAST_DELAY_MS;
					return true;
				}
				ufcs_pr_set_pdo(chip, chip->vol_set_mv + UFCS_PR_VSTEP2, false);
				pr_data->adjust_rising_cnt = 0;
				pr_data->cv_full_cnt = 0;
				pr_data->cv_target_cnt = 0;
				pr_data->cv_iterm_cnt = 0;
			}
		}
	} else {
		pr_data->cv_delta_cnt = 0;
	}

	return false;
}

static void ufcs_pr_cv_handle_vbat_lt_target(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	pr_data->cv_target_cnt = 0;
	pr_data->cv_full_cnt = 0;
	pr_data->cv_iterm_cnt = 0;
	pr_data->cv_ibus_cnt = 0;

	if (pr_data->pmic_vbat_mv > pr_data->vbat_full_comp_mv) {
		ufcs_pr_set_pdo(chip, chip->vol_set_mv - pr_data->pmic_vbat_mv + pr_data->vbat_full_comp_mv, false);
		pr_data->adjust_rising_cnt = 0;
		pr_data->cv_full_cnt = 0;
		pr_data->cv_target_cnt = 0;
		pr_data->cv_delta_cnt = 0;
	} else if (pr_data->vbat_mv <= pr_data->vbat_target_mv - UFCS_PR_CV_VBAT_TOLERANCE) {
		if (pr_data->adapter_ibus_ma > pr_data->target_current_ma - UFCS_PR_CC_IBUS_TOLERANCE)
			return;

		if (chip->led_on) {
			pr_data->cv_delta_cnt = 0;
			return;
		}

		if (ufcs_pr_cv_handle_vbat_lt_led_off(chip, delay_ms))
			return;
	} else {
		pr_data->cv_delta_cnt = 0;
	}
}

static int ufcs_pr_handle_cv(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int delay_ms = UFCS_PR_DELAY_MS;

	if (ufcs_pr_cv_basic_check(chip, &delay_ms))
		return delay_ms;

	if (pr_data->vbat_mv >= pr_data->vbat_target_mv)
		ufcs_pr_cv_handle_vbat_ge_target(chip, &delay_ms);
	else
		ufcs_pr_cv_handle_vbat_lt_target(chip, &delay_ms);

	return delay_ms;
}

static int ufcs_pr_handle_full(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int delay_ms = UFCS_PR_FULL_DELAY_MS;

	if (!pr_data->full) {
		chg_err("not full\n");
		ufcs_pr_set_state(chip, UFCS_PR_IDLE);
		delay_ms = UFCS_PR_FAST_DELAY_MS;
		goto done;
	}

done:
	return delay_ms;
}

static bool ufcs_pr_handle_pdo_err_finish(struct oplus_ufcs *chip, int *delay_ms)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int rc;

	if (pr_data->pdo_err_high - pr_data->pdo_err_low <= UFCS_PR_PDO_ERR_PRECISION) {
		chg_err("high:%d low:%d req:%d\n", pr_data->pdo_err_high, pr_data->pdo_err_low,
			pr_data->request_current_ma);
		pr_data->request_current_ma = pr_data->pdo_err_low;
		ufcs_pr_align_pdo(chip);
		rc = oplus_ufcs_pdo_set(chip, chip->pr_data.request_voltage_mv, pr_data->request_current_ma);
		if (rc) {
			pr_data->pdo_err = true;
			if (pr_data->pdo_err_high <= UFCS_PR_PDO_ERR_MIN_CURR_MA) {
				vote(chip->ufcs_disable_votable, PR_VOTER, true, 0, false);
				*delay_ms = UFCS_PR_PDO_ERR_DELAY_MS;
			} else {
				pr_data->pdo_err_high = pr_data->request_current_ma;
				pr_data->pdo_err_low = UFCS_PR_PDO_ERR_MIN_CURR_MA;
				chg_err("high:%d low:%d req:%d\n", pr_data->pdo_err_high,
					pr_data->pdo_err_low, pr_data->request_current_ma);
			}
		} else {
			pr_data->err_current_ma = pr_data->request_current_ma;
			rc = ufcs_pr_set_pdo(chip, pr_data->adapter_vbus_mv, false);
			if (rc < 0) {
				vote(chip->ufcs_disable_votable, PR_VOTER, true, 0, false);
				*delay_ms = UFCS_PR_PDO_ERR_DELAY_MS;
			} else {
				pr_data->adjust = false;
				ufcs_pr_set_state(chip, UFCS_PR_IDLE);
				*delay_ms = UFCS_PR_FAST_DELAY_MS;
			}
		}
		return true;
	}

	return false;
}

static int ufcs_pr_handle_pdo_err(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int delay_ms = UFCS_PR_DELAY_MS;
	int mid;
	int next_current;
	int diff;
	int rc;

	if (ufcs_pr_handle_pdo_err_finish(chip, &delay_ms))
		return delay_ms;

	mid = (pr_data->pdo_err_high + pr_data->pdo_err_low) / 2;
	diff = mid - pr_data->request_current_ma;
	if (diff > UFCS_PR_PDO_ERR_MAX_STEP_MA)
		next_current = pr_data->request_current_ma + UFCS_PR_PDO_ERR_MAX_STEP_MA;
	else if (diff < -UFCS_PR_PDO_ERR_MAX_STEP_MA)
		next_current = pr_data->request_current_ma - UFCS_PR_PDO_ERR_MAX_STEP_MA;
	else
		next_current = mid;

	pr_data->request_current_ma = next_current;
	ufcs_pr_align_pdo(chip);
	rc = oplus_ufcs_pdo_set(chip, chip->pr_data.request_voltage_mv, pr_data->request_current_ma);
	if (rc == 0) {
		pr_data->pdo_err = false;
		pr_data->pdo_err_low = max(pr_data->pdo_err_low, pr_data->request_current_ma);
		chg_err("pdo set success high:%d low:%d req:%d\n", pr_data->pdo_err_high, pr_data->pdo_err_low,
			pr_data->request_current_ma);
	} else {
		pr_data->pdo_err = true;
		pr_data->pdo_err_high = min(pr_data->pdo_err_high, pr_data->request_current_ma);
		chg_err("pdo set fail high:%d low:%d req:%d\n", pr_data->pdo_err_high, pr_data->pdo_err_low,
			pr_data->request_current_ma);
	}

	return delay_ms;
}

static const struct ufcs_pr_handler ufcs_pr_handles[] = {
	[UFCS_PR_IDLE] = {
		.name = "IDLE",
		.handle = ufcs_pr_handle_idle
	},
	[UFCS_PR_RISING] = {
		.name = "RISING",
		.handle = ufcs_pr_handle_rising
	},
	[UFCS_PR_FALLING] = {
		.name = "FALLING",
		.handle = ufcs_pr_handle_falling
	},
	[UFCS_PR_CC] = {
		.name = "CC",
		.handle = ufcs_pr_handle_cc
	},
	[UFCS_PR_CV] = {
		.name = "CV",
		.handle = ufcs_pr_handle_cv
	},
	[UFCS_PR_FULL] = {
		.name = "FULL",
		.handle = ufcs_pr_handle_full,
	},
	[UFCS_PR_PDO_ERR] = {
		.name = "PDO_ERR",
		.handle = ufcs_pr_handle_pdo_err,
	},
};

static const char *ufcs_pr_state_name(enum ufcs_pr_state state)
{
	if (state < 0 || state >= UFCS_PR_MAX)
		return "INVALID";
	return ufcs_pr_handles[state].name;
}

int ufcs_pr_run(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	enum ufcs_pr_state curr_state = pr_data->curr_state;
	int delay_ms = 200;

	chg_info("state:%s req_vbus:%d req_ibus:%d vbus:%d pmic_vbus:%d ibus:%d vbat:%d ibat:%d target_ibus:%d pmic_vbat:%d "
		"vbus_max:%d req_max_vbus:%d vfull:%d vfull_comp:%d vbat_target:%d vbat_target_by_ibus:%d "
		"curve_ibat:%d curve_ibus:%d next_curve_ibus:%d bcc_target:%d cv_curve:%d last_curve:%d iterm:%d "
		"adjust:%d adjust_cnt:%d adjust_time:%ums adjust_diff_time:%ums adjust_vbus:%d adajust_req:%d\n",
		ufcs_pr_state_name(curr_state),
		chip->vol_set_mv,
		chip->curr_set_ma,
		pr_data->adapter_vbus_mv,
		pr_data->pmic_vbus_mv,
		pr_data->adapter_ibus_ma,
		pr_data->vbat_mv,
		pr_data->ibat_ma,
		pr_data->target_current_ma,
		pr_data->pmic_vbat_mv,
		pr_data->vbus_max_mv,
		pr_data->req_vbus_max_mv,
		pr_data->vbat_full_mv,
		pr_data->vbat_full_comp_mv,
		pr_data->vbat_target_mv,
		pr_data->vbat_target_mv_by_ibus,
		pr_data->ibat_target_ma,
		pr_data->ibus_target_ma,
		pr_data->next_ibus_target_ma,
		pr_data->bcc_target_ma,
		pr_data->cv_curve,
		pr_data->last_curve,
		pr_data->iterm_ma,
		pr_data->adjust,
		pr_data->adjust_rising_cnt,
		jiffies_to_msecs(pr_data->adjust_jiffies - INITIAL_JIFFIES),
		jiffies_to_msecs(jiffies - pr_data->adjust_jiffies),
		pr_data->adjust_rising_adapter_vbus_mv,
		pr_data->adjust_rising_req_vbus_mv);

	if (ufcs_pr_handles[curr_state].handle)
		delay_ms = ufcs_pr_handles[curr_state].handle(chip);
	else
		chg_err("No handler for pr state: %s\n", ufcs_pr_state_name(curr_state));
	if (pr_data->pdo_err)
		ufcs_pr_set_state(chip, UFCS_PR_PDO_ERR);

	return delay_ms;
}

static int pr_get_vbat_target_by_ibus(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	int i = 0;
	int index = -1;

	for (i = 0; i < pr_data->curve.num; i++) {
		if (pr_data->adapter_ibus_ma >= pr_data->curve.data[i].target_ibus) {
			index = i;
			break;
		}
	}

	if (index < 0)
		return pr_data->curve.data[pr_data->curve.num - 1].target_vbat;

	if (index == 0)
		return pr_data->curve.data[0].target_vbat;

	return pr_data->curve.data[index - 1].target_vbat;
}

static void oplus_ufcs_pr_set_ratio(struct oplus_ufcs *chip, int batt_num)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;

	switch (chip->cp_work_mode) {
	case CP_WORK_MODE_4_TO_1:
		pr_data->ibus_to_ibat_ratio = 4;
		pr_data->vbus_to_vbat_ratio = 4 * batt_num;
		break;
	case CP_WORK_MODE_3_TO_1:
		pr_data->ibus_to_ibat_ratio = 3;
		pr_data->vbus_to_vbat_ratio = 3 * batt_num;
		break;
	case CP_WORK_MODE_2_TO_1:
		pr_data->ibus_to_ibat_ratio = 2;
		pr_data->vbus_to_vbat_ratio = 2 * batt_num;
		break;
	case CP_WORK_MODE_BYPASS:
	default:
		pr_data->ibus_to_ibat_ratio = 1;
		pr_data->vbus_to_vbat_ratio = 1 * batt_num;
		break;
	}
}

static int oplus_ufcs_pr_update_data(struct oplus_ufcs *chip)
{
	struct ufcs_pr_data *pr_data = &chip->pr_data;
	struct ufcs_pr_config *pr_config = &chip->pr_config;
	union mms_msg_data data = { 0 };
	struct puc_strategy_ret_data puc_data;
	int rc = 0;
	int batt_num = oplus_gauge_get_batt_num();

	rc = oplus_ufcs_get_src_info(chip, &chip->src_info);
	if (rc < 0) {
		chg_err("ufcs get src info error\n");
		goto err;
	}

	oplus_ufcs_pr_set_ratio(chip, batt_num);

	pr_data->adapter_vbus_mv = UFCS_SOURCE_INFO_VOL(chip->src_info);
	pr_data->adapter_ibus_ma = UFCS_SOURCE_INFO_CURR(chip->src_info);

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_CURR, &data, true);
	pr_data->ibat_ma = -data.intval;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_MAX, &data, true);
	pr_data->vbat_mv = data.intval;

	oplus_mms_get_item_data(chip->gauge_topic, GAUGE_ITEM_VOL_FCL, &data, true);
	pr_data->pmic_vbat_mv = data.intval;

	rc = oplus_chg_strategy_get_data(chip->strategy, &puc_data);
	if (rc < 0) {
		chg_err("can't get strategy data, rc=%d\n", rc);
		goto err;
	}

	pr_data->cv_curve = puc_data.support_cv;
	pr_data->last_curve = puc_data.last_gear;
	pr_data->vbat_target_mv = puc_data.target_vbat;
	if (pr_data->ibat_target_ma != puc_data.target_ibus * pr_data->ibus_to_ibat_ratio || pr_data->bcc_target_ma <= 0)
		pr_data->bcc_target_ma = puc_data.target_ibus * pr_data->ibus_to_ibat_ratio;
	pr_data->ibat_target_ma = puc_data.target_ibus * pr_data->ibus_to_ibat_ratio;
	pr_data->ibus_target_ma = puc_data.target_ibus;
	pr_data->iterm_ma = puc_data.iterm;

	if (puc_data.index < pr_data->curve.num - 1)
		pr_data->next_ibus_target_ma = pr_data->curve.data[puc_data.index + 1].target_ibus;
	else
		pr_data->next_ibus_target_ma = pr_data->curve.data[pr_data->curve.num - 1].target_ibus;
	pr_data->vbat_target_mv_by_ibus = pr_get_vbat_target_by_ibus(chip);

	pr_data->vbus_max_mv = pr_data->vbat_full_mv * pr_data->vbus_to_vbat_ratio +
		pr_config->pr_r_input_mohm * pr_data->adapter_ibus_ma / 1000;
	pr_data->vbat_full_comp_mv = pr_data->vbat_full_mv + pr_config->pr_r_btb_mohm * pr_data->ibat_ma / 1000 +
		UFCS_PR_FULL_COMP_VOL_MV;
	pr_data->pmic_vbus_mv = oplus_wired_get_vbus();
err:
	return rc;
}

#endif /* __OPLUS_CHG_UFCS_PR_H__ */
