// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021-2021 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[PD_MANAGER]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/extcon-provider.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/usb/typec.h>
#include <linux/version.h>
#include <linux/power_supply.h>

#include <tcpci.h>
#include <tcpm.h>
#include <tcpci_typec.h>
#if IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
#include <pd_policy_engine.h>
#endif

#include <oplus_chg_module.h>
#include <oplus_chg_voter.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_pps.h>
#include <oplus_chg_cpa.h>
#include <oplus_chg_comm.h>
#include <oplus_reverse_chg.h>

#define PROBE_CNT_MAX			100
/* 10ms * 100 = 1000ms = 1s */
#define USB_TYPE_POLLING_INTERVAL	10
#define USB_TYPE_POLLING_CNT_MAX	100
#define BC12_TIMEOUT_MS			1500
#define MICRO_5V 			5000
#define DELAY_TIME			100
#define MAX_PD_INPUT_CURRENT		2000
#define VBUS_5V			5000
#define VBUS_9V			9000

enum dr {
	DR_IDLE,
	DR_DEVICE,
	DR_HOST,
	DR_DEVICE_TO_HOST,
	DR_HOST_TO_DEVICE,
	DR_MAX,
};

static char *dr_names[DR_MAX] = {
	"Idle", "Device", "Host", "Device to Host", "Host to Device",
};

struct pd_manager_chip {
	struct device *dev;
	struct extcon_dev *extcon;
	struct oplus_chg_ic_dev *ic_dev;
	struct delayed_work usb_dwork;
	struct tcpc_device *tcpc;
	struct notifier_block pd_nb;
	enum dr usb_dr;
	int usb_type_polling_cnt;
	int sink_mv_new;
	int sink_ma_new;
	int sink_mv_old;
	int sink_ma_old;

	struct typec_capability typec_caps;
	struct typec_port *typec_port;
	struct typec_partner *partner;
	struct typec_partner_desc partner_desc;
	struct usb_pd_identity partner_identity;

	struct delayed_work bc12_wait_work;
	struct delayed_work vconn_wait_work;
	struct delayed_work svid_check_work;
	struct delayed_work tcpc_complete_work;

	struct oplus_mms *wired_topic;
	struct mms_subscribe *wired_subs;
	int chg_type;
	int pd_type;
	int voltage_max_mv;
	int voltage_min_mv;
	int current_max_ma;
	bool otg_enable;
	enum typec_data_role data_role;

	bool bc12_completed;
	bool bc12_ready;
	bool start_peripheral;
	bool first_check;
	bool pd_svooc;
	bool svid_completed;
	bool cpa_support;
	bool enable_tcpc_irq;
	struct power_supply *batt_psy;

	/* reverse charger	*/
	pd_msg_data pdo[PPS_PDO_MAX];
	bool oplus_pd_sdp_svid;
	bool reverse_enable;
	bool high_reverse_enable;
	uint32_t reverse_chg_svid;
	bool source_plug_out;
	int cap_nr;
	int pre_source_vbus;
	int power_role;
	int pre_pdo_voltage;
	int pre_pdo_current;
	enum reverse_chg_msg_type msg_type;
	struct oplus_chg_ic_dev *reverse_chg_ic_dev;
	struct delayed_work reverse_chg_svid_check_work;
	struct delayed_work sink_request_check_work;
	struct delayed_work source_pdo_check_work;
	struct delayed_work sourcecap_done_work;
};

static const unsigned int rpm_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static void tcpc_set_high_reverse_enable(struct oplus_chg_ic_dev *ic_dev, bool en);
static void tcpc_set_power_role(struct pd_manager_chip *chip, int role);

static bool oplus_pd_without_usb(struct pd_manager_chip *chip)
{
	if (!tcpm_inquire_pd_connected(chip->tcpc))
		return true;
	return (tcpm_inquire_dpm_flags(chip->tcpc) &
		DPM_FLAGS_PARTNER_USB_COMM) ?
		       false : true;
}

static void tcpc_set_pd_type(struct pd_manager_chip *chip, int type)
{
	if (type != OPLUS_CHG_USB_TYPE_UNKNOWN && !oplus_pd_without_usb(chip))
		type = OPLUS_CHG_USB_TYPE_PD_SDP;

	if (chip->pd_type == type)
		return;
	chg_info("pd_type = %d\n", type);
	chip->pd_type = type;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
}

static void tcpc_set_otg_enbale(struct pd_manager_chip *chip, bool en)
{
	if (chip->otg_enable == en)
		return;
	chg_info("otg_enable = %d\n", en);
	chip->otg_enable = en;
	chip->reverse_enable = en;
	if (!en)
		chip->reverse_chg_svid = 0;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_OTG_ENABLE);
	if (chip->reverse_chg_ic_dev)
		oplus_chg_ic_virq_trigger(chip->reverse_chg_ic_dev, OPLUS_IC_VIRQ_REVERSE_ENABLE);
	else
		chg_err("reverse_chg_ic_dev is NULL\n");
}

#define WAIT_SOURCECAP_DOWN 1000
static void oplus_pd_manager_handle_source_vbus(struct pd_manager_chip *chip,
	struct tcp_notify *noti)
{
	int vbus_mv;

	if (!chip || !chip->ic_dev) {
		chg_err("chip/ic_dev is NULL");
		return;
	}
	if (!noti) {
		chg_err("noti is NULL");
		return;
	}
	vbus_mv = noti->vbus_state.mv;
	if (vbus_mv == VBUS_5V) {
		tcpc_set_otg_enbale(chip, true);
		tcpc_set_high_reverse_enable(chip->ic_dev, false);
		tcpc_set_power_role(chip, POWER_ROLE_SOURCE);
	} else if (vbus_mv == VBUS_9V) {
		tcpc_set_high_reverse_enable(chip->ic_dev, true);
		tcpc_set_power_role(chip, POWER_ROLE_SOURCE);
	} else {
		tcpc_set_high_reverse_enable(chip->ic_dev, false);
		tcpc_set_otg_enbale(chip, false);
	}

	if (chip->pre_source_vbus == 0 && vbus_mv == VBUS_5V)
		schedule_delayed_work(&chip->reverse_chg_svid_check_work,
			msecs_to_jiffies(WAIT_SOURCECAP_DOWN));
	chip->pre_source_vbus = vbus_mv;
}

#if IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
static void oplus_pd_manager_handle_sourcecap_done(struct pd_manager_chip *chip,
	struct tcp_notify *noti)
{
	int i;
	int max_nr;

	if (!chip) {
		chg_err("chip is NULL");
		return;
	}
	if (!noti || !noti->caps_msg.caps) {
		chg_err("caps is NULL");
		return;
	}
	chg_info("PD_SOURCECAP_DONE\n");
	max_nr = min(PPS_PDO_MAX, (int)(sizeof(noti->caps_msg.caps->pdos) / sizeof(noti->caps_msg.caps->pdos[0])));
	chip->cap_nr = min(noti->caps_msg.caps->nr, max_nr);
	for (i = 0; i < chip->cap_nr; i++) {
		chip->pdo[i].pdo_data = (u32)noti->caps_msg.caps->pdos[i];
		chg_info("SourceCap[%d]: %08X\n", i + 1, chip->pdo[i].pdo_data);
	}
	if (oplus_chg_get_common_charge_icl_support_flags())
		schedule_delayed_work(&chip->sourcecap_done_work, 0);
}
#endif

static void tcpc_set_high_reverse_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct pd_manager_chip *chip;
	if (ic_dev  == NULL) {
		chg_err("ic_dev is NULL");
		return;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return;
	}
	if (chip->high_reverse_enable == en)
		return;
	chg_info("high_reverse_enable = %d\n", en);
	chip->high_reverse_enable = en;
	if (chip->reverse_chg_ic_dev != NULL) {
		oplus_chg_ic_virq_trigger(chip->reverse_chg_ic_dev, OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE);
	} else {
		chg_err("reverse_chg_ic_dev is NULL, can't trigger virq\n");
	}
}

static void tcpc_sink_request_check_work(struct work_struct *work)
{
	struct pd_manager_chip *chip = container_of(work,
		struct pd_manager_chip, sink_request_check_work.work);
	if (!chip)
		return;
	chip->msg_type = REVERSE_CHG_MSG_TYPE_SINK_REQ_PDO;
	if (chip->reverse_chg_ic_dev)
		oplus_chg_ic_virq_trigger(chip->reverse_chg_ic_dev, OPLUS_IC_VIRQ_SINK_REQ_MSG);
}

#define SRC_PE_READY_DELAY 100
#define SRC_WAIT_PE_RDY_TH 10
#define NORMAL_VBUS 5000
#define NORMAL_IBUS 1500
/*USB 2.0 default current threshold */
#define SINK_SUSPEND_CURRENT 100

#if IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
static void mtk_chg_wait_pe_src_ready(struct tcpc_device *tcpc)
{
	int i;

	if (tcpc == NULL) {
		chg_err("tcpc is null");
		return;
	}

	if (tcpc->pd_port.pe_pd_state == PE_SRC_READY)
		return;

	for (i = 0; i <= SRC_WAIT_PE_RDY_TH; i++) {
		msleep(SRC_PE_READY_DELAY);
		chg_info("!!!pe_pd_state : %d, src_pe_not_rdy_cnt: %d\n",
			 tcpc->pd_port.pe_pd_state, i);
		if (tcpc->pd_port.pe_pd_state == PE_SRC_READY)
			break;
	}
}

static int tcpc_check_reverse_enable(struct oplus_chg_ic_dev *ic_dev, struct pd_manager_chip **chip)
{
	*chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!*chip) {
		chg_err("chip is null\n");
		return -ENODEV;
	}

	if (!(*chip)->reverse_enable) {
		chg_err("reverse_enable = 0, return");
		return -EFAULT;
	}
	return 0;
}

static int tcpc_get_source_cap_flags(struct tcpc_device *tcpc, uint32_t *flags)
{
	struct tcpm_power_cap user_cap;
	int rc;

	rc = tcpm_inquire_pd_local_source_cap(tcpc, &user_cap);
	if (rc < 0) {
		chg_err("tcpm_inquire_pd_local_source_cap error: rc=%d\n", rc);
		return rc;
	}

	if (user_cap.cnt <= 0) {
		chg_err("user_cap.cnt is %d, invalid\n", user_cap.cnt);
		return -EINVAL;
	}

	*flags = user_cap.pdos[0] & (0x1FF << 23);
	return 0;
}

static struct tcpm_power_cap tcpc_build_cap(int pdo0_volt, int pdo0_curr,
					    int pdo_voltage, int pdo_current, uint32_t flags)
{
	struct tcpm_power_cap cap;

	if (pdo_voltage == NORMAL_VBUS) {
		cap.pdos[0] = PDO_FIXED(pdo_voltage, pdo_current, flags);
		cap.cnt = 1;
	} else {
		cap.pdos[0] = PDO_FIXED(pdo0_volt, pdo0_curr, flags);
		cap.pdos[1] = PDO_FIXED(pdo_voltage, pdo_current, 0);
		cap.cnt = 2;
	}
	return cap;
}

static int tcpc_set_source_cap_internal(struct tcpc_device *tcpc, struct tcpm_power_cap *cap)
{
	int rc;

	if (!tcpc || !cap)
		return -EINVAL;
	mtk_chg_wait_pe_src_ready(tcpc);

	chg_info("!!!REVERSE_CHANGE: update sourcecap cnt[%d] pdo: 0x%08X, 0x%08X\n",
		cap->cnt, cap->pdos[0], cap->pdos[1]);
	rc = tcpm_set_pd_local_source_cap(tcpc, cap);
	if (rc < 0) {
		chg_err("tcpm_set_pd_local_source_cap error: rc=%d\n", rc);
		return rc;
	}

	rc = tcpm_dpm_pd_source_cap(tcpc, NULL);
	if (rc < 0) {
		chg_err("tcpm_dpm_pd_source_cap error: rc=%d\n", rc);
		return rc;
	}
	return 0;
}

static int tcpc_set_reverse_boost_pdo(struct oplus_chg_ic_dev *ic_dev, int pdo0_volt, int pdo0_curr, int pdo_voltage, int pdo_current)
{
	struct tcpc_device *tcpc = tcpc_dev_get_by_name("type_c_port0");
	struct tcpm_power_cap cap = { 1, {0x26019096} };
	struct pd_manager_chip *chip;
	uint32_t flags;
	int rc;

	if (tcpc == NULL) {
		chg_info("tcpc_dev is null return\n");
		return -EINVAL;
	}

	rc = tcpc_check_reverse_enable(ic_dev, &chip);
	if (rc)
		return rc;

	rc = tcpc_get_source_cap_flags(tcpc, &flags);
	if (rc)
		return rc;

	cap = tcpc_build_cap(pdo0_volt, pdo0_curr, pdo_voltage, pdo_current, flags);
	rc = tcpc_set_source_cap_internal(tcpc, &cap);
	if (rc)
		return rc;

	chip->pre_pdo_voltage = pdo_voltage;
	chip->pre_pdo_current = pdo_current;

	schedule_delayed_work(&chip->sink_request_check_work,
			msecs_to_jiffies(WAIT_SOURCECAP_DOWN));

	return 0;
}

static int tcpc_set_rvs_high_mode_en(struct oplus_chg_ic_dev *ic_dev, int enable)
{
	struct pd_manager_chip *chip;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}
	if (enable) {
		rc = tcpc_set_reverse_boost_pdo(ic_dev, NORMAL_VBUS, NORMAL_IBUS, VBUS_9V, NORMAL_IBUS);
		if (rc) {
			chg_err("Failed to set reverse boost pdo, rc=%d", rc);
			return rc;
		}
	} else {
		tcpc_set_high_reverse_enable(ic_dev, false);
		tcpc_set_otg_enbale(chip, false);
		chg_err("close high reverse and otg");
	}
	return 0;
}
#endif

static void tcpc_set_data_role(struct pd_manager_chip *chip,
			       enum typec_data_role role)
{
	typec_set_data_role(chip->typec_port, role);
	if (chip->data_role == role)
		return;
	chg_info("data_role = %d\n", role);
	chip->data_role = role;
	oplus_chg_ic_virq_trigger(chip->ic_dev,
				  OPLUS_IC_VIRQ_DATA_ROLE_CHANGED);
}

static void tcpc_set_power_role(struct pd_manager_chip *chip, int role)
{
	if (chip->power_role == role)
		return;
	chg_info("power_role = %d\n", role);
	chip->power_role = role;
	oplus_chg_ic_virq_trigger(chip->ic_dev,
				  OPLUS_IC_VIRQ_POWER_ROLE_STATUS);
}

static void tcpc_set_voltage_max_and_min(struct pd_manager_chip *chip, int max, int min)
{
	if (chip->voltage_max_mv == max && chip->voltage_min_mv == min)
		return;
	chg_info("voltage_max_mv = %d, voltage_min_mv = %d\n", max, min);
	chip->voltage_max_mv = max;
	chip->voltage_min_mv = min;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_VOLTAGE_CHANGED);
}

static void tcpc_set_current_max(struct pd_manager_chip *chip, int max)
{
	struct votable *icl_votable;
	int icl_tmp_ma = 0;

	icl_votable = find_votable("WIRED_ICL");
	if (!icl_votable)
		chg_err("WIRED_ICL votable not found\n");
	else
		icl_tmp_ma = get_client_vote_locked(icl_votable, MAX_VOTER);

	if (chip->current_max_ma == max && icl_tmp_ma <= max && icl_tmp_ma > 0) {
		chg_info("current_max_ma = %d\n", icl_tmp_ma);
		return;
	}
	chg_info("current_max_ma = %d\n", max);
	chip->current_max_ma = max;
	if (oplus_chg_get_common_charge_icl_support_flags()) {
		if (max == 0)
			return;
	}
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CURRENT_CHANGED);
}

#define OPLUS_SVID 0x22d9
#define VCONN_TIMEOUT_MS 1000
static void tcpc_get_adapter_svid(struct pd_manager_chip *chip)
{
	int i = 0;
	uint32_t vdos[VDO_MAX_NR] = {0};
	struct tcpc_device *tcpc_dev = chip->tcpc;
	struct tcpm_svid_list svid_list = { 0, { 0 } };

	if (tcpc_dev == NULL) {
		chg_err("tcpc_dev is null return\n");
		return;
	}

	tcpm_inquire_pd_partner_svids(tcpc_dev, &svid_list);
	for (i = 0; i < svid_list.cnt; i++) {
		chg_info("svid[%d] = 0x%04x\n", i, svid_list.svids[i]);
		if (svid_list.svids[i] == OPLUS_SVID) {
			chip->pd_svooc = true;
			chg_info("match svid and this is oplus adapter\n");
			break;
		}
	}

	tcpm_inquire_pd_partner_inform(tcpc_dev, vdos);
	if ((vdos[0] & 0xFFFF) == OPLUS_SVID) {
		chip->pd_svooc = true;
		chg_info("match svid and this is oplus adapter 11\n");
	}

	if (chip->pd_svooc)
		schedule_delayed_work(&chip->vconn_wait_work,
				      msecs_to_jiffies(VCONN_TIMEOUT_MS));
	else
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_SVID);
}

#define TCPM_SUCCESS 0
#define DISCOVER_SVID_MAX_RETRIES 3
#define DISCOVER_SVID_RETRY_DELAY 50
#define DISCOVER_SVID_INTERNAL_CMD_DELAY 10
static int oplus_get_pd_partner_svids(struct pd_manager_chip *chip, struct tcpc_device *tcpc_dev)
{
	int i = 0;
	int ret = 0;
	struct tcpm_svid_list svid_list = {0, {0}};
	union mms_msg_data data = { 0 };

	if (!chip->wired_topic) {
		chg_err("wired_topic is NULL");
		return -EINVAL;
	}
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE,
				&data, true);
	chip->chg_type = data.intval;
	oplus_wired_get_chg_type_str(chip->chg_type);
	ret = tcpm_inquire_pd_partner_svids(tcpc_dev, &svid_list);
	if (ret == TCPM_SUCCESS) {
		for (i = 0; i < svid_list.cnt; i++) {
			chg_info("svid[%d] = 0x%x\n", i, svid_list.svids[i]);
			if (svid_list.svids[i] == OPLUS_SVID) {
				if (chip->chg_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
					chip->reverse_chg_svid = OPLUS_SVID;
					chip->oplus_pd_sdp_svid = true;
					chg_info("get svid is oplus device\n");
				} else {
					chip->pd_svooc = true;
					chg_info("get match svid and this is oplus adapter\n");
				}
				break;
			}
		}
	}

	return ret;
}

static int oplus_get_pd_partner_inform(struct pd_manager_chip *chip, struct tcpc_device *tcpc_dev)
{
	int ret = 0;
	uint32_t data[VDO_MAX_NR] = {0};
	union mms_msg_data msg_data = { 0 };

	if (!chip->wired_topic) {
		chg_err("wired topic is NULL");
		return -EINVAL;
	}
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE,
				&msg_data, true);
	chip->chg_type = msg_data.intval;
	oplus_wired_get_chg_type_str(chip->chg_type);
	ret = tcpm_inquire_pd_partner_inform(tcpc_dev, data);
	if (ret == TCPM_SUCCESS && (data[0] & 0xFFFF) == OPLUS_SVID) {
		if (chip->chg_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
			chip->reverse_chg_svid = OPLUS_SVID;
			chip->oplus_pd_sdp_svid = true;
			chg_info("get svid is oplus device\n");
		} else {
			chip->pd_svooc = true;
			chg_info("get match svid and this is oplus adapter\n");
		}
	} else {
		chg_err("get the partner infom failed, ret = %d\n", ret);
	}

	return ret;
}

static int oplus_discover_id(struct tcpc_device *tcpc_dev)
{
	int ret = 0;

	ret = tcpm_dpm_vdm_discover_id(tcpc_dev, NULL);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
	if (ret == TCP_DPM_RET_NOT_SUPPORT ||
	    ret == TCP_DPM_RET_DENIED_WRONG_ROLE ||
	    ret == TCPM_ERROR_PUT_EVENT) {
#else
	if (ret == TCP_DPM_RET_NOT_SUPPORT ||
	    ret == TCPM_ERROR_PUT_EVENT) {
#endif
		chg_err(" failed, ret = %d.\n", ret);
		return -EFAULT;
	} else if (ret != TCPM_SUCCESS) {
		chg_err("failed to discover id, ret = %d\n", ret);
	}

	return ret;
}

static int oplus_discover_svid(struct tcpc_device *tcpc_dev)
{
	int ret = 0;

	/* Note: the WC065A11JCH not support the cmd of discover_svid. */
	ret = tcpm_dpm_vdm_discover_svid(tcpc_dev, NULL);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)) && !IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
	if (ret == TCP_DPM_RET_DENIED_WRONG_ROLE ||
	    ret == TCPM_ERROR_PUT_EVENT) {
#else
	if (ret == TCPM_ERROR_PUT_EVENT) {
#endif
		chg_err(" failed, ret = %d.\n", ret);
		return -EFAULT;
	} else if (ret != TCPM_SUCCESS) {
		chg_err("failed to discover svid,ret = %d\n", ret);
	}

	return ret;
}

static int oplus_get_adapter_svid(struct pd_manager_chip *chip)
{
	int ret = 0;
	int discover_svid_ret;
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	int disc_svid_retries = 0;

	if (tcpc_dev == NULL) {
		chg_err("tcpc_dev is null return\n");
		return -EINVAL;
	}

	if (chip->pd_svooc)
		goto trigger_irq;

	do {
		disc_svid_retries++;
		if (disc_svid_retries > DISCOVER_SVID_MAX_RETRIES)
			break;

		ret = oplus_discover_id(tcpc_dev);
		if (ret == -EFAULT) {
			chg_err("vdm_discover_id failed, ret = %d, retries: %d, not try again.\n",
				ret, disc_svid_retries);
			break;
		} else if (ret != TCPM_SUCCESS) {
			chg_err("failed to discover id,  disc_svid_retries: %d, ret = %d\n",
				disc_svid_retries, ret);
			msleep(DISCOVER_SVID_RETRY_DELAY);
			continue;
		}

		msleep(DISCOVER_SVID_INTERNAL_CMD_DELAY);
		discover_svid_ret = oplus_discover_svid(tcpc_dev);
		if (discover_svid_ret == -EFAULT) {
			chg_err("get the svid failed, ret = %d, retries: %d, not try again.\n",
				discover_svid_ret, disc_svid_retries);
			break;
		} else if (ret != TCPM_SUCCESS) {
			chg_err("Failed to discover svid. ret %d retries: %d\n",
				discover_svid_ret, disc_svid_retries);
		}
		msleep(DISCOVER_SVID_INTERNAL_CMD_DELAY);

		ret = oplus_get_pd_partner_svids(chip, tcpc_dev);
		if (ret == TCPM_SUCCESS) {
			goto trigger_irq;
		} else {
			chg_err("get the pd partner svids failed, ret = %d, retries = %d\n",
				ret, disc_svid_retries);

			/* Note: the WC065A11JCH maybe not support the get_pd_partner_svids */
			if (discover_svid_ret == TCP_DPM_RET_NOT_SUPPORT)
				chg_info("not support to get_pd_partner_svids, ret = %d, discover_svid_ret = %d\n",
					 ret, discover_svid_ret);
			msleep(DISCOVER_SVID_RETRY_DELAY);
		}

		/* retry to get the SVID by PD partner inform. */
		msleep(DISCOVER_SVID_INTERNAL_CMD_DELAY);
		ret = oplus_get_pd_partner_inform(chip, tcpc_dev);
		if (ret == TCPM_SUCCESS) {
			goto trigger_irq;
		} else {
			chg_err("get the partner infom failed, ret = %d, retries = %d.\n",
				ret, disc_svid_retries);

			/* discover svid return not support and get pd_partner_inform failed, not try again.*/
			if (discover_svid_ret == TCP_DPM_RET_NOT_SUPPORT) {
				chg_info("not support to get the svid, ret = %d, discover_svid_ret = %d\n",
					  ret, discover_svid_ret);
				goto trigger_irq;
			}
			msleep(DISCOVER_SVID_RETRY_DELAY);
		}
	} while (ret != TCPM_SUCCESS && disc_svid_retries < DISCOVER_SVID_MAX_RETRIES);

trigger_irq:
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_SVID);

	return 0;
}

static void oplus_svid_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pd_manager_chip *chip = container_of(dwork, struct pd_manager_chip, svid_check_work);
	oplus_get_adapter_svid(chip);
}

static int extcon_init(struct pd_manager_chip *chip)
{
	int ret = 0;

	/*
	 * associate extcon with the dev as it could have a DT
	 * node which will be useful for extcon_get_edev_by_phandle()
	 */
	chip->extcon = devm_extcon_dev_allocate(chip->dev, rpm_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		ret = PTR_ERR(chip->extcon);
		chg_err("extcon dev alloc fail(%d)\n", ret);
		goto out;
	}

	ret = devm_extcon_dev_register(chip->dev, chip->extcon);
	if (ret) {
		chg_err("extcon dev reg fail(%d)\n", ret);
		goto out;
	}

	/* Support reporting polarity and speed via properties */
	extcon_set_property_capability(chip->extcon, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB,
				       EXTCON_PROP_USB_SS);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	extcon_set_property_capability(chip->extcon, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_MED_HIGH_CURRENT);
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)) */
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_SS);
out:
	return ret;
}

static inline void stop_usb_host(struct pd_manager_chip *chip)
{
	extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, false);
}

static inline void start_usb_host(struct pd_manager_chip *chip)
{
	union extcon_property_value val = {.intval = 0};

	val.intval = tcpm_inquire_cc_polarity(chip->tcpc);
	extcon_set_property(chip->extcon, EXTCON_USB_HOST,
			    EXTCON_PROP_USB_TYPEC_POLARITY, val);

	val.intval = 1;
	extcon_set_property(chip->extcon, EXTCON_USB_HOST,
			    EXTCON_PROP_USB_SS, val);

	extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, true);
}

static inline void stop_usb_peripheral(struct pd_manager_chip *chip)
{
	chip->start_peripheral = false;
	extcon_set_state_sync(chip->extcon, EXTCON_USB, false);
}

static inline void start_usb_peripheral(struct pd_manager_chip *chip)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	int rp = 0;
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)) */
	union extcon_property_value val = {.intval = 0};

	if (!chip->bc12_completed) {
		chg_info("wait bc1.2 completed\n");
		chip->start_peripheral = true;
		schedule_delayed_work(&chip->bc12_wait_work,
				      msecs_to_jiffies(BC12_TIMEOUT_MS));
		return;
	}
	chg_info("start usb peripheral\n");
	chip->start_peripheral = false;

	val.intval = tcpm_inquire_cc_polarity(chip->tcpc);
	extcon_set_property(chip->extcon, EXTCON_USB,
			    EXTCON_PROP_USB_TYPEC_POLARITY, val);

	val.intval = 1;
	extcon_set_property(chip->extcon, EXTCON_USB, EXTCON_PROP_USB_SS, val);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	rp = tcpm_inquire_typec_remote_rp_curr(chip->tcpc);
	val.intval = rp > 500 ? 1 : 0;
	extcon_set_property(chip->extcon, EXTCON_USB,
			    EXTCON_PROP_USB_TYPEC_MED_HIGH_CURRENT, val);
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)) */
	extcon_set_state_sync(chip->extcon, EXTCON_USB, true);
}

static void tcpc_bc12_wait_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pd_manager_chip *chip =
		container_of(dwork, struct pd_manager_chip, bc12_wait_work);

	chip->bc12_completed = true;

	if (chip->start_peripheral)
		start_usb_peripheral(chip);
}

static void tcpc_vconn_wait_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pd_manager_chip *chip =
		container_of(dwork, struct pd_manager_chip, vconn_wait_work);

	if (chip->pd_svooc)
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_SVID);
}

static void usb_dwork_handler(struct work_struct *work)
{
	int ret = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct pd_manager_chip *chip =
		container_of(dwork, struct pd_manager_chip, usb_dwork);
	enum dr usb_dr = chip->usb_dr;
	union mms_msg_data data = { 0 };

	if (usb_dr < DR_IDLE || usb_dr >= DR_MAX) {
		chg_err("invalid usb_dr = %d\n", usb_dr);
		return;
	}

	chg_debug("dr_names: %s\n", dr_names[usb_dr]);

	switch (usb_dr) {
	case DR_IDLE:
	case DR_MAX:
		stop_usb_peripheral(chip);
		stop_usb_host(chip);
		break;
	case DR_DEVICE:
		oplus_mms_get_item_data(chip->wired_topic,
					WIRED_ITEM_CHG_TYPE, &data,
					true);
		chip->chg_type = data.intval;
		chg_info("polling_cnt=%d, ret=%d type=%d, bc12_ready=%d\n",
			 ++chip->usb_type_polling_cnt, ret, chip->chg_type,
			 chip->bc12_ready);
		if (!chip->bc12_ready || chip->chg_type == OPLUS_CHG_USB_TYPE_UNKNOWN) {
			if (chip->usb_type_polling_cnt <
			    USB_TYPE_POLLING_CNT_MAX)
				schedule_delayed_work(
					&chip->usb_dwork,
					msecs_to_jiffies(
						USB_TYPE_POLLING_INTERVAL));
			break;
		} else if (chip->chg_type == OPLUS_CHG_USB_TYPE_SDP ||
			   chip->chg_type == OPLUS_CHG_USB_TYPE_CDP ||
			   chip->chg_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
			chg_info("start usb peripheral\n");
			stop_usb_host(chip);
			start_usb_peripheral(chip);
		}
		break;
	case DR_HOST_TO_DEVICE:
		stop_usb_host(chip);
		start_usb_peripheral(chip);
		break;
	case DR_HOST:
	case DR_DEVICE_TO_HOST:
		stop_usb_peripheral(chip);
		start_usb_host(chip);
		break;
	}
}

static void pd_sink_set_vol_and_cur(struct pd_manager_chip *chip,
				    int mv, int ma, uint8_t type)
{
	unsigned long sel = 0;
	int max, min;

	/* Charger plug-in first time */
	if (chip->pd_type == OPLUS_CHG_USB_TYPE_UNKNOWN)
		tcpc_set_pd_type(chip, OPLUS_CHG_USB_TYPE_PD);

	if (mv < MICRO_5V)
		mv = MICRO_5V;

	switch (type) {
	case TCP_VBUS_CTRL_PD_HRESET:
	case TCP_VBUS_CTRL_PD_PR_SWAP:
	case TCP_VBUS_CTRL_PD_REQUEST:
		set_bit(0, &sel);
		set_bit(1, &sel);
		max = min = mv;
		break;
	case TCP_VBUS_CTRL_PD_STANDBY_UP:
		set_bit(1, &sel);
		max = mv;
		min = chip->voltage_min_mv;
		break;
	case TCP_VBUS_CTRL_PD_STANDBY_DOWN:
		set_bit(0, &sel);
		max = chip->voltage_max_mv;
		min = mv;
		break;
	default:
		max = chip->voltage_max_mv;
		min = chip->voltage_min_mv;
		break;
	}

	tcpc_set_voltage_max_and_min(chip, max, min);
	tcpc_set_current_max(chip, ma);
}

static inline void tcpc_handle_svid_check(struct pd_manager_chip *chip)
{
	if (chip->cpa_support) {
		cancel_delayed_work_sync(&chip->svid_check_work);
		schedule_delayed_work(&chip->svid_check_work, 0);
	} else {
		tcpc_get_adapter_svid(chip);
	}
}

static void tcpc_handle_dr_swap(struct pd_manager_chip *chip, struct tcp_notify *noti)
{
	chg_info("data role swap, new role = %d\n",
		 noti->swap_state.new_role);
	if (noti->swap_state.new_role == PD_ROLE_UFP) {
		chg_info("swap data role to device\n");
		/*
		 * disable host connection,
		 * and enable device connection
		 */
		cancel_delayed_work_sync(&chip->usb_dwork);
		chip->usb_dr = DR_HOST_TO_DEVICE;
		schedule_delayed_work(&chip->usb_dwork, 0);
		tcpc_set_data_role(chip, TYPEC_DEVICE);
	} else if (noti->swap_state.new_role == PD_ROLE_DFP) {
		chg_info("swap data role to host\n");
		/*
		 * disable device connection,
		 * and enable host connection
		 */
		cancel_delayed_work_sync(&chip->usb_dwork);
		chip->usb_dr = DR_DEVICE_TO_HOST;
		schedule_delayed_work(&chip->usb_dwork, 0);
		tcpc_set_data_role(chip, TYPEC_HOST);
	}
}

static int tcpc_pd_state_change(struct pd_manager_chip *chip, struct tcp_notify *noti)
{
	uint32_t partner_vdos[VDO_MAX_NR];
	int ret = 0;

	switch (noti->pd_state.connected) {
	case PD_CONNECT_NONE:
		tcpc_set_pd_type(chip, OPLUS_CHG_USB_TYPE_UNKNOWN);
		tcpc_set_power_role(chip, POWER_ROLE_UNKNOWN);
		break;
	case PD_CONNECT_HARD_RESET:
		tcpc_set_pd_type(chip, OPLUS_CHG_USB_TYPE_UNKNOWN);
		if (chip->reverse_chg_ic_dev)
			oplus_chg_ic_virq_trigger(chip->reverse_chg_ic_dev, OPLUS_IC_VIRQ_HARD_RESET);
		break;
	case PD_CONNECT_PE_READY_SNK:
	case PD_CONNECT_PE_READY_SNK_PD30:
		tcpc_set_pd_type(chip, OPLUS_CHG_USB_TYPE_PD);
		tcpc_handle_svid_check(chip);
		typec_set_pwr_opmode(chip->typec_port, TYPEC_PWR_MODE_PD);
		if (!chip->partner)
			break;
		ret = tcpm_inquire_pd_partner_inform(chip->tcpc, partner_vdos);
		if (ret != TCPM_SUCCESS)
			break;
		chip->partner_identity.id_header = partner_vdos[0];
		chip->partner_identity.cert_stat = partner_vdos[1];
		chip->partner_identity.product = partner_vdos[2];
		typec_partner_set_identity(chip->partner);
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_POWER_CHANGED);
		break;
	case PD_CONNECT_PE_READY_SNK_APDO:
		ret = tcpm_inquire_dpm_flags(chip->tcpc);
		tcpc_set_pd_type(chip, OPLUS_CHG_USB_TYPE_PD_PPS);
		tcpc_handle_svid_check(chip);
		typec_set_pwr_opmode(chip->typec_port, TYPEC_PWR_MODE_PD);
		if (!chip->partner)
			break;
		ret = tcpm_inquire_pd_partner_inform(chip->tcpc, partner_vdos);
		if (ret != TCPM_SUCCESS)
			break;
		chip->partner_identity.id_header = partner_vdos[0];
		chip->partner_identity.cert_stat = partner_vdos[1];
		chip->partner_identity.product = partner_vdos[2];
		typec_partner_set_identity(chip->partner);
		break;
	case PD_CONNECT_PE_READY_SRC:
	case PD_CONNECT_PE_READY_SRC_PD30:
		typec_set_pwr_opmode(chip->typec_port, TYPEC_PWR_MODE_PD);
		if (!chip->partner)
			break;
		ret = tcpm_inquire_pd_partner_inform(chip->tcpc, partner_vdos);
		if (ret != TCPM_SUCCESS)
			break;
		chip->partner_identity.id_header = partner_vdos[0];
		chip->partner_identity.cert_stat = partner_vdos[1];
		chip->partner_identity.product = partner_vdos[2];
		typec_partner_set_identity(chip->partner);
		break;
	case PD_CONNECT_TYPEC_ONLY_SNK:
		/* not support svid */
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_SVID);
		break;
	}

	return ret;
}

static int pd_tcp_notifier_call(struct notifier_block *nb, unsigned long event,
				void *data)
{
	int ret = 0;
	struct tcp_notify *noti = data;
	struct pd_manager_chip *chip =
		container_of(nb, struct pd_manager_chip, pd_nb);
	uint8_t old_state = TYPEC_UNATTACHED, new_state = TYPEC_UNATTACHED;
	enum typec_pwr_opmode opmode = TYPEC_PWR_MODE_USB;
	bool hard_reset;
	bool first_boot = false;

	if (IS_ERR_OR_NULL(chip) || IS_ERR_OR_NULL(chip->ic_dev))
		return NOTIFY_OK;

	switch (event) {
	case TCP_NOTIFY_SINK_VBUS:
		chip->sink_mv_new = noti->vbus_state.mv;
		chip->sink_ma_new = noti->vbus_state.ma;
		chg_info("sink vbus %dmV %dmA type(0x%02X)\n",
			 chip->sink_mv_new, chip->sink_ma_new,
			 noti->vbus_state.type);
		tcpc_set_power_role(chip, POWER_ROLE_SINK);
		if ((chip->sink_mv_new != chip->sink_mv_old) ||
		    (chip->sink_ma_new != chip->sink_ma_old)) {
			chip->sink_mv_old = chip->sink_mv_new;
			chip->sink_ma_old = chip->sink_ma_new;
			if (chip->sink_mv_new && chip->sink_ma_new) {
				/* enable VBUS power path */
			} else {
				/* disable VBUS power path */
			}
		}

		if (noti->vbus_state.type & TCP_VBUS_CTRL_PD_DETECT)
			pd_sink_set_vol_and_cur(chip, chip->sink_mv_new,
						chip->sink_ma_new,
						noti->vbus_state.type);
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		chg_info("source vbus %dmV\n", noti->vbus_state.mv);
		oplus_pd_manager_handle_source_vbus(chip, noti);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		if (chip->first_check) {
			old_state = TYPEC_UNATTACHED;
			chip->first_check = false;
			first_boot = true;
		} else {
			old_state = noti->typec_state.old_state;
		}
		new_state = noti->typec_state.new_state;
		chg_err("old_state=%d, new_state=%d\n", old_state, new_state);
		if (old_state == TYPEC_UNATTACHED &&
		    (new_state == TYPEC_ATTACHED_SNK ||
		     new_state == TYPEC_ATTACHED_NORP_SRC ||
		     new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
		     new_state == TYPEC_ATTACHED_DBGACC_SNK)) {
			chg_info("Charger plug in, polarity = %d\n",
				 noti->typec_state.polarity);
			/*
			 * start charger type detection,
			 * and enable device connection
			 */
			if (first_boot)
				chip->bc12_completed = true;
			else
				chip->bc12_completed = false;

			chip->start_peripheral = false;
			cancel_delayed_work_sync(&chip->usb_dwork);
			chip->usb_dr = DR_DEVICE;
			chip->usb_type_polling_cnt = 0;
			schedule_delayed_work(
				&chip->usb_dwork,
				msecs_to_jiffies(USB_TYPE_POLLING_INTERVAL));
			tcpc_set_data_role(chip, TYPEC_DEVICE);
			typec_set_pwr_role(chip->typec_port, TYPEC_SINK);
			opmode = noti->typec_state.rp_level -
				 TYPEC_CC_VOLT_SNK_DFT;
			if (opmode > TYPEC_PWR_MODE_PD) {
				chg_err("Unknown typec power opmode, opmode=%d\n",
					opmode);
				opmode = TYPEC_PWR_MODE_3_0A;
			}
			typec_set_pwr_opmode(chip->typec_port, opmode);
			typec_set_vconn_role(chip->typec_port, TYPEC_SINK);
		} else if ((old_state == TYPEC_ATTACHED_SNK ||
			    old_state == TYPEC_ATTACHED_NORP_SRC ||
			    old_state == TYPEC_ATTACHED_CUSTOM_SRC ||
			    old_state == TYPEC_ATTACHED_DBGACC_SNK) &&
			   new_state == TYPEC_UNATTACHED) {
			chg_info("Charger plug out\n");
			/*
			 * report charger plug-out,
			 * and disable device connection
			 */
			chip->pd_svooc = false;
			chip->bc12_ready = false;
			chip->oplus_pd_sdp_svid = false;
			cancel_delayed_work_sync(&chip->usb_dwork);
			chip->usb_dr = DR_IDLE;
			schedule_delayed_work(&chip->usb_dwork, 0);
		} else if (old_state == TYPEC_UNATTACHED &&
			   (new_state == TYPEC_ATTACHED_SRC ||
			    new_state == TYPEC_ATTACHED_DEBUG)) {
			chip->source_plug_out = false;
			chg_info("OTG plug in, polarity = %d\n",
				 noti->typec_state.polarity);
			/* enable host connection */
			cancel_delayed_work_sync(&chip->usb_dwork);
			chip->usb_dr = DR_HOST;
			schedule_delayed_work(&chip->usb_dwork, 0);
			tcpc_set_data_role(chip, TYPEC_HOST);
			typec_set_pwr_role(chip->typec_port, TYPEC_SOURCE);
			if (!chip->batt_psy)
				chip->batt_psy = power_supply_get_by_name("battery");
			if (chip->batt_psy)
				power_supply_changed(chip->batt_psy);
			switch (noti->typec_state.local_rp_level) {
			case TYPEC_CC_RP_3_0:
				opmode = TYPEC_PWR_MODE_3_0A;
				break;
			case TYPEC_CC_RP_1_5:
				opmode = TYPEC_PWR_MODE_1_5A;
				break;
			case TYPEC_CC_RP_DFT:
			default:
				opmode = TYPEC_PWR_MODE_USB;
				break;
			}
			typec_set_pwr_opmode(chip->typec_port, opmode);
			typec_set_vconn_role(chip->typec_port, TYPEC_SOURCE);
		} else if ((old_state == TYPEC_ATTACHED_SRC ||
			    old_state == TYPEC_ATTACHED_DEBUG) &&
			   new_state == TYPEC_UNATTACHED) {
			chip->source_plug_out = true;
			chg_info("OTG plug out\n");
			/* disable host connection */
			cancel_delayed_work_sync(&chip->usb_dwork);
			chip->usb_dr = DR_IDLE;
			schedule_delayed_work(&chip->usb_dwork, 0);
		} else if (old_state == TYPEC_UNATTACHED &&
			   new_state == TYPEC_ATTACHED_AUDIO) {
			chg_info("Audio plug in\n");
			/* enable AudioAccessory connection */
		} else if (old_state == TYPEC_ATTACHED_AUDIO &&
			   new_state == TYPEC_UNATTACHED) {
			chg_info("Audio plug out\n");
			/* disable AudioAccessory connection */
		}

		if (new_state == TYPEC_UNATTACHED) {
			tcpc_set_pd_type(chip, OPLUS_CHG_USB_TYPE_UNKNOWN);

			typec_unregister_partner(chip->partner);
			chip->partner = NULL;
			if (chip->typec_caps.prefer_role == TYPEC_SOURCE) {
				tcpc_set_data_role(chip, TYPEC_HOST);
				typec_set_pwr_role(chip->typec_port,
						   TYPEC_SOURCE);
				typec_set_pwr_opmode(chip->typec_port,
						     TYPEC_PWR_MODE_USB);
				typec_set_vconn_role(chip->typec_port,
						     TYPEC_SOURCE);
			} else {
				tcpc_set_data_role(chip, TYPEC_DEVICE);
				typec_set_pwr_role(chip->typec_port,
						   TYPEC_SINK);
				typec_set_pwr_opmode(chip->typec_port,
						     TYPEC_PWR_MODE_USB);
				typec_set_vconn_role(chip->typec_port,
						     TYPEC_SINK);
			}
		} else if (!chip->partner) {
			memset(&chip->partner_identity, 0,
			       sizeof(chip->partner_identity));
			chip->partner_desc.usb_pd = false;
			switch (new_state) {
			case TYPEC_ATTACHED_AUDIO:
				chip->partner_desc.accessory =
					TYPEC_ACCESSORY_AUDIO;
				break;
			case TYPEC_ATTACHED_DEBUG:
			case TYPEC_ATTACHED_DBGACC_SNK:
			case TYPEC_ATTACHED_CUSTOM_SRC:
				chip->partner_desc.accessory =
					TYPEC_ACCESSORY_DEBUG;
				break;
			default:
				chip->partner_desc.accessory =
					TYPEC_ACCESSORY_NONE;
				break;
			}
			chip->partner = typec_register_partner(
				chip->typec_port, &chip->partner_desc);
			if (IS_ERR(chip->partner)) {
				ret = PTR_ERR(chip->partner);
				chg_debug("typec register partner fail(%d)\n",
					  ret);
			}
		}

		if (new_state == TYPEC_ATTACHED_SNK) {
			switch (noti->typec_state.rp_level) {
			/* SNK_RP_3P0 */
			case TYPEC_CC_VOLT_SNK_3_0:
				break;
			/* SNK_RP_1P5 */
			case TYPEC_CC_VOLT_SNK_1_5:
				break;
			/* SNK_RP_STD */
			case TYPEC_CC_VOLT_SNK_DFT:
			default:
				break;
			}
		} else if (new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
			   new_state == TYPEC_ATTACHED_DBGACC_SNK) {
			switch (noti->typec_state.rp_level) {
			/* DAM_3000 */
			case TYPEC_CC_VOLT_SNK_3_0:
				break;
			/* DAM_1500 */
			case TYPEC_CC_VOLT_SNK_1_5:
				break;
			/* DAM_500 */
			case TYPEC_CC_VOLT_SNK_DFT:
			default:
				break;
			}
		} else if (new_state == TYPEC_ATTACHED_NORP_SRC) {
			/* Both CCs are open */
		}
		break;
	case TCP_NOTIFY_PR_SWAP:
		chg_info("power role swap, new role = %d\n",
			 noti->swap_state.new_role);
		if (noti->swap_state.new_role == PD_ROLE_SINK) {
			chg_info("swap power role to sink\n");
			/*
			 * report charger plug-in without charger type detection
			 * to not interfering with USB2.0 communication
			 */

			/* toggle chip->pd_active to clean up the effect of
			 * smblib_uusb_removal() */
			tcpc_set_pd_type(chip, OPLUS_CHG_USB_TYPE_UNKNOWN);

			typec_set_pwr_role(chip->typec_port, TYPEC_SINK);
			tcpc_set_power_role(chip, POWER_ROLE_SINK);
		} else if (noti->swap_state.new_role == PD_ROLE_SOURCE) {
			chg_info("swap power role to source\n");
			/* report charger plug-out */

			typec_set_pwr_role(chip->typec_port, TYPEC_SOURCE);
			tcpc_set_power_role(chip, POWER_ROLE_SOURCE);
		}
		break;
	case TCP_NOTIFY_DR_SWAP:
		tcpc_handle_dr_swap(chip, noti);
		break;
	case TCP_NOTIFY_VCONN_SWAP:
		chg_info("vconn role swap, new role = %d\n",
			 noti->swap_state.new_role);
		if (noti->swap_state.new_role) {
			chg_info("swap vconn role to on\n");
			typec_set_vconn_role(chip->typec_port, TYPEC_SOURCE);
		} else {
			chg_info("swap vconn role to off\n");
			typec_set_vconn_role(chip->typec_port, TYPEC_SINK);
			cancel_delayed_work_sync(&chip->vconn_wait_work);
			schedule_delayed_work(&chip->vconn_wait_work, 0);
		}
		break;
	case TCP_NOTIFY_EXT_DISCHARGE:
		chg_info("ext discharge = %d\n", noti->en_state.en);
		/* enable/disable VBUS discharge */
		break;
	case TCP_NOTIFY_PD_STATE:
		chg_info("pd state = %d\n", noti->pd_state.connected);
		ret = tcpc_pd_state_change(chip, noti);
		break;
	case TCP_NOTIFY_HARD_RESET_STATE:
		switch (noti->hreset_state.state) {
		case TCP_HRESET_SIGNAL_SEND:
		case TCP_HRESET_SIGNAL_RECV:
			hard_reset = true;
			break;
		default:
			hard_reset = false;
			break;
		}
		/* smblib_set_prop(chip, POWER_SUPPLY_PROP_PD_IN_HARD_RESET, &val); */
		break;
#if defined(CONFIG_OPLUS_CHARGER_MTK) || IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
	case TCP_NOTIFY_WD0_STATE:
		oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_CC_DETECT);
		break;
#endif
#if IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
	case TCP_NOTIFY_PD_SOURCECAP_DONE:
		oplus_pd_manager_handle_sourcecap_done(chip, noti);
		break;
#endif
	default:
		break;
	}
	return NOTIFY_OK;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static int tcpc_typec_try_role(struct typec_port *port, int role)
{
	struct pd_manager_chip *chip = typec_get_drvdata(port);
#else
static int tcpc_typec_try_role(const struct typec_capability *cap, int role)
{
	struct pd_manager_chip *chip =
		container_of(cap, struct pd_manager_chip, typec_caps);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)) */
	uint8_t typec_role = TYPEC_ROLE_UNKNOWN;

	chg_info("role = %d\n", role);

	switch (role) {
	case TYPEC_NO_PREFERRED_ROLE:
		typec_role = TYPEC_ROLE_DRP;
		break;
	case TYPEC_SINK:
		typec_role = TYPEC_ROLE_TRY_SNK;
		break;
	case TYPEC_SOURCE:
		typec_role = TYPEC_ROLE_TRY_SRC;
		break;
	default:
		return 0;
	}

	return tcpm_typec_change_role_postpone(chip->tcpc, typec_role, true);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static int tcpc_typec_dr_set(struct typec_port *port, enum typec_data_role role)
{
	struct pd_manager_chip *chip = typec_get_drvdata(port);
#else
static int tcpc_typec_dr_set(const struct typec_capability *cap,
			     enum typec_data_role role)
{
	struct pd_manager_chip *chip =
		container_of(cap, struct pd_manager_chip, typec_caps);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)) */
	int ret = 0;
	uint8_t data_role = tcpm_inquire_pd_data_role(chip->tcpc);
	bool do_swap = false;

	chg_info("role = %d\n", role);

	if (role == TYPEC_HOST) {
		if (data_role == PD_ROLE_UFP) {
			do_swap = true;
			data_role = PD_ROLE_DFP;
		}
	} else if (role == TYPEC_DEVICE) {
		if (data_role == PD_ROLE_DFP) {
			do_swap = true;
			data_role = PD_ROLE_UFP;
		}
	} else {
		chg_err("invalid role\n");
		return -EINVAL;
	}

	if (do_swap) {
		ret = tcpm_dpm_pd_data_swap(chip->tcpc, data_role, NULL);
		if (ret != TCPM_SUCCESS) {
			chg_err("data role swap fail(%d)\n", ret);
			return -EPERM;
		}
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static int tcpc_typec_pr_set(struct typec_port *port, enum typec_role role)
{
	struct pd_manager_chip *chip = typec_get_drvdata(port);
#else
static int tcpc_typec_pr_set(const struct typec_capability *cap,
			     enum typec_role role)
{
	struct pd_manager_chip *chip =
		container_of(cap, struct pd_manager_chip, typec_caps);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)) */
	int ret = 0;
	uint8_t power_role = tcpm_inquire_pd_power_role(chip->tcpc);
	bool do_swap = false;

	chg_info("role = %d\n", role);

	if (role == TYPEC_SOURCE) {
		if (power_role == PD_ROLE_SINK) {
			do_swap = true;
			power_role = PD_ROLE_SOURCE;
		}
	} else if (role == TYPEC_SINK) {
		if (power_role == PD_ROLE_SOURCE) {
			do_swap = true;
			power_role = PD_ROLE_SINK;
		}
	} else {
		chg_err("invalid role\n");
		return -EINVAL;
	}

	if (do_swap) {
		ret = tcpm_dpm_pd_power_swap(chip->tcpc, power_role, NULL);
		if (ret == TCPM_ERROR_NO_PD_CONNECTED)
			ret = tcpm_typec_role_swap(chip->tcpc);
		if (ret != TCPM_SUCCESS) {
			chg_err("power role swap fail(%d)\n", ret);
			return -EPERM;
		}
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static int tcpc_typec_vconn_set(struct typec_port *port, enum typec_role role)
{
	struct pd_manager_chip *chip = typec_get_drvdata(port);
#else
static int tcpc_typec_vconn_set(const struct typec_capability *cap,
				enum typec_role role)
{
	struct pd_manager_chip *chip =
		container_of(cap, struct pd_manager_chip, typec_caps);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)) */
	int ret = 0;
	uint8_t vconn_role = tcpm_inquire_pd_vconn_role(chip->tcpc);
	bool do_swap = false;

	chg_info("role = %d\n", role);

	if (role == TYPEC_SOURCE) {
		if (vconn_role == PD_ROLE_VCONN_OFF) {
			do_swap = true;
			vconn_role = PD_ROLE_VCONN_ON;
		}
	} else if (role == TYPEC_SINK) {
		if (vconn_role == PD_ROLE_VCONN_ON) {
			do_swap = true;
			vconn_role = PD_ROLE_VCONN_OFF;
		}
	} else {
		chg_err("invalid role\n");
		return -EINVAL;
	}

	if (do_swap) {
		ret = tcpm_dpm_pd_vconn_swap(chip->tcpc, vconn_role, NULL);
		if (ret != TCPM_SUCCESS) {
			chg_err("vconn role swap fail(%d)\n", ret);
			return -EPERM;
		}
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static int tcpc_typec_port_type_set(struct typec_port *port,
				    enum typec_port_type type)
{
	struct pd_manager_chip *chip = typec_get_drvdata(port);
	const struct typec_capability *cap = &chip->typec_caps;
#else
static int tcpc_typec_port_type_set(const struct typec_capability *cap,
				    enum typec_port_type type)
{
	struct pd_manager_chip *chip =
		container_of(cap, struct pd_manager_chip, typec_caps);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)) */
	bool as_sink = tcpc_typec_is_act_as_sink_role(chip->tcpc);
	uint8_t typec_role = TYPEC_ROLE_UNKNOWN;

	chg_info("type = %d, as_sink = %d\n", type, as_sink);

	switch (type) {
	case TYPEC_PORT_SNK:
		if (as_sink)
			return 0;
		break;
	case TYPEC_PORT_SRC:
		if (!as_sink)
			return 0;
		break;
	case TYPEC_PORT_DRP:
		if (cap->prefer_role == TYPEC_SOURCE)
			typec_role = TYPEC_ROLE_TRY_SRC;
		else if (cap->prefer_role == TYPEC_SINK)
			typec_role = TYPEC_ROLE_TRY_SNK;
		else
			typec_role = TYPEC_ROLE_DRP;
		return tcpm_typec_change_role(chip->tcpc, typec_role);
	default:
		return 0;
	}

	return tcpm_typec_role_swap(chip->tcpc);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
const struct typec_operations tcpc_typec_ops = {
	.try_role = tcpc_typec_try_role,
	.dr_set = tcpc_typec_dr_set,
	.pr_set = tcpc_typec_pr_set,
	.vconn_set = tcpc_typec_vconn_set,
	.port_type_set = tcpc_typec_port_type_set,
};
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)) */

static int typec_init(struct pd_manager_chip *chip)
{
	int ret = 0;

	chip->typec_caps.type = TYPEC_PORT_DRP;
	chip->typec_caps.data = TYPEC_PORT_DRD;
	chip->typec_caps.revision = 0x0120;
	chip->typec_caps.pd_revision = 0x0300;
	switch (chip->tcpc->desc.role_def) {
	case TYPEC_ROLE_SRC:
	case TYPEC_ROLE_TRY_SRC:
		chip->typec_caps.prefer_role = TYPEC_SOURCE;
		break;
	case TYPEC_ROLE_SNK:
	case TYPEC_ROLE_TRY_SNK:
		chip->typec_caps.prefer_role = TYPEC_SINK;
		break;
	default:
		chip->typec_caps.prefer_role = TYPEC_NO_PREFERRED_ROLE;
		break;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	chip->typec_caps.driver_data = chip;
	chip->typec_caps.ops = &tcpc_typec_ops;
#else
	chip->typec_caps.try_role = tcpc_typec_try_role;
	chip->typec_caps.dr_set = tcpc_typec_dr_set;
	chip->typec_caps.pr_set = tcpc_typec_pr_set;
	chip->typec_caps.vconn_set = tcpc_typec_vconn_set;
	chip->typec_caps.port_type_set = tcpc_typec_port_type_set;
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)) */

	chip->typec_port = typec_register_port(chip->dev, &chip->typec_caps);
	if (IS_ERR(chip->typec_port)) {
		ret = PTR_ERR(chip->typec_port);
		chg_err("typec register port fail(%d)\n", ret);
		goto out;
	}

	chip->partner_desc.identity = &chip->partner_identity;
out:
	return ret;
}

static int oplus_pdc_setup(struct pd_manager_chip *chip, int *vbus_mv, int *ibus_ma)
{
	int ret = 0;
	int vbus_mv_t = 0;
	int ibus_ma_t = 0;
	struct tcpc_device *tcpc = chip->tcpc;

	if (*vbus_mv == VBUS_5V)
		ret = tcpm_set_pd_charging_policy(tcpc, DPM_CHARGING_POLICY_VSAFE5V, NULL);
	else
		ret = tcpm_set_pd_charging_policy(tcpc, DPM_CHARGING_POLICY_MAX_POWER_LVIC, NULL);

	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_set_apdo_charging_policy fail\n");
		return -EINVAL;
	}

	ret = tcpm_dpm_pd_request(tcpc, *vbus_mv, *ibus_ma, NULL);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail, rc=%d\n", ret);
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, &vbus_mv_t, &ibus_ma_t);
	if (ret != TCPM_SUCCESS) {
		chg_err("inquire current vbus_mv and ibus_ma fail\n");
		return -EINVAL;
	}

	chg_info("request vbus_mv[%d], ibus_ma[%d]\n", vbus_mv_t, ibus_ma_t);

	return 0;
}

static int oplus_pdo_select(struct pd_manager_chip *chip, int vbus_mv, int ibus_ma)
{
	int vbus = 5000, ibus = 2000;
	struct tcpm_power_cap_val apdo_cap;
	struct tcpm_remote_power_cap pd_cap;

	uint8_t cap_i = 0;
	int ret;
	int i;

	if (chip->pd_type == OPLUS_CHG_USB_TYPE_PD_PPS) {
		while (1) {
			ret = tcpm_inquire_pd_source_apdo(chip->tcpc,
					TCPM_POWER_CAP_APDO_TYPE_PPS,
					&cap_i, &apdo_cap);
			if (ret == TCPM_ERROR_NOT_FOUND) {
				break;
			} else if (ret != TCPM_SUCCESS) {
				chg_err("tcpm_inquire_pd_source_apdo failed, rc=%d\n", ret);
				break;
			}

			chg_info(
				"pps_boundary[%d], %d mv ~ %d mv, %d ma pl:%d\n",
				cap_i, apdo_cap.min_mv, apdo_cap.max_mv,
				apdo_cap.ma, apdo_cap.pwr_limit);

			if (apdo_cap.min_mv <= vbus_mv && vbus_mv <= apdo_cap.max_mv) {
				vbus = vbus_mv;
				ibus = apdo_cap.ma;
				if (ibus > ibus_ma)
					ibus = ibus_ma;
				break;
			}
		}
		if (cap_i == 0)
			chg_err("no APDO for pps\n");
	} else if (chip->pd_type == OPLUS_CHG_USB_TYPE_PD) {
		pd_cap.nr = 0;
		pd_cap.selected_cap_idx = 0;
		tcpm_get_remote_power_cap(chip->tcpc, &pd_cap);

		if (pd_cap.nr != 0) {
			for (i = 0; i < pd_cap.nr; i++) {
				if (vbus_mv <= pd_cap.max_mv[i]) {
					vbus = pd_cap.max_mv[i];
					ibus = pd_cap.ma[i];
					if (ibus > ibus_ma)
						ibus = ibus_ma;
					goto out;
				}
				chg_info("%d mv:[%d,%d] type:%d %d\n", i,
					 pd_cap.min_mv[i], pd_cap.max_mv[i],
					 pd_cap.ma[i], pd_cap.type[i]);
			}
			return -EINVAL;
		}
	} else {
		vbus = 5000;
		ibus = 2000;
	}
out:
	return oplus_pdc_setup(chip, &vbus, &ibus);
}

static int pd_manager_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int pd_manager_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

static int pd_manager_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int pd_manager_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[],
			       int len)
{
	return 0;
}

static int pd_manager_get_charger_type(struct oplus_chg_ic_dev *ic_dev,
				       int *type)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip)
		return -ENODEV;

	*type = chip->pd_type;

	return 0;
}

static int pd_manager_get_cc_orientation(struct oplus_chg_ic_dev *ic_dev,
					 int *orientation)
{
	struct pd_manager_chip *chip;
	int val;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (tcpm_inquire_typec_attach_state(chip->tcpc) != TYPEC_UNATTACHED)
		val = (int)tcpm_inquire_cc_polarity(chip->tcpc) + 1;
	else
		val = 0;
	*orientation = val;
	chg_info("cc_orientation = %d\n", val);

	return 0;
}

static int pd_manager_suspend_charger(bool suspend)
{
	struct votable *suspend_votable;
	int rc;

	suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	if (!suspend_votable) {
		chg_err("WIRED_CHARGE_SUSPEND votable not found\n");
		return -EINVAL;
	}

	rc = vote(suspend_votable, PDQC_VOTER, suspend, 0, true);
	if (rc < 0)
		chg_err("%s charger error, rc=%d\n",
			suspend ? "suspend" : "unsuspend", rc);
	else
		chg_info("%s charger\n", suspend ? "suspend" : "unsuspend");

	return rc;
}

static int tcpc_search_fixed_pdo(struct pd_manager_chip *chip, int volt)
{
	int i;

	for (i = 0; i < (PPS_PDO_MAX - 1); i++) {
		if (chip->pdo[i].pdo_type != USBPD_PDMSG_PDOTYPE_FIXED_SUPPLY)
			continue;

		if (volt <= PD_PDO_VOL(chip->pdo[i].voltage_50mv)) {
			chg_info("SourceCap[%d]: %08X, FixedSupply PDO V=%d mV, I=%d mA,"
				"UsbCommCapable=%d, USBSuspendSupported:%d\n", i,
				chip->pdo[i].pdo_data, PD_PDO_VOL(chip->pdo[i].voltage_50mv),
				PD_PDO_CURR_MAX(chip->pdo[i].max_current_10ma),
				chip->pdo[i].usb_comm_capable, chip->pdo[i].usb_suspend_supported);
			return PD_PDO_CURR_MAX(chip->pdo[i].max_current_10ma);
		}
	}
	return -EINVAL;
}

#define DEFAULT_PDO_CURR 500 /*500ma*/
static int tcpc_get_max_current_from_fixed_pdo(struct pd_manager_chip *chip, int volt)
{
	union mms_msg_data data = { 0 };

	if (!chip->wired_topic) {
		chg_err("wired_topic is NULL");
		return -EINVAL;
	}
	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE,
				&data, true);
	chip->chg_type = data.intval;
	oplus_wired_get_chg_type_str(chip->chg_type);
	if (!oplus_chg_get_common_charge_icl_support_flags())
		return -EINVAL;

	if (chip->pdo[0].pdo_data == 0) {
		if (chip->chg_type == OPLUS_CHG_USB_TYPE_PD ||
			chip->chg_type == OPLUS_CHG_USB_TYPE_PD_DRP ||
			chip->chg_type == OPLUS_CHG_USB_TYPE_PD_PPS ||
			chip->chg_type == OPLUS_CHG_USB_TYPE_PD_SDP) {
			chg_err("get pdo info error. return %dma\n", DEFAULT_PDO_CURR);
			return DEFAULT_PDO_CURR;
		}
		chg_err("get pdo info error\n");
		return -EINVAL;
	}

	return tcpc_search_fixed_pdo(chip, volt);
}

static int oplus_chg_set_icl_by_vote(int icl, const char *client_str)
{
	struct votable *icl_votable;
	int rc;

	icl_votable = find_votable("WIRED_ICL");
	if (!icl_votable) {
		chg_err("WIRED_ICL votable not found\n");
		return -EINVAL;
	}

	rc = vote(icl_votable, client_str, true, icl, true);
	if (rc < 0)
		chg_err("set icl error: icl = %d, rc = %d\n", icl, rc);
	else
		chg_info("real icl = %d\n", icl);

	return rc;
}

static void tcpc_sourcecap_done_work(struct work_struct *work)
{
	struct pd_manager_chip *chip = container_of(work,
		struct pd_manager_chip, sourcecap_done_work.work);
	int max_pdo_current = 0;

	if(!chip || !chip->ic_dev)
		return;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_POWER_CHANGED);
	/*set default input current from pdo*/
	max_pdo_current = tcpc_get_max_current_from_fixed_pdo(chip, VBUS_5V);
	if (max_pdo_current >= 0)
		oplus_chg_set_icl_by_vote(max_pdo_current, PD_PDO_ICL_VOTER);
}

static void tcpc_source_pdo_check_work(struct work_struct *work)
{
	struct pd_manager_chip *chip = container_of(work, struct pd_manager_chip, source_pdo_check_work.work);
	int max_pdo_current = 0;
	if(!chip)
		return;
	max_pdo_current = tcpc_get_max_current_from_fixed_pdo(chip, VBUS_9V);
	if (max_pdo_current < 0)
		max_pdo_current = tcpc_get_max_current_from_fixed_pdo(chip, VBUS_5V);
	if (max_pdo_current >= 0)
		oplus_chg_set_icl_by_vote(max_pdo_current, PD_PDO_ICL_VOTER);
}

#define PD_BOOST_DELAY_MS	300
static int pd_manager_set_pd_config(struct oplus_chg_ic_dev *ic_dev, u32 pdo)
{
	struct pd_manager_chip *chip;
	int vol_mv, curr_ma;
	int rc = 0;
	/*
	 * Reviewed-by-domain: in fcs boot stage we intentionally skip suspend/
	 * delay window here. Current limiting/protection is handled by FCS ICL
	 * voting path before PDO switch, so do not re-introduce suspend around
	 * oplus_pdo_select() for this branch.
	 */
	bool fcs_support = oplus_chg_get_fcs_support_flags() && !oplus_comm_get_boot_completed();

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!fcs_support)
		pd_manager_suspend_charger(true);
	switch (PD_SRC_PDO_TYPE(pdo)) {
	case PD_SRC_PDO_TYPE_FIXED:
		vol_mv = PD_SRC_PDO_FIXED_VOLTAGE(pdo) * 50;
		curr_ma = PD_SRC_PDO_FIXED_MAX_CURR(pdo) * 10;
		rc = oplus_pdo_select(chip, vol_mv, curr_ma);
		if (rc < 0)
			chg_err("set PD to %d mV fail, rc=%d\n", vol_mv, rc);
		else
			chg_err("set PD to %d mV, rc=%d\n", vol_mv, rc);
		break;
	case PD_SRC_PDO_TYPE_BATTERY:
	case PD_SRC_PDO_TYPE_VARIABLE:
	case PD_SRC_PDO_TYPE_AUGMENTED:
	default:
		chg_err("Unsupported pdo type(=%d)\n", PD_SRC_PDO_TYPE(pdo));
		rc = -EINVAL;
		goto out;
	}
	if (!fcs_support)
		msleep(PD_BOOST_DELAY_MS);
	schedule_delayed_work(&chip->source_pdo_check_work, msecs_to_jiffies(WAIT_SOURCECAP_DOWN));
out:
	if (!fcs_support)
		pd_manager_suspend_charger(false);
	return rc;
}

static int tcpc_get_power_role(struct oplus_chg_ic_dev *ic_dev, int *power_role)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip) {
		chg_err("chip is NULL");
		return -ENODEV;
	}
	*power_role = chip->power_role;
	return 0;
}

static int pd_manager_get_typec_mode(struct oplus_chg_ic_dev *ic_dev,
				     enum oplus_chg_typec_port_role_type *mode)
{
	struct pd_manager_chip *chip;
	uint8_t typec_state;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	typec_state = tcpm_inquire_typec_attach_state(chip->tcpc);

	switch(typec_state) {
	case TYPEC_ATTACHED_SNK:
	case TYPEC_ATTACHED_DBGACC_SNK:
		*mode = TYPEC_PORT_ROLE_SNK;
		break;
	case TYPEC_ATTACHED_SRC:
	case TYPEC_ATTACHED_CUSTOM_SRC:
	case TYPEC_ATTACHED_NORP_SRC:
		*mode = TYPEC_PORT_ROLE_SRC;
		break;
	default:
		*mode = TYPEC_PORT_ROLE_INVALID;
		break;
	}
	return 0;
}

static int pd_manager_get_typec_role(struct oplus_chg_ic_dev *ic_dev,
				     enum oplus_chg_typec_port_role_type *mode)
{
	struct pd_manager_chip *chip;
	uint8_t typec_role;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	typec_role = tcpm_inquire_typec_role(chip->tcpc);
	chg_info("typec_role is %d", typec_role);
	switch(typec_role) {
	case TYPEC_ROLE_SNK:
		*mode = TYPEC_PORT_ROLE_SNK;
		break;
	case TYPEC_ROLE_SRC:
		*mode = TYPEC_PORT_ROLE_SRC;
		break;
	case TYPEC_ROLE_DRP:
		*mode = TYPEC_PORT_ROLE_DRP;
		break;
	case TYPEC_ROLE_TRY_SRC:
		*mode = TYPEC_PORT_ROLE_TRY_SRC;
		break;
	case TYPEC_ROLE_TRY_SNK:
		*mode = TYPEC_PORT_ROLE_TRY_SNK;
		break;
	case TYPEC_ROLE_UNKNOWN:
	case TYPEC_ROLE_NR:
	default:
		*mode = TYPEC_PORT_ROLE_INVALID;
		break;
	}

	return 0;
}

static int pd_manager_set_typec_mode(struct oplus_chg_ic_dev *ic_dev,
				     enum oplus_chg_typec_port_role_type mode)
{
	struct pd_manager_chip *chip;
	uint8_t typec_role;
	int rc;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip || !chip->tcpc) {
		chg_err("chip or chip->tcpc is null\n");
		return  -ENODEV;
	}

	switch(mode) {
	case TYPEC_PORT_ROLE_DRP:
		typec_role = TYPEC_ROLE_DRP;
		break;
	case TYPEC_PORT_ROLE_SNK:
		typec_role = TYPEC_ROLE_SNK;
		break;
	case TYPEC_PORT_ROLE_SRC:
		typec_role = TYPEC_ROLE_SRC;
		break;
	case TYPEC_PORT_ROLE_TRY_SNK:
		typec_role = TYPEC_ROLE_TRY_SNK;
		break;
	case TYPEC_PORT_ROLE_TRY_SRC:
		typec_role = TYPEC_ROLE_TRY_SRC;
		break;
	case TYPEC_PORT_ROLE_DISABLE:
		rc = tcpm_typec_disable_function(chip->tcpc, true);
		return rc;
	case TYPEC_PORT_ROLE_ENABLE:
		rc = tcpm_typec_disable_function(chip->tcpc, false);
		return rc;
	case TYPEC_PORT_ROLE_INVALID:
		typec_role = TYPEC_ROLE_UNKNOWN;
		break;
	}

	rc = tcpm_typec_change_role_postpone(chip->tcpc, typec_role, true);
	if (rc < 0)
		chg_err("can't set typec_role = %d\n", mode);

	return rc;
}

static int pd_manager_get_charger_vol_max(struct oplus_chg_ic_dev *ic_dev,
					  int *vol)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*vol = chip->voltage_max_mv;
	return 0;
}

static int pd_manager_get_charger_vol_min(struct oplus_chg_ic_dev *ic_dev,
					  int *vol)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*vol = chip->voltage_min_mv;
	return 0;
}

static int pd_manager_get_charger_curr_max(struct oplus_chg_ic_dev *ic_dev,
					   int *curr)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*curr = chip->current_max_ma;
	return 0;
}

static int pd_manager_bc12_completed(struct oplus_chg_ic_dev *ic_dev)
{
	struct pd_manager_chip *chip;
	union mms_msg_data data = { 0 };
	static bool first_boot = true;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (first_boot) {
		first_boot = false;
		oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE,
					&data, true);
		chip->chg_type = data.intval;
		chg_info("chg_type=%s\n", oplus_wired_get_chg_type_str(chip->chg_type));
		if (chip->chg_type == OPLUS_CHG_USB_TYPE_SDP ||
		    chip->chg_type == OPLUS_CHG_USB_TYPE_CDP ||
		    chip->chg_type == OPLUS_CHG_USB_TYPE_PD_SDP)
			chip->start_peripheral = true;
	}

	chip->bc12_completed = true;
	chip->bc12_ready = true;
	chg_info("bc1.2 completed, start_peripheral=%d\n",
		 chip->start_peripheral);
	if (chip->start_peripheral)
		start_usb_peripheral(chip);

	return 0;
}

static int pd_manager_get_otg_enable(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*en = chip->otg_enable;

	return 0;
}

static int tcpc_get_reverse_enable(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct pd_manager_chip *chip;
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	*enable = chip->reverse_enable;
	return 0;
}

static int tcpc_get_high_reverse_enable(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct pd_manager_chip *chip;
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	*enable = chip->high_reverse_enable;
	return 0;
}

static int tcpc_get_reverse_chg_svid(struct oplus_chg_ic_dev *ic_dev, uint32_t *svid)
{
	struct pd_manager_chip *chip;
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (chip == NULL) {
		chg_err("chip is NULL");
		return -ENODEV;
	}

	*svid = chip->reverse_chg_svid;
	return 0;
}

static int pd_manager_is_oplus_svid(struct oplus_chg_ic_dev *ic_dev, bool *oplus_svid)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip) {
		chg_err("chip is null\n");
		return  -ENODEV;
	}

	*oplus_svid = chip->pd_svooc;

	return 0;
}

static int tcpc_get_rvs_chg_msg_type(struct oplus_chg_ic_dev *ic_dev, int *msg_type)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip) {
		chg_err("chip is null\n");
		return  -ENODEV;
	}

	*msg_type = chip->msg_type;

	return 0;
}

static int tcpc_get_sink_req_pdo(struct oplus_chg_ic_dev *ic_dev, int *vbus_mv_t, int *ibus_ma_t)
{
	int ret = 0;
	struct tcpc_device *tcpc = NULL;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract(tcpc, vbus_mv_t, ibus_ma_t);

	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_inquire_pd_contract fail, ret=%d\n", ret);
		return -EINVAL;
	}
	chg_info("request vbus_mv[%d], ibus_ma[%d]\n", *vbus_mv_t, *ibus_ma_t);

	return 0;
}

#if IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
static int tcpm_inquire_pd_contract_rdo_position(struct tcpc_device *tcpc,
	uint8_t *rdo_pos)
{
	int ret;
	struct pd_port *pd_port;
	struct pe_data *pe_data;

	if (tcpc == NULL || rdo_pos == NULL)
		return -EINVAL;
	pd_port = &tcpc->pd_port;
	pe_data = &pd_port->pe_data;
	ret = tcpm_check_pd_attached(tcpc);
	if (ret != TCPM_SUCCESS) {
		chg_err("tcpm_check_pd_attached fail, ret=%d\n", ret);
		return ret;
	}

	mutex_lock(&pd_port->pd_lock);
	if (pd_port->pe_data.explicit_contract)
		*rdo_pos = pe_data->selected_cap;
	else
		ret = TCPM_ERROR_NO_EXPLICIT_CONTRACT;
	mutex_unlock(&pd_port->pd_lock);

	return ret;
}

static int tcpc_get_reverse_chg_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	int ret = 0;
	struct tcpc_device *tcpc = NULL;
	uint8_t rdo_pos;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		chg_err("get type_c_port0 fail\n");
		return -EINVAL;
	}

	ret = tcpm_inquire_pd_contract_rdo_position(tcpc, &rdo_pos);
	if (ret != TCPM_SUCCESS) {
		*type = REVERSE_CHG_TYPE_UNKNOWN;
		chg_err("tcpm_inquire_pd_contract_rdo_position fail, ret=%d\n", ret);
		return -EINVAL;
	}
	chg_info("request rdo_pos[0x%X]\n", rdo_pos);
	if (((rdo_pos - 1) & 0x03) == 0x03)
		*type = REVERSE_CHG_TYPE_PPS;
	else if (rdo_pos)
		*type = REVERSE_CHG_TYPE_PD;
	else
		*type = REVERSE_CHG_TYPE_NORMAL;
	chg_info("request type[%d]\n", *type);

	return 0;
}
#endif

static int tcpc_get_source_plug_out(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct pd_manager_chip *chip;
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip) {
		chg_err("chip is null\n");
		return  -ENODEV;
	}

	*enable = chip->source_plug_out;
	return 0;
}

static int pd_manager_get_data_role(struct oplus_chg_ic_dev *ic_dev, int *role)
{
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip) {
		chg_err("chip is null\n");
		return  -ENODEV;
	}

	*role = chip->data_role;

	return 0;
}

static void oplus_reverse_chg_svid_check_work(struct work_struct *work)
{
	int ret = 0;
	struct pd_manager_chip *chip = container_of(work,
		struct pd_manager_chip, reverse_chg_svid_check_work.work);
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	int disc_svid_retries = 0;
	uint32_t data[VDO_MAX_NR] = {0};

	if (tcpc_dev == NULL) {
		chg_err("tcpc_dev is null return\n");
		return;
	}

	do {
		disc_svid_retries++;
		if (disc_svid_retries > DISCOVER_SVID_MAX_RETRIES)
			break;

		ret = oplus_discover_id(tcpc_dev);
		if (ret == -EFAULT) {
			chg_err("oplus_discover_id failed, ret = %d, retries: %d, not try again.\n",
				ret, disc_svid_retries);
			break;
		} else if (ret != TCPM_SUCCESS) {
			chg_err("oplus_discover_id not success,  disc_svid_retries: %d, ret = %d\n",
				disc_svid_retries, ret);
			msleep(DISCOVER_SVID_RETRY_DELAY);
			continue;
		}

		/* retry to get the SVID by PD partner inform. */
		msleep(DISCOVER_SVID_INTERNAL_CMD_DELAY);

		ret = tcpm_inquire_pd_partner_inform(tcpc_dev, data);
		if (ret == TCPM_SUCCESS) {
			chip->reverse_chg_svid = data[0] & 0xFFFF;
			chg_info("reverse chg get the svid = 0x%04X", chip->reverse_chg_svid);
			break;
		} else if (ret != TCPM_SUCCESS) {
			chg_err("tcpm_inquire_pd_partner_inform failed, ret = %d, retries = %d.\n",
				ret, disc_svid_retries);
			msleep(DISCOVER_SVID_RETRY_DELAY);
		}
	} while (ret != TCPM_SUCCESS && disc_svid_retries < DISCOVER_SVID_MAX_RETRIES);
}

static int oplus_chg_set_fixed_pd_config(struct oplus_chg_ic_dev *ic_dev, int vol_mv, int curr_ma)
{
	int rc = 0;
	struct pd_manager_chip *chip;

#define OPLUS_PD_5V_PDO 0x31912c
#define OPLUS_PD_9V_PDO 0x32d12c

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (curr_ma >= MAX_PD_INPUT_CURRENT)
		curr_ma = MAX_PD_INPUT_CURRENT;
	if ((oplus_chg_get_fcs_support_flags() && !oplus_comm_get_boot_completed())) {
		oplus_pdc_setup(chip, &vol_mv, &curr_ma);
	} else {
		pd_manager_suspend_charger(true);
		if (vol_mv == VBUS_5V)
			rc = pd_manager_set_pd_config(ic_dev, OPLUS_PD_5V_PDO);
		else
			rc = pd_manager_set_pd_config(ic_dev, OPLUS_PD_9V_PDO);
		if (rc < 0) {
			pd_manager_suspend_charger(false);
			return rc;
		}
		oplus_pdc_setup(chip, &vol_mv, &curr_ma);
		msleep(DELAY_TIME);
		pd_manager_suspend_charger(false);
	}

	return rc;
}

static int oplus_chg_set_pps_config(struct oplus_chg_ic_dev *ic_dev, int vbus_mv, int ibus_ma)
{
	int rc = 0;
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	chg_info("request vbus_mv[%d], ibus_ma[%d]\n", vbus_mv, ibus_ma);
	if (chip->tcpc->pd_port.dpm_charging_policy != DPM_CHARGING_POLICY_PPS) {
		rc = tcpm_set_apdo_charging_policy(chip->tcpc, DPM_CHARGING_POLICY_PPS, vbus_mv, ibus_ma, NULL);
		if (rc == TCP_DPM_RET_REJECT) {
			chg_err("set_apdo_charging_policy reject\n");
			return 0;
		} else if (rc != 0) {
			chg_err("set_apdo_charging_policy error\n");
			return -EINVAL;
		}
	}

	rc = tcpm_dpm_pd_request(chip->tcpc, vbus_mv, ibus_ma, NULL);
	if (rc != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail\n");
		return -EINVAL;
	}

	return 0;
}

static int oplus_get_pps_info(struct oplus_chg_ic_dev *ic_dev, u32 *pdo, int num)
{
	struct tcpm_power_cap cap;
	int pdo_max = num > PPS_PDO_MAX ? PPS_PDO_MAX : num;
	int rc, pdo_index;
	struct pd_manager_chip *chip;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = tcpm_dpm_pd_get_source_cap(chip->tcpc, NULL);
	if (rc == TCPM_SUCCESS)
		chg_info("tcpm_dpm_pd_get_source_cap_ext ok(%d)\n", rc);
	else
		chg_err("tcpm_dpm_pd_get_source_cap_ext failed(%d)\n", rc);

	rc = tcpm_inquire_pd_source_cap(chip->tcpc, &cap);
	if (rc != TCPM_SUCCESS)
		return rc;

	for (pdo_index = 0; pdo_index < pdo_max; pdo_index++)
		pdo[pdo_index] = cap.pdos[pdo_index];

	return 0;
}

static int mtk_chg_get_source_pdo(struct oplus_chg_ic_dev *ic_dev, u32 *data, int *num)
{
	struct pd_manager_chip *chip;
	int pdo_index;

	if (ic_dev  == NULL) {
		chg_err("ic_dev is NULL");
		return -EINVAL;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip) {
		chg_err("chip is NULL");
		return -EINVAL;
	}
	*num = chip->cap_nr;
	for (pdo_index = 0; pdo_index < chip->cap_nr; pdo_index++) {
		data[pdo_index] = chip->pdo[pdo_index].pdo_data;
	}

	return 0;
}

static void *oplus_chg_get_func(struct oplus_chg_ic_dev *ic_dev,
				enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, pd_manager_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, pd_manager_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, pd_manager_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, pd_manager_smt_test);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE, pd_manager_get_charger_type);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION, pd_manager_get_cc_orientation);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG, pd_manager_set_pd_config);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_POWER_ROLE, tcpc_get_power_role);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_MODE, pd_manager_get_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_TYPEC_MODE, pd_manager_set_typec_mode);
		break;
	case OPLUS_IC_FUNC_GET_CHARGER_VOL_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_CHARGER_VOL_MAX, pd_manager_get_charger_vol_max);
		break;
	case OPLUS_IC_FUNC_GET_CHARGER_VOL_MIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_CHARGER_VOL_MIN, pd_manager_get_charger_vol_min);
		break;
	case OPLUS_IC_FUNC_GET_CHARGER_CURR_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_CHARGER_CURR_MAX, pd_manager_get_charger_curr_max);
		break;
	case OPLUS_IC_FUNC_BUCK_BC12_COMPLETED:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_BC12_COMPLETED, pd_manager_bc12_completed);
		break;
	case OPLUS_IC_FUNC_GET_OTG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_ENABLE, pd_manager_get_otg_enable);
		break;
	case OPLUS_IC_FUNC_GET_SOURCE_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SOURCE_PDO, mtk_chg_get_source_pdo);
		break;
	case OPLUS_IC_FUNC_IS_OPLUS_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_IS_OPLUS_SVID, pd_manager_is_oplus_svid);
		break;
	case OPLUS_IC_FUNC_GET_DATA_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_DATA_ROLE, pd_manager_get_data_role);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_ROLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_ROLE, pd_manager_get_typec_role);
		break;
	case OPLUS_IC_FUNC_FIXED_PDO_SET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_FIXED_PDO_SET, oplus_chg_set_fixed_pd_config);
		break;
	case OPLUS_IC_FUNC_PPS_PDO_SET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_PPS_PDO_SET, oplus_chg_set_pps_config);
		break;
	case OPLUS_IC_FUNC_PPS_GET_PDO_INFO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_PPS_GET_PDO_INFO, oplus_get_pps_info);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq pd_manager_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_CHG_TYPE_CHANGE },
	{ .virq_id = OPLUS_IC_VIRQ_CC_DETECT },
	{ .virq_id = OPLUS_IC_VIRQ_SVID },
	{ .virq_id = OPLUS_IC_VIRQ_VOLTAGE_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_CURRENT_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_OTG_ENABLE },
	{ .virq_id = OPLUS_IC_VIRQ_POWER_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_POWER_ROLE_STATUS },
	{ .virq_id = OPLUS_IC_VIRQ_DATA_ROLE_CHANGED },
};

static void tcpc_wired_subs_callback(struct mms_subscribe *subs,
				     enum mms_msg_type type, u32 id, bool sync)
{
	struct pd_manager_chip *chip = subs->priv_data;
	union mms_msg_data data = { 0 };

	switch (type) {
	case MSG_TYPE_ITEM:
		switch (id) {
		case WIRED_ITEM_CHG_TYPE:
			oplus_mms_get_item_data(chip->wired_topic, id, &data,
						false);
			chip->chg_type = data.intval;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void tcpc_subscribe_wired_topic(struct oplus_mms *topic, void *prv_data)
{
	struct pd_manager_chip *chip = prv_data;
	union mms_msg_data data = { 0 };

	chip->wired_topic = topic;
	chip->wired_subs =
		oplus_mms_subscribe(chip->wired_topic, chip,
				    tcpc_wired_subs_callback, "tcpc");
	if (IS_ERR_OR_NULL(chip->wired_subs)) {
		chg_err("subscribe wired topic error, rc=%ld\n",
			PTR_ERR(chip->wired_subs));
		return;
	}

	oplus_mms_get_item_data(chip->wired_topic, WIRED_ITEM_CHG_TYPE, &data,
				true);
	chip->chg_type = data.intval;

	/* Re-notify the typec status after the driver is loaded */
	chip->first_check = true;
	tcpci_notify_typec_state(chip->tcpc);
}

static void tcpc_variable_init(struct pd_manager_chip *chip)
{
	chip->chg_type = OPLUS_CHG_USB_TYPE_UNKNOWN;
	chip->voltage_max_mv = MICRO_5V;
	chip->voltage_min_mv = MICRO_5V;
	chip->current_max_ma = 0;
}

static void oplus_pd_tcpc_complete_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pd_manager_chip *chip = container_of(dwork, struct pd_manager_chip, tcpc_complete_work);

	if (chip->tcpc != NULL) {
		tcpc_device_irq_enable(chip->tcpc);
		chg_info("enable tcpc_device irq");
	}

	return;
}

static int tcpc_reverse_chg_init(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	ic_dev->online = true;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_ONLINE);
	return 0;
}

static int tcpc_reverse_chg_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	oplus_chg_ic_virq_trigger(ic_dev, OPLUS_IC_VIRQ_OFFLINE);
	return 0;
}

static int tcpc_reverse_chg_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int  tcpc_reverse_chg_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int  tcpc_set_wdt_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	return 0;
}

static int  tcpc_set_kick_wdt(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static void *oplus_chg_reverse_chg_get_func(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT, tcpc_reverse_chg_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT, tcpc_reverse_chg_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP, tcpc_reverse_chg_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST, tcpc_reverse_chg_smt_test);
		break;
	case OPLUS_IC_FUNC_GET_REVERSE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_ENABLE, tcpc_get_reverse_enable);
		break;
	case OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_HIGH_REVERSE_ENABLE, tcpc_get_high_reverse_enable);
		break;
#if IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
	case OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_RVS_SET_HIGH_PWR_MODE_EN, tcpc_set_rvs_high_mode_en);
		break;
	case OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_REVERSE_SRC_PDO, tcpc_set_reverse_boost_pdo);
		break;
#endif
	case OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_CHG_SVID, tcpc_get_reverse_chg_svid);
		break;
	case OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REVERSE_CHG_WDT_ENABLE, tcpc_set_wdt_enable);
		break;
	case OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REVERSE_CHG_KICK_WDT, tcpc_set_kick_wdt);
		break;
	case OPLUS_IC_FUNC_GET_RVS_CHG_MSG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_RVS_CHG_MSG, tcpc_get_rvs_chg_msg_type);
		break;
	case OPLUS_IC_FUNC_GET_SINK_REQ_PDO:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SINK_REQ_PDO, tcpc_get_sink_req_pdo);
		break;
#if IS_ENABLED(CONFIG_OPLUS_PD_EXT_SUPPORT)
	case OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_REVERSE_CHG_TYPE, tcpc_get_reverse_chg_type);
		break;
#endif
	case OPLUS_IC_FUNC_GET_SOURCE_PLUG_OUT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SOURCE_PLUG_OUT, tcpc_get_source_plug_out);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static struct oplus_chg_ic_virq oplus_chg_reverse_chg_virq_table[] = {
	{.virq_id = OPLUS_IC_VIRQ_ERR },
	{.virq_id = OPLUS_IC_VIRQ_ONLINE },
	{.virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{.virq_id = OPLUS_IC_VIRQ_REVERSE_ENABLE },
	{.virq_id = OPLUS_IC_VIRQ_HIGH_REVERSE_ENABLE },
	{.virq_id = OPLUS_IC_VIRQ_HARD_RESET },
	{.virq_id = OPLUS_IC_VIRQ_SINK_REQ_MSG },
};

static int oplus_pd_parse_child_node_ic_info(struct device_node *child,
					      enum oplus_chg_ic_type *ic_type,
					      int *ic_index)
{
	int ret;

	ret = of_property_read_u32(child, "oplus,ic_type", ic_type);
	if (ret >= 0) {
		ret = of_property_read_u32(child, "oplus,ic_index", ic_index);
		if (ret >= 0)
			chg_info("node %s, ic_type = %d, ic_index = %d\n", child->name, *ic_type, *ic_index);
		else
			chg_err("can't get %s ic index, ret=%d\n", child->name, ret);
	} else {
		chg_err("can't get %s ic type, ret=%d\n", child->name, ret);
	}
	return ret;
}

static struct oplus_chg_ic_dev *oplus_pd_register_reverse_chg_ic(
	struct pd_manager_chip *chip,
	struct device_node *child,
	enum oplus_chg_ic_type ic_type,
	int ic_index)
{
	struct oplus_chg_ic_dev *ic_dev;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };

	ic_cfg.name = child->name;
	ic_cfg.index = ic_index;
	ic_cfg.type = ic_type;
	ic_cfg.of_node = child;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "reverse-PD_MANAGER");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.get_func = oplus_chg_reverse_chg_get_func;
	ic_cfg.virq_data = oplus_chg_reverse_chg_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(oplus_chg_reverse_chg_virq_table);

	ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!ic_dev)
		chg_err("register %s error\n", child->name);
	else
		chg_info("register %s success.\n", child->name);

	return ic_dev;
}

static void oplus_pd_manager_register_child_ic(struct pd_manager_chip *chip,
					     struct device_node *child)
{
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	struct oplus_chg_ic_dev *ic_dev;
	int ret;

	ret = oplus_pd_parse_child_node_ic_info(child, &ic_type, &ic_index);
	if (ret < 0)
		return;

	if (ic_type != OPLUS_CHG_IC_REVERSE)
		return;
	ic_dev = oplus_pd_register_reverse_chg_ic(chip, child, ic_type, ic_index);
	if (!ic_dev)
		return;

	chip->reverse_chg_ic_dev = ic_dev;
	oplus_chg_ic_func(ic_dev, OPLUS_IC_FUNC_INIT);
}

static void oplus_pd_manager_init_delayed_works(struct pd_manager_chip *chip)
{
	INIT_DELAYED_WORK(&chip->bc12_wait_work, tcpc_bc12_wait_work);
	INIT_DELAYED_WORK(&chip->vconn_wait_work, tcpc_vconn_wait_work);
	INIT_DELAYED_WORK(&chip->svid_check_work, oplus_svid_check_work);
	INIT_DELAYED_WORK(&chip->sink_request_check_work, tcpc_sink_request_check_work);
	INIT_DELAYED_WORK(&chip->reverse_chg_svid_check_work, oplus_reverse_chg_svid_check_work);
	INIT_DELAYED_WORK(&chip->source_pdo_check_work, tcpc_source_pdo_check_work);
	INIT_DELAYED_WORK(&chip->sourcecap_done_work, tcpc_sourcecap_done_work);
	INIT_DELAYED_WORK(&chip->usb_dwork, usb_dwork_handler);
}

static void oplus_pd_manager_init_chip_vars(struct pd_manager_chip *chip)
{
	chip->usb_dr = DR_IDLE;
	chip->usb_type_polling_cnt = 0;
	chip->sink_mv_old = -1;
	chip->sink_ma_old = -1;
}

static int oplus_pd_manager_probe(struct platform_device *pdev)
{
	int ret = 0;
	static int probe_cnt = 0;
	struct pd_manager_chip *chip = NULL;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child;
	enum oplus_chg_ic_type ic_type;
	int ic_index;

	chg_info("probe_cnt = %d\n", ++probe_cnt);

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->oplus_pd_sdp_svid = false;
	oplus_pd_manager_init_delayed_works(chip);

	ret = extcon_init(chip);
	if (ret) {
		chg_err("init extcon fail(%d)\n", ret);
		ret = -EPROBE_DEFER;
		if (probe_cnt >= PROBE_CNT_MAX)
			goto out;
		else
			goto err_init_extcon;
	}

	chip->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!chip->tcpc) {
		chg_err("get tcpc dev fail\n");
		ret = -EPROBE_DEFER;
		if (probe_cnt >= PROBE_CNT_MAX)
			goto out;
		else
			goto err_get_tcpc_dev;
	}

	INIT_DELAYED_WORK(&chip->usb_dwork, usb_dwork_handler);
	oplus_pd_manager_init_chip_vars(chip);

	ret = typec_init(chip);
	if (ret < 0) {
		chg_err("init typec fail(%d)\n", ret);
		ret = -EPROBE_DEFER;
		if (probe_cnt >= PROBE_CNT_MAX)
			goto out;
		else
			goto err_init_typec;
	}

	chip->pd_nb.notifier_call = pd_tcp_notifier_call;
	ret = register_tcp_dev_notifier(chip->tcpc, &chip->pd_nb,
					TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		chg_err("register tcpc notifier fail(%d)\n", ret);
		ret = -EPROBE_DEFER;
		if (probe_cnt >= PROBE_CNT_MAX)
			goto out;
		else
			goto err_reg_tcpc_notifier;
	}

	ret = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (ret < 0) {
		chg_err("can't get ic type, rc=%d\n", ret);
		goto reg_ic_err;
	}
	ret = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (ret < 0) {
		chg_err("can't get ic index, rc=%d\n", ret);
		goto reg_ic_err;
	}
	chip->enable_tcpc_irq = of_property_read_bool(node, "oplus,enable_tcpc_irq");
	chg_info("enable_tcpc_irq:%d", chip->enable_tcpc_irq);

	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, OPLUS_CHG_IC_MANU_NAME_MAX - 1, "buck-PD_MANAGER");
	snprintf(ic_cfg.fw_id, OPLUS_CHG_IC_FW_ID_MAX - 1, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = pd_manager_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(pd_manager_virq_table);
	ic_cfg.of_node = node;
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		ret = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}
	for_each_child_of_node(node, child)
		oplus_pd_manager_register_child_ic(chip, child);
	chip->batt_psy = power_supply_get_by_name("battery");
	chip->cpa_support = oplus_cpa_support();

	if (chip->enable_tcpc_irq) {
		INIT_DELAYED_WORK(&chip->tcpc_complete_work, oplus_pd_tcpc_complete_work);
		schedule_delayed_work(&chip->tcpc_complete_work, msecs_to_jiffies(100));
	}

out:
	platform_set_drvdata(pdev, chip);
	tcpc_variable_init(chip);
	oplus_mms_wait_topic("wired", tcpc_subscribe_wired_topic, chip);
	chg_info("%s!!\n", ret == -EPROBE_DEFER ? "Over probe cnt max" : "OK");
	return 0;

reg_ic_err:
	unregister_tcp_dev_notifier(chip->tcpc, &chip->pd_nb,
				    TCP_NOTIFY_TYPE_ALL);
err_reg_tcpc_notifier:
	typec_unregister_port(chip->typec_port);
err_init_typec:
err_get_tcpc_dev:
err_init_extcon:
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static void oplus_pd_manager_remove(struct platform_device *pdev)
#else
static int oplus_pd_manager_remove(struct platform_device *pdev)
#endif
{
	int ret = 0;
	struct pd_manager_chip *chip = platform_get_drvdata(pdev);

	if (!chip) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
		return -EINVAL;
#else
		return;
#endif
	}

	if (!IS_ERR_OR_NULL(chip->wired_subs))
		oplus_mms_unsubscribe(chip->wired_subs);

	ret = unregister_tcp_dev_notifier(chip->tcpc, &chip->pd_nb,
					  TCP_NOTIFY_TYPE_ALL);
	if (ret < 0)
		chg_err("unregister tcpc notifier fail(%d)\n", ret);
	cancel_delayed_work_sync(&chip->sourcecap_done_work);
	cancel_delayed_work_sync(&chip->source_pdo_check_work);
	cancel_delayed_work_sync(&chip->sink_request_check_work);
	cancel_delayed_work_sync(&chip->reverse_chg_svid_check_work);
	typec_unregister_port(chip->typec_port);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	if (chip->usb_psy)
		power_supply_put(chip->usb_psy);
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)) */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	return ret;
#endif
}

static const struct of_device_id oplus_pd_manager_of_match[] = {
	{ .compatible = "oplus,hal-pd-manager" },
	{ }
};
MODULE_DEVICE_TABLE(of, oplus_pd_manager_of_match);

static struct platform_driver oplus_pd_manager_driver = {
	.driver = {
		.name = "oplus-pd_manager",
		.of_match_table = of_match_ptr(oplus_pd_manager_of_match),
	},
	.probe = oplus_pd_manager_probe,
	.remove = oplus_pd_manager_remove,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static int __init oplus_pd_manager_init(void)
{
	return platform_driver_register(&oplus_pd_manager_driver);
}

static void __exit oplus_pd_manager_exit(void)
{
	platform_driver_unregister(&oplus_pd_manager_driver);
}

late_initcall(oplus_pd_manager_init);
module_exit(oplus_pd_manager_exit);
#else
int oplus_pd_manager_init(void)
{
	return platform_driver_register(&oplus_pd_manager_driver);
}

void oplus_pd_manager_exit(void)
{
	platform_driver_unregister(&oplus_pd_manager_driver);
}
oplus_chg_module_register(oplus_pd_manager);
#endif
