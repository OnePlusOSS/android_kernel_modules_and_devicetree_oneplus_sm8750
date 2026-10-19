/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2016-2019 The Linux Foundation. All rights reserved.
 */

#ifndef __PMIC_VOTER_H
#define __PMIC_VOTER_H

#include <linux/mutex.h>

struct votable;

enum votable_type {
	VOTE_MIN,
	VOTE_MAX,
	VOTE_SET_ANY,
	NUM_VOTABLE_TYPES,
};

#define PLC_VOTER		"PLC_VOTER"
#define PLC_RETRY_VOTER		"PLC_RETRY_VOTER"
#define PLC_SOC_VOTER		"PLC_SOC_VOTER"

#define OVERRIDE_VOTER		"OVERRIDE_VOTER"
#define JEITA_VOTER		"JEITA_VOTER"
#define STEP_VOTER		"STEP_VOTER"
#define USER_VOTER		"USER_VOTER"
#define DEF_VOTER		"DEF_VOTER"
#define LIQ_ERR_VOTER		"LIQ_ERR_VOTER"
#define MAX_VOTER		"MAX_VOTER"
#define BASE_MAX_VOTER		"BASE_MAX_VOTER"
#define ADAPTER_MAX_POWER	"ADAPTER_MAX_POWER"
#define CABLE_MAX_VOTER		"CABLE_MAX_VOTER"
#define PR_VOTER		"PR_VOTER"
#define RX_MAX_VOTER		"RX_MAX_VOTER"
#define EXIT_VOTER		"EXIT_VOTER"
#define FCC_VOTER		"FCC_VOTER"
#define CEP_VOTER		"CEP_VOTER"
#define QUIET_VOTER		"QUIET_VOTER"
#define BATT_VOL_VOTER		"BATT_VOL_VOTER"
#define BATT_CURR_VOTER		"BATT_CURR_VOTER"
#define IOUT_CURR_VOTER		"IOUT_CURR_VOTER"
#define DEEP_RATIO_LIMIT_VOTER	"DEEP_RATIO_LIMIT_VOTER"
#define SKIN_VOTER		"SKIN_VOTER"
#define STARTUP_CEP_VOTER	"STARTUP_CEP_VOTER"
#define HW_ERR_VOTER		"HW_ERR_VOTER"
#define CURR_ERR_VOTER		"CURR_ERR_VOTER"
#define OTG_EN_VOTER		"OTG_EN_VOTER"
#define TRX_EN_VOTER		"TRX_EN_VOTER"
#define UPGRADE_FW_VOTER	"UPGRADE_FW_VOTER"
#define DEBUG_VOTER		"DEBUG_VOTER"
#define FFC_VOTER		"FFC_VOTER"
#define USB_VOTER		"USB_VOTER"
#define ADAPTER_RESET_VOTER	"ADAPTER_RESET_VOTER"
#define UPGRADE_VOTER		"UPGRADE_VOTER"
#define CONNECT_VOTER		"CONNECT_VOTER"
#define UOVP_VOTER		"UOVP_VOTER"
#define STOP_VOTER		"STOP_VOTER"
#define FTM_TEST_VOTER		"FTM_TEST_VOTER"
#define CAMERA_VOTER		"CAMERA_VOTER"
#define CALL_VOTER		"CALL_VOTER"
#define COOL_DOWN_VOTER		"COOL_DOWN_VOTER"
#define BCC_CURRENT_VOTER	"BCC_CURRENT_VOTER"
#define RX_IIC_VOTER		"RX_IIC_VOTER"
#define TIMEOUT_VOTER		"TIMEOUT_VOTER"
#define CHG_DONE_VOTER		"CHG_DONE_VOTER"
#define RX_FAST_ERR_VOTER	"RX_FAST_ERR_VOTER"
#define VERITY_VOTER		"VERITY_VOTER"
#define NON_FFC_VOTER		"NON_FFC_VOTER"
#define CV_VOTER		"CV_VOTER"
#define RXAC_VOTER		"RXAC_VOTER"
#define WLS_ONLINE_VOTER	"WLS_ONLINE_VOTER"
#define EPP_CURVE_VOTER		"EPP_CURVE_VOTER"
#define SPEC_VOTER		"SPEC_VOTER"
#define BOOST_CV_VOTER		"BOOST_CV_VOTER"
#define FV_MAX_VOTER		"FV_MAX_VOTER"
#define OVER_FV_VOTER		"OVER_FV_VOTER"
#define CHG_FULL_VOTER		"CHG_FULL_VOTER"
#define CHG_FULL_WARM_VOTER	"CHG_FULL_WARM_VOTER"
#define CHG_FULL_COOL_VOTER	"CHG_FULL_COOL_VOTER"
#define WARM_FULL_VOTER		"WARM_FULL_VOTER"
#define SWITCH_RANGE_VOTER	"SWITCH_TEMP_RANGE"
#define ADSP_CRASH_VOTER	"ADSP_CRASH_VOTER"
#define NON_STANDARD_VOTER	"NON_STANDARD_VOTER"
#define BATT_TEMP_VOTER		"BATT_TEMP_VOTER"
#define CURR_LIMIT_VOTER	"CURR_LIMIT_VOTER"
#define BATT_SOC_VOTER		"BATT_SOC_VOTER"
#define WARM_SOC_VOTER		"WARM_SOC_VOTER"
#define WARM_VOL_VOTER		"WARM_VOL_VOTER"
#define FASTCHG_VOTER		"FASTCHG_VOTER"
#define FASTCHG_DUMMY_VOTER	"FASTCHG_DUMMY_VOTER"
#define MMI_CHG_VOTER		"MMI_CHG_VOTER"
#define CHG_LIMIT_CHG_VOTER	"CHG_LIMIT_CHG_VOTER"
#define HIDL_VOTER		"HIDL_VOTER"
#define SVID_VOTER		"SVID_VOTER"
#define FACTORY_TEST_VOTER	"FACTORY_TEST_VOTER"
#define BTB_TEMP_OVER_VOTER	"BTB_TEMP_OVER_VOTER"
#define STRATEGY_VOTER		"STRATEGY_VOTER"
#define BAD_VOLT_VOTER		"BAD_VOLT_VOTER"
#define WLAN_VOTER		"WLAN_VOTER"
#define LED_ON_VOTER		"LED_ON_VOTER"
#define BAD_CONNECTED_VOTER	"BAD_CONNECTED"
#define BCC_VOTER		"BCC_VOTER"
#define EIS_VOTER		"EIS_VOTER"
#define PDQC_VOTER		"PDQC_VOTER"
#define TYPEC_VOTER		"TYPEC_VOTER"
#define VOL_DIFF_VOTER		"VOL_DIFF_VOTER"
#define CHARGE_SUSPEND_VOTER	"CHARGE_SUSPEND_VOTER"
#define CHAEGE_DISABLE_VOTER	"CHAEGE_DISABLE_VOTER"
#define OTG_ONLINE_VOTER	"OTG_ONLINE_VOTER"
#define COPYCAT_ADAPTER		"COPYCAT_ADAPTER"
#define PARALLEL_VOTER		"PARALLEL_VOTER"
#define SHELL_TEMP_VOTER	"SHELL_TEMP_VOTER"
#define SLOW_CHG_VOTER		"SLOW_CHG_VOTER"
#define SVOOC_CURR_OCP_VOTER	"SVOOC_CURR_OCP_VOTER"
#define UFCS_VOTER		"UFCS_VOTER"
#define TFG_VOTER		"TFG_VOTER"
#define NO_DATA_VOTER		"NO_DATA_VOTER"
#define IBAT_OVER_VOTER		"IBAT_OVER_VOTER"
#define IMP_VOTER		"IMP_VOTER"
#define AUTH_VOTER		"AUTH_VOTER"
#define BAD_SUBBOARD_VOTER	"BAD_SUBBOARD_VOTER"
#define PPS_VOTER		"PPS_VOTER"
#define PPS_IBAT_ABNOR_VOTER	"PPS_IBAT_ABNOR_VOTER"
#define PD_PDO_ICL_VOTER 	"PD_PDO_ICL_VOTER"
#define USB_IBUS_DRAW_VOTER 	"USB_IBUS_DRAW_VOTER"
#define TCPC_IBUS_DRAW_VOTER 	"TCPC_IBUS_DRAW_VOTER"
#define SHUTDOWN_VOTER		"SHUTDOWN_VOTER"
#define CYCLE_COUNT_VOTER	"CYCLE_COUNT_VOTER"
#define READY_VOTER		"READY_VOTER"
#define SUPER_ENDURANCE_MODE_VOTER	"SUPER_ENDURANCE_MODE_VOTER"
#define DEEP_COUNT_VOTER	"DEEP_COUNT_VOTER"
#define SUB_DEEP_COUNT_VOTER	"SUB_DEEP_COUNT_VOTER"
#define BOOST_VOTER		"BOOST_VOTER"
#define BOOST_RUS_VOTER		"BOOST_RUS_VOTER"
#define BOOST_SOC_VOTER		"BOOST_SOC_VOTER"
#define BOOST_ERROR_VOTER	"BOOST_ERROR_VOTER"
#define CP_ERR_VOTER		"CP_ERR_VOTER"
#define CP_OFFLINE_VOTER	"CP_OFFLINE_VOTER"
#define BAL_STATE_VOTER		"BAL_STATE_VOTER"
#define BATT_BAL_VOTER		"BATT_BAL_VOTER"
#define IBUS_OVER_VOTER		"IBUS_OVER_VOTER"
#define ADAPTER_IMAX_VOTER	"ADAPTER_IMAX_VOTER"
#define SALE_MODE_VOTER 	"SALE_MODE_VOTER"
#define RX_ADAPTER_CURVE_VOTER	"RX_ADAPTER_CURVE_VOTER"
#define WLS_PLOSS_WARN_VOTER	"WLS_PLOSS_WARN_VOTER"
#define WLS_TA_UV_VOTER		"WLS_TA_UV_VOTER"
#define WLS_QUIET_MODE_VOTER	"WLS_QUIET_MODE_VOTER"
#define WLS_BOOT_QUIET_VOTER	"WLS_BOOT_QUIET_VOTER"
#define WLS_AC_OV_QUIET_VOTER	"WLS_AC_OV_QUIET_VOTER"
#define WLS_AUDIO_MODE_VOTER	"WLS_AUDIO_MODE_VOTER"
#define WLS_CAMERA_MODE_VOTER	"WLS_CAMERA_MODE_VOTER"
#define WLS_FORCE_EPP_TO_BPP_VOTER	"WLS_FORCE_EPP_TO_BPP_VOTER"
#define WLS_TX_VOTER		"WLS_TX_VOTER"
#define WLS_Q_VALUE_ERROR_VOTER	"WLS_Q_VALUE_ERROR_VOTER"
#define BATT_FG_I2CREST_VOTER	"BATT_FG_I2CREST_VOTER"
#define USB_ENHANCE_VOTER	"USB_ENHANCE_VOTER"
#define BAD_SUB_BTB_VOTER	"BAD_SUB_BTB_VOTER"
#define IC_VOTER 		"IC_VOTER"
#define PARENT_VOTER		"PARENT_VOTER"
#define UCP_VOTER		"UCP_VOTER"
#define OCP_VOTER		"OCP_VOTER"
#define IIC_VOTER		"IIC_VOTER"
#define WLS_TIMEOUT_VOTER	"WLS_TIMEOUT_VOTER"
#define COMMON_POWER_CHECK	"COMMON_POWER_CHECK"
#define FLASH_MODE_VOTER	"FLASH_MODE_VOTER"
#define LIMIT_FCL_VOTER		"LIMIT_FCL_VOTER"
#define STATE_KEEP_VOTER	"STATE_KEEP_VOTER"
#define REC_VDD_VOTER		"REC_VDD_VOTER"
#define DUAL_CELLS_PROTECTION_VOTER	  "DUAL_CELLS_PROTECTION_VOTER"
#define VBUS_MV_VOTER		"VBUS_MV_VOTER"
#define VFA_VOTER		"VFA_VOTER"
#define FCS_ICL_VOTER		"FCS_ICL_VOTER"
#define SVOOC_SUSPEND_ICL_VOTER	"SVOOC_SUSPEND_ICL_VOTER"

/* TOPIC voter */
#define COMM_TOPIC_VOTER	"COMM_TOPIC_VOTER"
#define WIRED_TOPIC_VOTER	"WIRED_TOPIC_VOTER"
#define GAUGE_TOPIC_VOTER	"GAUGE_TOPIC_VOTER"
#define WLS_TOPIC_VOTER		"WLS_TOPIC_VOTER"
#define ERR_TOPIC_VOTER		"ERR_TOPIC_VOTER"
#define VOOC_TOPIC_VOTER	"VOOC_TOPIC_VOTER"
#define UFCS_TOPIC_VOTER	"UFCS_TOPIC_VOTER"
#define CPA_TOPIC_VOTER		"CPA_TOPIC_VOTER"
#define USB_PSY_VOTER		"USB_PSY_VOTER"

bool is_client_vote_enabled(struct votable *votable, const char *client_str);
bool is_client_vote_enabled_locked(struct votable *votable,
							const char *client_str);
bool is_override_vote_enabled(struct votable *votable);
bool is_override_vote_enabled_locked(struct votable *votable);
int get_client_vote(struct votable *votable, const char *client_str);
int get_client_vote_locked(struct votable *votable, const char *client_str);
int get_effective_result(struct votable *votable);
int get_effective_result_locked(struct votable *votable);
int get_effective_result_exclude_client(struct votable *votable, const char *exclude_str);
int get_effective_result_exclude_client_locked(struct votable *votable, const char *exclude_str);
const char *get_effective_client(struct votable *votable);
const char *get_effective_client_locked(struct votable *votable);
int vote(struct votable *votable, const char *client_str, bool state, int val, bool step);
int vote_override(struct votable *votable, const char *override_client,
		  bool state, int val, bool step);
int rerun_election(struct votable *votable, bool step);
int rerun_election_unlock(struct votable *votable, bool step);
struct votable *find_votable(const char *name);
struct votable *create_votable(const char *name,
				int votable_type,
				int (*callback)(struct votable *votable,
						void *data,
						int effective_result,
						const char *effective_client,
						bool step),
				void *data);
int votable_add_map(struct votable *votable, int original, int mapped);
int votable_add_check_func(struct votable *votable,
	int (*func)(struct votable *votable,
		void *data, const char *client_str,
		bool enabled, int val, bool step));
void destroy_votable(struct votable *votable);
void lock_votable(struct votable *votable);
void unlock_votable(struct votable *votable);
const char *votable_name(struct votable *votable);

#endif /* __PMIC_VOTER_H */
