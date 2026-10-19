// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef __SM8350_CHARGER_H
#define __SM8350_CHARGER_H

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
#include <linux/soc/qcom/qti_pmic_glink.h>
#else
#include <linux/soc/qcom/pmic_glink.h>
#endif
#include <linux/soc/qcom/battery_charger.h>
#include <linux/regmap.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_pps.h>
#include <oplus_reverse_chg.h>

#ifdef OPLUS_FEATURE_CHG_BASIC
#include <oplus_mms_gauge.h>
#ifndef CONFIG_FB
#define CONFIG_FB
#endif
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
#define OEM_OPCODE_READ_BUFFER    0x10000
#define BCC_OPCODE_READ_BUFFER    0x10003
#define PPS_OPCODE_READ_BUFFER    0x10004
#define AP_OPCODE_UFCS_BUFFER     0x10005
#define AP_OPCODE_READ_BUFFER     0x10006
#define AP_OPCODE_WRITE_BUFFER    0x10008
#define OPLUS_OPCODE_GET_SINK_MSG 0x10009
#define AP_OPCODE_PD_INFO_BUFFER  0x1000A
#define OEM_READ_WAIT_TIME_MS    500
#define MAX_OEM_PROPERTY_DATA_SIZE 128
#define AP_READ_WAIT_TIME_MS      1000
#define MAX_AP_PROPERTY_DATA_SIZE 512
#define AP_UFCS_WAIT_TIME_MS      500
#define MAX_UFCS_CAPS_ITEM        16
#define MAX_REVERSE_CHG_MSG_ITEM  16
#define MAX_PD_INFO_MSG_SIZE_MAX  8
#endif

#define MSG_OWNER_BC			32778
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2

/* opcode for battery charger */
#define BC_SET_NOTIFY_REQ		0x04
#define BC_NOTIFY_IND			0x07
#define BC_BATTERY_STATUS_GET		0x30
#define BC_BATTERY_STATUS_SET		0x31
#define BC_USB_STATUS_GET		0x32
#define BC_USB_STATUS_SET		0x33
#define BC_WLS_STATUS_GET		0x34
#define BC_WLS_STATUS_SET		0x35
#define BC_SHIP_MODE_REQ_SET		0x36
#define BC_WLS_FW_CHECK_UPDATE		0x40
#define BC_WLS_FW_PUSH_BUF_REQ		0x41
#define BC_WLS_FW_UPDATE_STATUS_RESP	0x42
#define BC_WLS_FW_PUSH_BUF_RESP		0x43
#define BC_WLS_FW_GET_VERSION		0x44
#define BC_SHUTDOWN_NOTIFY		0x47
#define BC_GENERIC_NOTIFY		0x80

#ifdef OPLUS_FEATURE_CHG_BASIC
#define BC_VOOC_STATUS_GET		0X48
#define BC_VOOC_STATUS_SET		0X49
#define BC_OTG_ENABLE			0x50
#define BC_OTG_DISABLE			0x51
#define BC_VOOC_VBUS_ADC_ENABLE		0x52
#define BC_CID_DETECT			0x53
#define BC_QC_DETECT			0x54
#define BC_TYPEC_STATE_CHANGE		0x55
#define BC_PD_SVOOC			0x56
#define BC_PLUGIN_IRQ 			0x57
#define BC_APSD_DONE			0x58
#define BC_CHG_STATUS_GET		0x59
#define BC_PD_SOFT_RESET		0x5A
#define BC_CHG_STATUS_SET		0x60
#define BC_ADSP_NOTIFY_AP_SUSPEND_CHG	0X61
#define BC_ADSP_NOTIFY_AP_CP_BYPASS_INIT	0x0062
#define BC_ADSP_NOTIFY_AP_CP_MOS_ENABLE		0x0063
#define BC_ADSP_NOTIFY_AP_CP_MOS_DISABLE	0x0064
#define BC_PPS_OPLUS				0x65
#define BC_ADSP_NOTIFY_TRACK			0x66
#define BC_UFCS_TEST_MODE_TRUE		0X68
#define BC_UFCS_TEST_MODE_FALSE		0X69
#define BC_UFCS_POWER_READY		0X70
#define BC_UFCS_HANDSHAKE_OK		0X71
#define BC_VOOC_GAN_MOS_ERROR	   0X72
#define BC_UFCS_DISABLE_MOS		0X73
#define BC_UFCS_PDO_READY		0X74
#define BC_UFCS_VERIFY_AUTH_READY	0X75
#define BC_UFCS_VDM_EMARK_READY		0X76
#define BC_UFCS_PWR_INFO_READY		0X77
#define BC_BATTERY_RESET_START		0X78
#define PD_SOURCECAP_DONE		0X79
#define REQUEST_QOS			0X7a
#define RELEASE_QOS			0X7b
#define HMAC_UPDATE			0X7c
#define BC_POWER_ROLE_STATUS		0X7d
#define UFCS_EXIT_MODE_NOTIFY		0X7e
#define PD_CONNECT_HARD_RESET		0x7f
#define GAUGE_INITED			0X90
#define PD_PPS_CHECK_COMPLETED		0x91
#define PD_VDM_INFO_READY		0x92
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
#define WLS_BOOST_VOL_MIN_MV 4800
#endif

/* Generic definitions */
#define MAX_STR_LEN			128
#define BC_WAIT_TIME_MS			2500/* sjc 1K->2K */
#define WLS_FW_PREPARE_TIME_MS		300
#define WLS_FW_WAIT_TIME_MS		500
#define WLS_FW_BUF_SIZE			128
#define DEFAULT_RESTRICT_FCC_UA		1000000
#define OPLUS_VDM_INFO_MAX		5

#ifdef OPLUS_FEATURE_CHG_BASIC
struct oem_read_buffer_req_msg {
	struct pmic_glink_hdr hdr;
	u32 data_size;
};

struct oem_read_buffer_resp_msg {
	struct pmic_glink_hdr hdr;
	u32 data_buffer[MAX_OEM_PROPERTY_DATA_SIZE];
	u32 data_size;
};

struct oplus_ap_read_ufcs_req_msg {
	struct pmic_glink_hdr hdr;
	u32 data_size;
	u32 msg_id;
};

struct oplus_ap_read_ufcs_resp_msg {
	struct pmic_glink_hdr hdr;
	u64 data_buffer[MAX_UFCS_CAPS_ITEM];
	u32 data_size;
	u32 msg_id;
};

struct oplus_ap_read_pd_info_msg {
	struct pmic_glink_hdr hdr;
	u32 data_buffer[MAX_PD_INFO_MSG_SIZE_MAX];
	u32 data_size;
	u32 msg_id;
};

struct oplus_ap_read_req_msg {
	struct pmic_glink_hdr hdr;
	u32 message_id;
	u32 value;
};

struct oplus_ap_read_buffer_resp_msg {
	struct pmic_glink_hdr hdr;
	u32 message_id;
	u8 data_buffer[MAX_AP_PROPERTY_DATA_SIZE];
	u32 data_size;
};

enum oplus_ap_message_id {
	AP_MESSAGE_ACK,
	AP_MESSAGE_GET_GAUGE_REG_INFO,
	AP_MESSAGE_GET_GAUGE_CALIB_TIME,
	AP_MESSAGE_GET_GAUGE_BATTINFO,
	AP_MESSAGE_GET_LPD_INFO,
	AP_MESSAGE_GET_GAUGE_LIFETIME_STATUS,
	AP_MESSAGE_GET_GAUGE_LIFETIME_INFO,
	AP_MESSAGE_GET_GAUGE_R_INFO,
	AP_MESSAGE_GET_GAUGE_THREE_LEVEL_TERM_VOLT,
	AP_MESSAGE_GET_GAUGE_RA0_INFO,
	AP_MESSAGE_GET_GAUGE_IMP_INFO,
	AP_MESSAGE_GET_GAUGE_DELTA_VOLTAGE_INFO,
	AP_MESSAGE_MAX_SIZE = 32,
};

struct oplus_ap_write_req_msg {
	struct pmic_glink_hdr hdr;
	u32 message_id;
	u8 data_buffer[MAX_AP_PROPERTY_DATA_SIZE];
	u32 data_size;
};

struct oplus_ap_write_buffer_resp_msg {
	struct pmic_glink_hdr hdr;
	u32 message_id;
	int ret;
};

enum oplus_ap_write_message_id {
	AP_MESSAGE_WRITE_CALIB_TIME,
	AP_MESSAGE_WRITE_THREE_LEVEL_TERM_VOLT,
};
#endif

enum lcm_en_status {
	LCM_EN_DEAFULT = 1,
	LCM_EN_ENABLE,
	LCM_EN_DISABLE,
};

enum psy_type {
	PSY_TYPE_BATTERY,
	PSY_TYPE_USB,
	PSY_TYPE_WLS,
	PSY_TYPE_MAX,
};

enum ship_mode_type {
	SHIP_MODE_PMIC,
	SHIP_MODE_PACK_SIDE,
};

typedef enum {
	DOUBLE_SERIES_WOUND_CELLS = 0,
	SINGLE_CELL,
	DOUBLE_PARALLEL_WOUND_CELLS,
} SCC_CELL_TYPE;

typedef enum {
	TI_GAUGE = 0,
	SW_GAUGE,
	UNKNOWN_GAUGE_TYPE,
} SCC_GAUGE_TYPE;

/* property ids */
enum battery_property_id {
	BATT_STATUS,
	BATT_HEALTH,
	BATT_PRESENT,
	BATT_CHG_TYPE,
	BATT_CAPACITY,
	BATT_SOH,
	BATT_VOLT_OCV,
	BATT_VOLT_NOW,
	BATT_VOLT_MAX,
	BATT_CURR_NOW,
	BATT_CHG_CTRL_LIM,
	BATT_CHG_CTRL_LIM_MAX,
	BATT_TEMP,
	BATT_TECHNOLOGY,
	BATT_CHG_COUNTER,
	BATT_CYCLE_COUNT,
	BATT_CHG_FULL_DESIGN,
	BATT_CHG_FULL,
	BATT_MODEL_NAME,
	BATT_TTF_AVG,
	BATT_TTE_AVG,
	BATT_RESISTANCE,
	BATT_POWER_NOW,
	BATT_POWER_AVG,
#ifdef OPLUS_FEATURE_CHG_BASIC
	BATT_CHG_EN,/* sjc add */
	BATT_SET_PDO,/* sjc add */
	BATT_SET_QC,/* sjc add */
	BATT_SET_SHIP_MODE,/*sjc add*/
	BATT_SET_COOL_DOWN,/*lzj add*/
	BATT_SET_MATCH_TEMP,/*lzj add*/
	BATT_BATTERY_AUTH,/*lzj add*/
	BATT_RTC_SOC,/*lzj add*/
	BATT_UEFI_INPUT_CURRENT,
	BATT_UEFI_PRE_CHG_CURRENT,
	BATT_UEFI_UEFI_CHG_EN,
	BATT_UEFI_LOAD_ADSP,
	BATT_UEFI_SET_VSYSTEM_MIN,
	BATT_SEND_CHG_STATUS,
	BATT_ADSP_GAUGE_INIT,
	BATT_UPDATE_SOC_SMOOTH_PARAM,
	BATT_BATTERY_HMAC,
	BATT_SET_BCC_CURRENT,
	BATT_ZY0603_CHECK_RC_SFR,
	BATT_ZY0603_SOFT_RESET,
	BATT_AFI_UPDATE_DONE,
	BATT_UI_SOC,
	BATT_AP_FASTCHG_ALLOW,
	BATT_SET_VOOC_CURVE_NUM,
	BATT_BAT_FULL_CURR_SET,
	BATT_DEEP_DISCHG_COUNT,
	BATT_DEEP_TERM_VOLT,
	BATT_SET_FIRST_USAGE_DATE,
	BATT_SET_UI_CYCLE_COUNT,
	BATT_SET_UI_SOH,
	BATT_SET_USED_FLAG,
	BATT_DEEP_DISCHG_LAST_CC,
	BATT_GET_UFCS_RUNNING_STATE,
	BATT_VOLT_MIN,
	BATT_SET_CHG_PATH,
	BATT_GET_CHG_PATH_STATUS,
	BATT_GET_CAR_C,
	BATT_SET_CAR_C_CLEAR,
	BATT_GET_VCT,
	BATT_SET_VCT,
	BATT_SET_BATT_FULL,
	BATT_SET_CUV_STATE,
	BATT_GET_CUV_STATE,
	BATT_ITERM_CHECK_STAT,
	BATT_ITERM_TIMEOUT,
	BATT_SET_TRUE_FCC,
#endif
	BATT_PROP_MAX,
};

enum usb_property_id {
	USB_ONLINE,
	USB_VOLT_NOW,
	USB_VOLT_MAX,
	USB_CURR_NOW,
	USB_CURR_MAX,
	USB_INPUT_CURR_LIMIT,
	USB_TYPE,
	USB_ADAP_TYPE,
	USB_MOISTURE_DET_EN,
	USB_MOISTURE_DET_STS,
	USB_TEMP,
	USB_REAL_TYPE,
	USB_TYPEC_COMPLIANT,
#ifdef OPLUS_FEATURE_CHG_BASIC
	USB_ADAP_SUBTYPE,
	USB_VBUS_COLLAPSE_STATUS,
	USB_VOOCPHY_STATUS,
	USB_VOOCPHY_ENABLE,
	USB_OTG_AP_ENABLE,
	USB_OTG_SWITCH,
	USB_POWER_SUPPLY_RELEASE_FIXED_FREQUENCE,
	USB_TYPEC_CC_ORIENTATION,
	USB_CID_STATUS,
	USB_TYPEC_MODE,
	USB_TYPEC_SINKONLY,
	USB_OTG_VBUS_REGULATOR_ENABLE,
	USB_VOOC_CHG_PARAM_INFO,
	USB_VOOC_FAST_CHG_TYPE,
	USB_DEBUG_REG,
	USB_VOOCPHY_RESET_AGAIN,
	USB_SUSPEND_PMIC,
	USB_OEM_MISC_CTL,
	USB_CCDETECT_HAPPENED,
	USB_GET_PPS_TYPE,
	USB_GET_PPS_STATUS,
	USB_SET_PPS_VOLT,
	USB_SET_PPS_CURR,
	USB_GET_PPS_MAX_CURR,
	USB_PPS_READ_VBAT0_VOLT,
	USB_PPS_CHECK_BTB_TEMP,
	USB_PPS_MOS_CTRL,
	USB_PPS_CP_MODE_INIT,
	USB_PPS_CHECK_AUTHENTICATE,
	USB_PPS_GET_AUTHENTICATE,
	USB_PPS_GET_CP_VBUS,
	USB_PPS_GET_CP_MASTER_IBUS,
	USB_PPS_GET_CP_SLAVE_IBUS,
	USB_PPS_MOS_SLAVE_CTRL,
	USB_PPS_GET_R_COOL_DOWN,
	USB_PPS_GET_DISCONNECT_STATUS,
	USB_PPS_VOOCPHY_ENABLE,
	USB_IN_STATUS,
	USB_GET_BATT_CURR,
	USB_PPS_FORCE_SVOOC,
	USB_SET_OVP_CFG,
	USB_SET_UFCS_START,
	USB_SET_UFCS_VOLT,
	USB_SET_UFCS_CURRENT,
	USB_GET_UFCS_STATUS,
	USB_GET_DEV_INFO_L,
	USB_GET_DEV_INFO_H,
	USB_SET_WD_TIME,
	USB_SET_EXIT,
	USB_GET_SRC_INFO_L,
	USB_GET_SRC_INFO_H,
	USB_OTG_BOOST_CURRENT,
	USB_SNS_STATUS,
	USB_SET_UFCS_SM_PERIOD,
	USB_SET_RERUN_AICL,
	USB_SET_AICL_VOL,
	USB_GET_AICL_VOL,
	USB_SET_PLC_STATUS,
	USB_GET_POWER_ROLE,
	USB_REVERSE_CHG_SET_VOLT,
	USB_REVERSE_CHG_SET_CURRENT,
	USB_RVS_HIGH_MODE_EN,
	USB_SET_WIRED_USB_STATUS,
	/* PD partner SVID (lower 16 bits valid) */
	USB_ADAPTER_SVID,
	USB_PD_SEND_GET_SINK_CAP,
#endif /*OPLUS_FEATURE_CHG_BASIC*/
	USB_PROP_MAX,
};

enum wireless_property_id {
	WLS_ONLINE,
	WLS_VOLT_NOW,
	WLS_VOLT_MAX,
	WLS_CURR_NOW,
	WLS_CURR_MAX,
	WLS_TYPE,
	WLS_BOOST_EN,
	WLS_HBOOST_VMAX,
	WLS_INPUT_CURR_LIMIT,
	WLS_ADAP_TYPE,
	WLS_CONN_TEMP,
#ifdef OPLUS_FEATURE_CHG_BASIC
	WLS_BOOST_VOLT = 12,
	WLS_BOOST_AICL_ENABLE,
	WLS_BOOST_AICL_RERUN,
	WLS_BOOST_CURRENT,
#endif
	WLS_PROP_MAX,
};

enum ufcs_read_msg_id {
	UFCS_PDO_INFO,
	UFCS_VDM_PWR_INFO,
	UFCS_VDM_EMARK_INFO,
	UFCS_ADAPTER_VERIFY,
};

enum oplus_pd_info_msg_id {
	OPLUS_PD_INFO_MSG_ID_VDM_ID,
	OPLUS_PD_INFO_MSG_ID_MAX,
    };

enum {
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP = 0x80,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5,
};

enum oplus_power_supply_usb_type {
	POWER_SUPPLY_USB_TYPE_PD_SDP = 17,
};

enum OTG_SCHEME {
	OTG_SCHEME_CID,
	OTG_SCHEME_CCDETECT_GPIO,
	OTG_SCHEME_SWITCH,
	OTG_SCHEME_UNDEFINE,
};

enum OTG_BOOST_SOURCE {
	OTG_BOOST_SOURCE_PMIC,
	OTG_BOOST_SOURCE_EXTERNAL,
};

enum WLS_BOOST_SOURCE {
	WLS_BOOST_SOURCE_PMIC_OTG,
	WLS_BOOST_SOURCE_PMIC_WLS,
};

enum charging_status {
	CHARGING_TYPE_UNKNOW,
	CHARGING_TYPE_VOOC_SVOOC,
	CHARGING_TYPE_OPLUS_UFCS,
	CHARGING_TYPE_OPLUS_PPS,
	CHARGING_TYPE_THIRD_UFCS,
	CHARGING_TYPE_THIRD_PPS,
	CHARGING_TYPE_FFC,
	CHARGING_TYPE_MAX,
};

enum qbg_full_temp_region {
	QBG_TEMP_COLD,
	QBG_TEMP_COOL,
	QBG_TEMP_NORMAL,
	QBG_TEMP_WARM,
	QBG_TEMP_MAX,
};

enum OEM_MISC_CTL_CMD {
	OEM_MISC_CTL_CMD_LCM_EN = 0,
	OEM_MISC_CTL_CMD_LCM_25K = 2,
	OEM_MISC_CTL_CMD_NCM_AUTO_MODE = 4,
	OEM_MISC_CTL_CMD_VPH_TRACK_HIGH = 6,
};

typedef enum _QCOM_PM_TYPEC_PORT_ROLE_TYPE
{
	QCOM_TYPEC_PORT_ROLE_DRP,
	QCOM_TYPEC_PORT_ROLE_SNK,
	QCOM_TYPEC_PORT_ROLE_SRC,
	QCOM_TYPEC_PORT_ROLE_DISABLE,
	QCOM_TYPEC_PORT_ROLE_INVALID,
} QCOM_PM_TYPEC_PORT_ROLE_TYPE;

struct battery_charger_set_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			power_state;
	u32			low_capacity;
	u32			high_capacity;
};

struct battery_charger_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
};

struct battery_charger_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			property_id;
	u32			value;
};

struct battery_charger_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			value;
	u32			ret_code;
};

struct battery_model_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	char			model[MAX_STR_LEN];
};

struct wireless_fw_check_req {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
	u32			fw_size;
	u32                     fw_crc;
};

struct wireless_fw_check_resp {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};

struct wireless_fw_push_buf_req {
	struct pmic_glink_hdr	hdr;
	u8			buf[WLS_FW_BUF_SIZE];
	u32			fw_chunk_id;
};

struct wireless_fw_push_buf_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_status;
};

struct wireless_fw_update_status {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_done;
};

struct wireless_fw_get_version_req {
	struct pmic_glink_hdr	hdr;
};

struct wireless_fw_get_version_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
};

struct battery_charger_ship_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			ship_mode_type;
};

enum reserve_chg_msg_id {
	RESERVED_CHG_MSG_GET_SINK_PDO,
	RESERVED_CHG_MSG_ID_MAX,
};
struct reverse_chg_msg {
	struct pmic_glink_hdr	hdr;
	u32 data_buffer[MAX_REVERSE_CHG_MSG_ITEM];
	u32 data_size;
	u32 msg_id;
};

#ifdef OPLUS_FEATURE_CHG_BASIC
#define ADAPTER_VERIFY_AUTH_DATA_SIZE 16
struct adapter_verify_req_msg {
	struct oplus_ap_read_ufcs_req_msg ufcs_req;
	u8 key_index;
	u32 auth_data_size;
	u8 auth_data[ADAPTER_VERIFY_AUTH_DATA_SIZE];
};
#endif

struct psy_state {
	struct power_supply	*psy;
	char			*model;
	const int		*map;
	u32			*prop;
	u32			prop_count;
	u32			opcode_get;
	u32			opcode_set;
};

#ifdef OPLUS_FEATURE_CHG_BASIC
struct oplus_custom_gpio_pinctrl {
	int vchg_trig_gpio;
	int otg_boost_en_gpio;
	int otg_ovp_en_gpio;
	int tx_boost_en_gpio;
	int tx_ovp_en_gpio;
	int wrx_ovp_off_gpio;
	int wrx_otg_en_gpio;
	int supplementary_power_mos_gpio;
	struct mutex pinctrl_mutex;
	struct pinctrl *vchg_trig_pinctrl;
	struct pinctrl_state *vchg_trig_default;
	struct pinctrl *usbtemp_l_gpio_pinctrl;
	struct pinctrl_state *usbtemp_l_gpio_default;
	struct pinctrl *usbtemp_r_gpio_pinctrl;
	struct pinctrl *subboard_temp_gpio_pinctrl;
	struct pinctrl_state *subboard_temp_gpio_default;
	struct pinctrl *btb_temp_gpio_pinctrl;
	struct pinctrl_state *btb_temp_gpio_default;
	struct pinctrl_state *usbtemp_r_gpio_default;
	struct pinctrl *otg_boost_en_pinctrl;
	struct pinctrl_state *otg_boost_en_active;
	struct pinctrl_state *otg_boost_en_sleep;
	struct pinctrl *otg_ovp_en_pinctrl;
	struct pinctrl_state *otg_ovp_en_active;
	struct pinctrl_state *otg_ovp_en_sleep;
	struct pinctrl *tx_boost_en_pinctrl;
	struct pinctrl_state *tx_boost_en_active;
	struct pinctrl_state *tx_boost_en_sleep;
	struct pinctrl *tx_ovp_en_pinctrl;
	struct pinctrl_state *tx_ovp_en_active;
	struct pinctrl_state *tx_ovp_en_sleep;
	struct pinctrl *wrx_ovp_off_pinctrl;
	struct pinctrl_state *wrx_ovp_off_active;
	struct pinctrl_state *wrx_ovp_off_sleep;
	struct pinctrl *wrx_otg_en_pinctrl;
	struct pinctrl_state *wrx_otg_en_active;
	struct pinctrl_state *wrx_otg_en_sleep;
	struct pinctrl *supplementary_power_pinctrl;
	struct pinctrl_state *supplementary_power_mos_active;
	struct pinctrl_state *supplementary_power_mos_sleep;
};

#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
struct oplus_chg_iio {
	struct iio_channel	*usbtemp_v_chan;
	struct iio_channel	*usbtemp_sup_v_chan;
	struct iio_channel	*subboard_temp_chan;
	struct iio_channel	*batt_con_btb_chan;
	struct iio_channel	*sub_batt_con_btb_chan;
	struct iio_channel	*usb_con_btb_chan;
	struct iio_channel	*chg_mos_temp_chan;
	struct iio_channel	*sub_con_btb_detect_adc;
	struct iio_channel	*vph_pwr_chan;
	struct iio_channel	*vbat_sns_qbg_chan;
	struct iio_channel	*pmic_vbat_adc;
	struct iio_channel	*shaft_btb_temp_chan;
};

enum oplus_sub_btb_adc_index {
	OPLUS_SUB_BTB_VALD_MIN_ADC,
	OPLUS_SUB_BTB_VALD_MAX_ADC,
	OPLUS_SUB_BTB_MAX,
};

#define OPLUS_CHG_TRACK_PLAT_CALI_INFO_LEN	300

enum oplus_gauge_track_type {
	GAUGE_TRACK_CALI_FLAG_ZCV = 1,
	GAUGE_TRACK_CALI_FLAG_AGING = 2,
	GAUGE_TRACK_CALI_FLAG_PLUGOUT = 4,
	GAUGE_TRACK_CALI_FLAG_CHG_FULL = 5
};

struct gauge_track_cali_info_s {
	int vbat;
	int tbat;
	int soc;
	int cycle_count;
	int learn_capacity;
	int imp;
	int soh;
};
#endif

#define AICL_POINT_INDEX_MAX 4
struct aicl_threshold {
	int vbat_thr;
	int hw_aicl;
	int sw_aicl;
};

struct battery_chg_dev {
	struct device			*dev;
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct oplus_chg_ic_dev		*buck_ic;
	struct oplus_chg_ic_dev		*gauge_ic;
	struct oplus_chg_ic_dev		*cp_ic;
	struct oplus_chg_ic_dev		*misc_ic;
	struct oplus_chg_ic_dev		*pps_ic;
	struct oplus_chg_ic_dev *reverse_chg_ic_dev;
	struct oplus_mms		*vooc_topic;
	struct oplus_mms		*cpa_topic;
	struct oplus_chg_ic_dev		*ufcs_ic;
	struct oplus_impedance_node	*input_imp_node;
	struct oplus_mms		*common_topic;
	struct oplus_mms		*pps_topic;
	struct oplus_mms		*ufcs_topic;
	struct oplus_mms		*gauge_topic;
	struct oplus_mms		*wls_topic;
	struct oplus_mms		*err_topic;
	struct oplus_mms		*plc_topic;
	struct mms_subscribe		*plc_subs;
	struct oplus_mms		*wired_topic;
	struct mms_subscribe		*wired_subs;
	struct votable			*chg_disable_votable;
	struct mutex			chg_en_lock;
	bool 				    chg_en;
	struct psy_state		oplus_psy;
#endif
	struct class			battery_class;
	struct pmic_glink_client	*client;
	struct mutex			rw_lock;
	struct completion		ack;
	struct completion		fw_buf_ack;
	struct completion		fw_update_ack;
	struct psy_state		psy_list[PSY_TYPE_MAX];
	struct dentry			*debugfs_dir;
	u32				*thermal_levels;
	const char			*wls_fw_name;
	int				curr_thermal_level;
	int				num_thermal_levels;
	int				charger_type;
	int				last_charger_type;
	int				adsp_crash;
	atomic_t			adsp_reboot;
	atomic_t			state;
	int				rerun_max;
	int				pd_chg_volt;
	struct work_struct		subsys_up_work;
	struct work_struct		usb_type_work;

	/* reverse charger	*/
	bool 				reverse_enable;
	bool				high_reverse_enable;
	int				power_role;
	int				sink_req_volt;
	int				sink_req_curr;
	enum reverse_chg_msg_type		rvs_msg_type;


#ifdef OPLUS_FEATURE_CHG_BASIC
	int ccdetect_irq;
	struct work_struct	plc_status_update_work;
	struct delayed_work	publish_close_cp_item_work;
	struct delayed_work	suspend_check_work;
	struct delayed_work	adsp_voocphy_status_work;
	struct delayed_work	otg_init_work;
	struct delayed_work	cid_status_change_work;
	struct delayed_work	usbtemp_recover_work;
	struct delayed_work	adsp_crash_recover_work;
	struct delayed_work	crash_track_work;
	struct delayed_work	voocphy_enable_check_work;
	struct delayed_work	otg_vbus_enable_work;
	struct delayed_work	otg_status_check_work;
	struct delayed_work	vbus_adc_enable_work;
	struct delayed_work	plugin_irq_work;
	struct delayed_work	recheck_input_current_work;
	struct delayed_work	unsuspend_usb_work;
	struct delayed_work	oem_lcm_en_check_work;
	struct delayed_work	ctrl_lcm_frequency;
	struct delayed_work	sourcecap_done_work;
	struct delayed_work	pdo_update_work;
	struct delayed_work	sourcecap_suspend_recovery_work;
	struct delayed_work	update_pd_svooc_work;
	struct delayed_work	iterm_timeout_work;
	struct delayed_work	request_qos_work;
	struct delayed_work	release_qos_work;
	struct work_struct	wired_otg_enable_work;
	struct delayed_work	gauge_register_work;
	struct delayed_work	ufcs_reset_work;
	struct delayed_work	update_common_charge_flag_work;
	struct delayed_work	check_abnormal_usbin_status_work;
	struct delayed_work     crash_timeout_work;
	struct delayed_work	update_pd_completed_work;
	int			abnormal_usbin_count;
	struct delayed_work	reverse_chg_svid_check_work;
	struct delayed_work	source_pdo_check_work;
	bool			qos_status;
	u32			oem_misc_ctl_data;
	bool			oem_usb_online;
	bool			oem_lcm_check;
	struct delayed_work	voocphy_err_work;
	bool					otg_prohibited;
	bool					otg_online;
	bool					is_chargepd_ready;
	bool					pd_svooc;
	bool			is_external_chg;
	struct oplus_chg_iio	iio;
	unsigned long long 	hvdcp_detect_time;
	unsigned long long 	hvdcp_detach_time;
	bool 				hvdcp_detect_ok;
	bool					hvdcp_disable;
	bool				bc12_completed;
	bool				ufcs_test_mode;
	bool				ufcs_power_ready;
	bool				ufcs_handshake_ok;
	bool				ufcs_pdo_ready;
	bool				ufcs_key_to_adsp_done;
	bool				ufcs_verify_auth_ready;
	bool				ufcs_power_info_ready;
	bool				ufcs_vdm_emark_ready;
	bool				ufcs_exiting;
	bool				adapter_verify_auth;
	bool				adspfg_i2c_reset_processing;
	bool				adspfg_i2c_reset_notify_done;
	struct delayed_work 	hvdcp_disable_work;
	struct delayed_work 	pd_only_check_work;
	pd_msg_data			pdo[PPS_PDO_MAX];
	bool					voocphy_err_check;
	bool			bypass_vooc_support;
	bool			usb_aicl_enhance;
	bool			soccp_support;
	bool				qcom_gauge_cali_track_support;
	bool			fg_register_flag;
	struct gauge_track_cali_info_s 	*pre_info;
	struct work_struct		gauge_cali_track_by_plug_work;
	struct work_struct		gauge_cali_track_by_full_work;
	struct mutex                    pre_info_lock;
	struct mutex                    cur_info_lock;

	struct work_struct	gauge_ra0_check_work;
	struct work_struct	gauge_imp_check_work;
	struct work_struct	gauge_delta_voltage_check_work;
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	int vchg_trig_irq;
	struct delayed_work vchg_trig_work;
	struct delayed_work vbus_collapse_rerun_icl_work;
	struct delayed_work wait_wired_charge_on;
	struct delayed_work wait_wired_charge_off;
	bool wls_fw_update;
	int batt_num;
	bool voocphy_bidirect_cp_support;
	bool cid_status;
	atomic_t suspended;
	bool wls_boost_soft_start;
	int wls_set_boost_vol;
	int wls_boost_vol_start_mv;
	int wls_boost_vol_max_mv;
	int wls_boost_src;
#endif /*OPLUS_FEATURE_CHG_BASIC*/
	int				fake_soc;
	bool				block_tx;
	bool				ship_mode_en;
	bool				debug_battery_detected;
	bool				wls_fw_update_reqd;
	u32				wls_fw_version;
	u16				wls_fw_crc;
	struct notifier_block		reboot_notifier;
	u32				thermal_fcc_ua;
	u32				restrict_fcc_ua;
	bool				restrict_chg_en;
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct oplus_custom_gpio_pinctrl oplus_custom_gpio;
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct mutex    read_buffer_lock;
	struct completion    oem_read_ack;
	struct oem_read_buffer_resp_msg  read_buffer_dump;
	struct mutex    bcc_read_buffer_lock;
	struct completion    bcc_read_ack;
	struct oem_read_buffer_resp_msg  bcc_read_buffer_dump;
	struct oem_read_buffer_resp_msg  pps_read_buffer_dump;
	struct mutex	pps_read_buffer_lock;
	struct completion	 pps_read_ack;
	struct oplus_ap_read_buffer_resp_msg	*ap_read_buffer_dump;
	struct mutex				ap_read_buffer_lock;
	struct completion			ap_read_ack[AP_MESSAGE_MAX_SIZE];
	struct mutex				ap_write_buffer_lock;
	struct completion			ap_write_ack;
	struct oplus_ap_read_ufcs_resp_msg  ufcs_read_buffer_dump;
	struct mutex	ufcs_read_buffer_lock;
	struct completion	 ufcs_read_ack;
	struct reverse_chg_msg  rvs_chg_msg_t;
	struct completion	 rvs_chg_msg_ack;
	struct mutex	rvs_chg_msg_lock;

	bool calib_info_init;
	bool real_mvolts_min_support;
	int cp_work_mode;
	bool gauge_data_initialized;
	int otg_scheme;
	int otg_boost_src;
	struct notifier_block	ssr_nb;
	void		*subsys_handle;
	int usb_in_status;

	enum oplus_dpdm_switch_mode dpdm_switch_mode;

	struct regmap *regmap;
	struct delayed_work get_regmap_work;
	struct battery_manufacture_info battinfo;
	struct delayed_work get_manu_battinfo_work;
	int read_by_reg;
	struct delayed_work check_adspfg_status;
	struct delayed_work hboost_notify_work;
	bool ufcs_run_check_support;
	bool error_prop;
	int sub_btb_valid_temp[OPLUS_SUB_BTB_MAX];
	u32 vdm_info_data[OPLUS_VDM_INFO_MAX];
	int vdm_info_cnt;
	struct mutex vdm_info_lock;
#endif
	int batt_full_para[CHARGING_TYPE_MAX][QBG_TEMP_MAX];
	int batt_full_temp[QBG_TEMP_MAX];
	bool batt_full_method_new;
	bool power_mos_status;
	bool need_check_mos;
	bool pd_check_completed;
	int mos_retry_cnt;
	atomic_t is_shaft_btb_over;
	bool adsp_reboot_discnt_chg_support;
	struct aicl_threshold aicl_thr_table[AICL_POINT_INDEX_MAX];
	bool aicl_thr_table_init;
};

/**********************************************************************
 **********************************************************************/

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

struct qcom_pmic {
	struct battery_chg_dev *bcdev_chip;

	/* for complie*/
	bool			otg_pulse_skip_dis;
	int			pulse_cnt;
	unsigned int	therm_lvl_sel;
	bool			psy_registered;
	int			usb_online;

	/* copy from msm8976_pmic begin */
	int			bat_charging_state;
	bool	 		suspending;
	bool			aicl_suspend;
	bool			usb_hc_mode;
	int    		usb_hc_count;
	bool			hc_mode_flag;
	/* copy form msm8976_pmic end */
};

#ifdef OPLUS_FEATURE_CHG_BASIC
int oplus_adsp_voocphy_get_fast_chg_type(void);
int oplus_adsp_voocphy_enable(bool enable);
int oplus_adsp_voocphy_reset_again(void);
int oplus_adsp_batt_curve_current(void);
void oplus_chg_set_match_temp_ui_soc_to_voocphy(void);
void oplus_chg_set_ap_fastchg_allow_to_voocphy(int allow);
int oplus_adsp_voocphy_set_cool_down(int cool_down, int curr_ma);
int oplus_adsp_voocphy_get_bcc_max_current(void);
int oplus_adsp_voocphy_get_bcc_min_current(void);
int oplus_adsp_voocphy_get_atl_last_geat_current(void);
int oplus_adsp_voocphy_set_curve_num(int number);
#endif
#endif /*__SM8350_CHARGER_H*/

