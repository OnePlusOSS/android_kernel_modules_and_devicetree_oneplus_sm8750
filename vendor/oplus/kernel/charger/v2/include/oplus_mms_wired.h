#ifndef __OPLUS_MMS_WIRED_H__
#define __OPLUS_MMS_WIRED_H__

#include <oplus_mms.h>

enum wired_topic_item {
	WIRED_ITEM_PRESENT,
	WIRED_ITEM_ONLINE,
	WIRED_ITEM_ERR_CODE,
	WIRED_ITEM_CHG_TYPE,
	WIRED_ITEM_BC12_COMPLETED,
	WIRED_ITEM_CC_MODE,
	WIRED_ITEM_CC_DETECT,
	WIRED_ITEM_USB_STATUS,
	WIRED_ITEM_USB_TEMP_VOLT_L,
	WIRED_ITEM_USB_TEMP_VOLT_R,
	WIRED_ITEM_USB_TEMP_L,
	WIRED_ITEM_USB_TEMP_R,
	WIRED_ITEM_PRE_OTG_ENABLE,
	WIRED_ITEM_OTG_ENABLE,
	WIRED_ITEM_CHARGER_CURR_MAX,
	WIRED_ITEM_CHARGER_VOL_MAX,
	WIRED_ITEM_CHARGER_VOL_MIN,
	WIRED_TIME_ABNORMAL_ADAPTER,
	WIRED_TIME_TYPEC_STATE,
	WIRED_ITEM_REAL_CHG_TYPE,
	WIRED_ITEM_VBUS,
	WIRED_ITEM_ONLINE_STATUS_ERR,
	WIRED_ITEM_CHARGING_DISABLE,
	WIRED_ITEM_COEXISTENCE,
	WIRED_ITEM_PD_COMPLETED,
	WIRED_ITEM_ICL_DONE_STATUS,
	WIRED_ITEM_VBUS_VOL_TYPE,
	WIRED_ITEM_BUCK_EIS_CURRENT_RATE,
	WIRED_ITEM_POWER_ROLE,
	WIRED_ITEM_SOURCE_PDO_VOLT,
};

enum oplus_wired_cc_detect_status {
	CC_DETECT_NULL,
	CC_DETECT_NOTPLUG,
	CC_DETECT_PLUGIN,
};

typedef enum {
	BCC_CURR_DONE_UNKNOW = 0,
	BCC_CURR_DONE_REQUEST,
	BCC_CURR_DONE_ACK,
}OPLUS_BCC_CURR_DONE_STATUS;

typedef enum
{
	OPLUS_LPD_SEL_SBU1 = 0x0,
	OPLUS_LPD_SEL_SBU2,
	OPLUS_LPD_SEL_SBU1_400K,
	OPLUS_LPD_SEL_SBU2_400K,
	OPLUS_LPD_SEL_CC1,
	OPLUS_LPD_SEL_CC2,
	OPLUS_LPD_SEL_DP,
	OPLUS_LPD_SEL_DM,
	OPLUS_LPD_SEL_INVALID,
} OPLUS_LPD_SEL_TYPE;

#define OPLUS_LPD_SEL_SBU1_MASK		(1 << OPLUS_LPD_SEL_SBU1)
#define OPLUS_LPD_SEL_SBU2_MASK		(1 << OPLUS_LPD_SEL_SBU2)
#define OPLUS_LPD_SEL_SBU1_PULLUP_MASK	(1 << OPLUS_LPD_SEL_SBU1_400K)
#define OPLUS_LPD_SEL_SBU2_PULLUP_MASK	(1 << OPLUS_LPD_SEL_SBU2_400K)
#define OPLUS_LPD_SEL_CC1_MASK		(1 << OPLUS_LPD_SEL_CC1)
#define OPLUS_LPD_SEL_CC2_MASK		(1 << OPLUS_LPD_SEL_CC2)
#define OPLUS_LPD_SEL_DP_MASK		(1 << OPLUS_LPD_SEL_DP)
#define OPLUS_LPD_SEL_DM_MASK		(1 << OPLUS_LPD_SEL_DM)

#define OPLUS_LPD_NOT_SUPPORT		0
#define OPLUS_LPD_SUPPORT		1
#define OPLUS_LPD_SUPPORT_NOT_SHOW_UI	2

#define OPLUS_LPD_NOT_DETECT		0
#define OPLUS_LPD_DETECT		1
#define OPLUS_LPD_ERROR			2

#define OPLUS_LPD_INFO_LEN		16
#define MAX_LPD_CONFIG_NUM		10
#define MAX_LPD_RANG_NUM		2

#define SMART_CHARGE_USER_USBTEMP	1
#define SMART_CHARGE_USER_OTHER		0

const char *oplus_wired_get_chg_type_str(enum oplus_chg_usb_type);
int oplus_wired_get_charger_cycle(void);
void oplus_wired_dump_regs(void);
int oplus_wired_get_vbus(void);
bool oplus_wired_get_vbus_collapse_status(void);
int oplus_wired_set_icl(int icl_ma, bool step);
int oplus_wired_set_icl_by_vooc(struct oplus_mms *topic, int icl_ma);
int oplus_wired_set_fcc(int fcc_ma);
int oplus_wired_smt_test(char buf[], int len);
bool oplus_wired_is_present(void);
int oplus_wired_input_enable(bool enable);
bool oplus_wired_input_is_enable(void);
int oplus_wired_output_enable(bool enable);
bool oplus_wired_output_is_enable(void);
int oplus_wired_get_icl(void);
int oplus_wired_set_fv(int fv_mv);
int oplus_wired_get_fv(int *fv_mv);
int oplus_wired_set_iterm(int iterm_ma);
int oplus_wired_set_rechg_vol(int vol_mv);
int oplus_wired_get_ibus(void);
int oplus_wired_get_chg_type(void);
int oplus_wired_otg_boost_enable(bool enable);
int oplus_wired_set_otg_boost_vol(int vol_mv);
int oplus_wired_set_otg_boost_curr_limit(int curr_ma);
int oplus_wired_aicl_enable(bool enable);
int oplus_wired_aicl_rerun(void);
int oplus_wired_aicl_reset(void);
int oplus_wired_set_aicl_point(void);
int oplus_wired_get_cc_orientation(void);
int oplus_wired_get_hw_detect(void);
int oplus_wired_get_hw_detect_recheck(void);
int oplus_wired_rerun_bc12(void);
int oplus_wired_qc_detect_enable(bool enable);
int oplus_wired_shipmode_enable(bool enable);
bool oplus_wired_shipmode_is_enabled(void);
int oplus_wired_set_qc_config(enum oplus_chg_qc_version version, int vol_mv);
int oplus_wired_set_pd_config(u32 pdo);
int oplus_wired_get_usb_temp_volt(int *vol_l, int *vol_r);
int oplus_wired_get_usb_temp(int *temp_l, int *temp_r);
bool oplus_wired_usb_temp_check_is_support(void);
int oplus_wired_get_usb_btb_temp(void);
int oplus_wired_get_batt_btb_temp(void);
enum oplus_chg_typec_port_role_type oplus_wired_get_typec_mode(void);
int oplus_wired_set_typec_mode(enum oplus_chg_typec_port_role_type mode);
int oplus_wired_set_otg_switch_status(bool en);
bool oplus_wired_get_otg_switch_status(void);
int oplus_wired_wdt_enable(struct oplus_mms *topic, bool enable);
int oplus_wired_kick_wdt(struct oplus_mms *topic);
int oplus_wired_get_voltage_max(struct oplus_mms *topic);
int oplus_wired_get_shutdown_soc(struct oplus_mms *topic);
int oplus_wired_get_otg_online_status(struct oplus_mms *topic);
void oplus_wired_check_bcc_curr_done(struct oplus_mms *topic);
int oplus_wired_get_bcc_curr_done_status(struct oplus_mms *topic);
void oplus_wired_set_bcc_curr_request(struct oplus_mms *topic);
int oplus_wired_get_byb_id_info(struct oplus_mms *topic);
int oplus_wired_get_byb_id_match_info(struct oplus_mms *topic);
int oplus_wired_get_byb_status_info(struct oplus_mms *topic, char *buf);
int oplus_wired_get_byb_vout_info(struct oplus_mms *topic, char *byb_buff);
int oplus_wired_set_byb_vout_info(struct oplus_mms *topic, int byb_vout);
int oplus_wired_get_vbat_pwr(void);
bool oplus_wired_is_usb_aicl_enhance(void);
int oplus_wired_get_lpd_info_status(struct oplus_mms *topic);
int oplus_wired_set_lpd_config(struct oplus_mms *topic, int *config);
int oplus_wired_set_chg_path(struct oplus_mms *topic, int path);
int oplus_wired_get_chg_path_status(struct oplus_mms *topic);
int oplus_wired_push_info(struct oplus_mms *topic, const char *err_scene, const char *err_reason);
int oplus_wired_iterm_check(struct oplus_mms *topic, bool enable);
int oplus_wired_get_shaft_btb_temp(void);
bool oplus_wired_get_shaft_btb_is_normal(void);
int oplus_wired_set_shaft_btb_over(bool is_shaft_btb_over);
bool oplus_wired_get_supplementary_power_mos(void);
int oplus_wired_set_supplementary_power_mos(struct oplus_mms *topic, bool enable);
int oplus_wired_set_dischg_status(bool dischg_en);
int oplus_set_ovp_forced(bool enable);
int oplus_set_dpdm_ovp_disable(bool disable);
int oplus_wired_get_vdm_info(struct oplus_mms *topic, u32 *vdm_data, int *vdm_cnt);
bool oplus_wired_is_gamepad_active(void);
int oplus_wired_send_get_sink_cap(void);

#endif /* __OPLUS_MMS_WIRED_H__ */
