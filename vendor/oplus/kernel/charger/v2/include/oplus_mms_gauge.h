#ifndef __OPLUS_MMS_GAUGE_H__
#define __OPLUS_MMS_GAUGE_H__

#include <oplus_mms.h>
#include <oplus_chg.h>

#define GAUGE_INVALID_TEMP	(-400)

enum gauge_topic_item {
	GAUGE_ITEM_SOC,
	GAUGE_ITEM_VOL,
	GAUGE_ITEM_VOL_MAX,
	GAUGE_ITEM_VOL_MIN,
	GAUGE_ITEM_GAUGE_VBAT,
	GAUGE_ITEM_CURR,
	GAUGE_ITEM_TEMP,
	GAUGE_ITEM_FCC,
	GAUGE_ITEM_CC,
	GAUGE_ITEM_SOH,
	GAUGE_ITEM_RM,
	GAUGE_ITEM_BATT_EXIST,
	GAUGE_ITEM_ERR_CODE,
	GAUGE_ITEM_RESUME,
	GAUGE_ITEM_HMAC,
	GAUGE_ITEM_AUTH,
	GAUGE_ITEM_REAL_TEMP,
	GAUGE_ITEM_SUBBOARD_TEMP_ERR,
	GAUGE_ITEM_VBAT_UV,
	GAUGE_ITEM_DEEP_SUPPORT,
	GAUGE_ITEM_REG_INFO,
	GAUGE_ITEM_CALIB_TIME,
	GAUGE_ITEM_UV_INC,
	GAUGE_ITEM_FCC_COEFF,
	GAUGE_ITEM_SOH_COEFF,
	GAUGE_ITEM_SILI_IC_ALG_DSG_ENABLE,
	GAUGE_ITEM_SILI_IC_ALG_CFG,
	GAUGE_ITEM_SPARE_POWER_ENABLE,
	GAUGE_ITEM_SILI_IC_ALG_TERM_VOLT,
	GAUGE_ITEM_LIFETIME_STATUS,
	GAUGE_ITEM_RATIO_VALUE,
	GAUGE_ITEM_RATIO_TRANGE,
	GAUGE_ITEM_QMAX,
	GAUGE_ITEM_CAR_C,
	GAUGE_ITEM_RATIO_LIMIT_CURR,
	GAUGE_ITEM_SUB_BTB_STATE,
	GAUGE_ITEM_GAUGE_R_INFO,
	GAUGE_ITEM_SOC_CENTI,
	GAUGE_ITEM_VOL_FCL,
};

enum gauge_type_id {
	DEVICE_BQ27541,
	DEVICE_BQ27411,
	DEVICE_BQ28Z610,
	DEVICE_ZY0602,
	DEVICE_ZY0603,
	DEVICE_NFG8011B,
	DEVICE_SN28Z729 = 7,
	DEVICE_MPC7022,
};

typedef enum {
	DEC_CV_PACK_UNKNOWN,
	DEC_CV_PACK_DOUBLE_SERIES,
	DEC_CV_PACK_DOUBLE_PARALLE,
	DEC_CV_PACK_SINGLE,
	DEC_CV_PACK_SOH,
	DEC_CV_QCOM_FG,
	DEC_CV_MTK_FG,
	DEC_CV_MB_TI,
	DEC_CV_MB_CW,
	DEC_CV_MB_SMI,
	DEC_CV_PACK_MAX,
} DEC_CV_PACK_TYPE;

#define OPLUS_BATTINFO_DATE_SIZE 11
#define OPLUS_BATT_SERIAL_NUM_SIZE 20
#define CHEM_ID_LENGTH 8
#define CHEMID_MAX_LENGTH 512

enum batt_connect_type {
	DEFAULT_CONNECT_TYPE,
	PARALLEL_CONNECT_TYPE,
	SERIAL_CONNECT_TYPE,
};

enum {
    GAUGE_TYPE_UNKNOW = 0,
    GAUGE_TYPE_PLATFORM = 1,
    GAUGE_TYPE_PACK = 2,
    GAUGE_TYPE_BOARD = 3,
    GAUGE_TYPE_MAX,
};

#define BATT_SUB_BTB_ABNORMAL_MAX_CURR	9500
#define BATT_SUB_BTB_ABNORMAL_MIN_CURR	2000

struct battery_manufacture_info {
    u16 manu_date;
    u16 first_usage_date;
    u16 ui_cycle_count;
    u8 ui_soh;
    u8 used_flag;
    char batt_serial_num[OPLUS_BATT_SERIAL_NUM_SIZE];
} __attribute__((packed));

#define GAUGE_CALIB_ARGS_LEN 12
struct gauge_calib_info {
	int dod_time;
	int qmax_time;
	unsigned char calib_args[GAUGE_CALIB_ARGS_LEN];
}__attribute__((aligned(4)));

#define OPLUS_GAUGE_THREE_LEVEL_TERM_VOLT_LEN   18
#define OPLUS_GAUGE_CUV_STATE_CHECK_TRY_MAX    2
#define OPLUS_GAUGE_CUV_STATE_1    1
#define OPLUS_GAUGE_CUV_STATE_2    0

enum battery_test_state {
	BATTERY_STOP_TEST_TYPE,
	BATTERY_START_TEST_TYPE,
	BATTERY_MAX_TEST_TYPE,
};

struct oplus_gauge_nvram_stress_test {
	int input_count;
	int interval_ms;
	int input_state;

	/* nvram test */
	int sum_cnt;
	int fail_cnt;
	int suc_cnt;

	/* term volt test */
	int term_volt_sum_cnt;
	int term_volt_fail_cnt;
	int term_volt_suc_cnt;

	/* normal gauge test */
	int read_sum_cnt;
	int read_fail_cnt;
	int read_suc_cnt;

	int test_state;
	int read_test_state;
	int term_volt_test_state;
};

int oplus_gauge_get_batt_mvolts(void);
int oplus_gauge_get_batt_fc(void);
int oplus_gauge_get_batt_qm(void);
int oplus_gauge_get_batt_pd(void);
int oplus_gauge_get_batt_rcu(void);
int oplus_gauge_get_batt_rcf(void);
int oplus_gauge_get_batt_fcu(void);
int oplus_gauge_get_batt_fcf(void);
int oplus_gauge_get_batt_sou(void);
int oplus_gauge_get_batt_do0(void);
int oplus_gauge_get_batt_doe(void);
int oplus_gauge_get_batt_trm(void);
int oplus_gauge_get_batt_pc(void);
int oplus_gauge_get_batt_qs(void);
int oplus_gauge_get_batt_mvolts_2cell_max(void);
int oplus_gauge_get_batt_mvolts_2cell_min(void);

int oplus_gauge_get_batt_soc(void);
int oplus_gauge_get_batt_current(void);
int oplus_gauge_get_real_time_current(void);
int oplus_gauge_get_remaining_capacity(void);
int oplus_gauge_get_device_type(void);
int oplus_gauge_get_device_type_for_vooc(void);

int oplus_gauge_get_batt_fcc(void);

int oplus_gauge_get_batt_cc(void);
int oplus_gauge_get_batt_soh(void);
bool oplus_gauge_get_batt_hmac(void);
bool oplus_gauge_get_batt_authenticate(void);
int oplus_gauge_get_physical_name(struct oplus_mms *mms, char *name, int len);
void oplus_gauge_set_batt_full(bool);
bool oplus_gauge_check_chip_is_null(void);
bool oplus_gauge_is_exist(struct oplus_mms *topic);
bool oplus_sub_gauge_is_exist(struct oplus_mms *topic);

int oplus_gauge_update_battery_dod0(void);
int oplus_gauge_update_soc_smooth_parameter(void);
int oplus_gauge_get_battery_cb_status(void);
int oplus_gauge_get_i2c_err(void);
void oplus_gauge_clear_i2c_err(void);
int oplus_gauge_get_passedchg(int *val);
int oplus_gauge_dump_register(void);
int oplus_gauge_lock(void);
int oplus_gauge_unlock(void);
bool oplus_gauge_is_locked(void);
int oplus_gauge_get_batt_num(void);
int oplus_get_gauge_type(void);
int oplus_gauge_get_batt_capacity_mah(struct oplus_mms *topic);

int oplus_gauge_get_dod0(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_dod0_passed_q(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_car_c(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_qmax(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_qmax_passed_q(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_volt(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_gauge_type(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_bcc_parameters(char *buf);
int oplus_gauge_fastchg_update_bcc_parameters(char *buf);
int oplus_gauge_get_prev_bcc_parameters(char *buf);
int oplus_gauge_set_bcc_parameters(const char *buf);
int oplus_gauge_get_sub_btb_curr_limit(struct oplus_mms *topic);
int oplus_gauge_protect_check(void);
bool oplus_gauge_afi_update_done(void);
void oplus_gauge_set_plugin_status(void);

bool oplus_gauge_check_reset_condition(void);
bool oplus_gauge_reset(void);
int is_support_parallel_battery(struct oplus_mms *topic);
void oplus_gauge_set_deep_dischg_count(struct oplus_mms *topic, int count);
int oplus_gauge_show_deep_dischg_count(struct oplus_mms *topic);
void oplus_gauge_set_deep_count_cali(struct oplus_mms *topic, int val);
int oplus_gauge_get_deep_count_cali(struct oplus_mms *topic);
void oplus_gauge_set_deep_dischg_ratio_thr(struct oplus_mms *topic, int ratio);
int oplus_gauge_get_deep_dischg_ratio_thr(struct oplus_mms *topic);
int oplus_gauge_get_battinfo_sn(struct oplus_mms *topic, char *sn_buff, int size_buffer);
int oplus_gauge_get_sili_alg_application_info(struct oplus_mms *topic, u8 *info, int len);
int oplus_gauge_get_sili_alg_lifetime_info(struct oplus_mms *mms, u8 *info, int len);
int oplus_gauge_get_battinfo_manu_date(struct oplus_mms *topic, char *buff, int size_buffer);
int oplus_gauge_get_battinfo_first_usage_date(struct oplus_mms *topic, char *buff, int size_buffer);
int oplus_gauge_set_battinfo_first_usage_date(struct oplus_mms *topic, const char *buff);
int oplus_set_reset_gauge_parms(struct oplus_mms *topic, const int *buff);
int oplus_gauge_get_batt_sn(struct oplus_mms *topic, char *buff, int size_buffer);
int oplus_gauge_get_historic_soh_date(struct oplus_mms *topic, int *buff, int size_buffer);
int oplus_set_histrioc_soh_date(struct oplus_mms *topic, const int *buff);
int oplus_gauge_get_ui_cc(struct oplus_mms *topic);
int oplus_gauge_set_ui_cc(struct oplus_mms *topic, int count);
int oplus_gauge_get_ui_soh(struct oplus_mms *topic);
int oplus_gauge_set_ui_soh(struct oplus_mms *topic, int ui_soh);
int oplus_gauge_get_used_flag(struct oplus_mms *topic);
int oplus_gauge_set_used_flag(struct oplus_mms *topic, int flag);
int oplus_gauge_show_batt_chem_id(struct oplus_mms *topic, char *buf, int len);
int oplus_gauge_set_seal_flag(struct oplus_mms *topic, int seal_flag);
bool oplus_gauge_set_fg_vct(struct oplus_mms *mms, int count);
int oplus_gauge_get_fg_vct(struct oplus_mms *mms);
int oplus_gauge_get_dec_pack_type(struct oplus_mms *mms);
int oplus_gauge_get_dec_cv_soh(struct oplus_mms *mms);
int oplus_gauge_sec_get_romid(struct oplus_mms *topic, uint8_t *romid, int *len);
int oplus_gauge_sec_write_page(struct oplus_mms *topic, int page_id, uint8_t *data, int len);
int oplus_gauge_sec_read_page(struct oplus_mms *topic, int page_id, uint8_t *data, int *len);
int oplus_gauge_sec_ecdsa(struct oplus_mms *topic, bool *val);
int oplus_gauge_sec_ecw(struct oplus_mms *topic, bool *val);
int oplus_gauge_sec_shutdown(struct oplus_mms *topic, bool *val);
int oplus_gauge_sec_set_prikey(struct oplus_mms *topic, int index, uint8_t *prikey, int len);
int oplus_gauge_sec_get_prikey_index(struct oplus_mms *topic, int *index);
int oplus_gauge_set_cuv_state(struct oplus_mms *mms, int state);
int oplus_gauge_get_cuv_state(struct oplus_mms *mms, int *state);
bool oplus_gauge_get_fcl_curr(int hw_vth, int sw_vth, int vbat, int *curr_dec, int *min_curr, bool *hw);
int oplus_gauge_get_fcl_support(struct oplus_mms *mms, bool *fcl_support);
int oplus_gauge_get_vb_offset(struct oplus_mms *mms, int *vb_offset);

int oplus_gauge_start_nvram_stress_test(struct oplus_mms *topic,
		int input_count, int interval_ms);
int oplus_gauge_start_stress_read_test(struct oplus_mms *topic,
		int input_count, int interval_ms);
int oplus_gauge_get_nvram_stress_test(struct oplus_mms *topic,
		struct oplus_gauge_nvram_stress_test *data);
int oplus_gauge_start_term_volt_stress_test(struct oplus_mms *topic,
		int input_count, int interval_ms);
int oplus_gauge_get_three_level_term_volt(struct oplus_mms *topic, int term_volt[]);

#endif /* __OPLUS_MMS_GAUGE_H__ */
