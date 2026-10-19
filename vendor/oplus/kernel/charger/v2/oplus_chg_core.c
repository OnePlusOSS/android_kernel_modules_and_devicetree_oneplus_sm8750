#define pr_fmt(fmt) "[CORE]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#endif
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mtk_boot_common.h>
#endif

#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>

int oplus_log_level = LOG_LEVEL_INFO;
module_param(oplus_log_level, int, 0644);
MODULE_PARM_DESC(oplus_log_level, "debug log level");
EXPORT_SYMBOL(oplus_log_level);

int charger_abnormal_log = 0;

int oplus_is_rf_ftm_mode(void)
{
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int boot_mode = get_boot_mode();
#ifdef CONFIG_OPLUS_CHARGER_MTK
	struct device_node * of_chosen = NULL;
	char *bootargs = NULL;

	of_chosen = of_find_node_by_path("/chosen");

	if (boot_mode == META_BOOT || boot_mode == FACTORY_BOOT ||
	    boot_mode == ADVMETA_BOOT || boot_mode == ATE_FACTORY_BOOT) {
		chg_debug(" boot_mode:%d, return\n", boot_mode);
		if (of_chosen) {
			/* Add for MTK FTM AGING DDR test mode, if FTM AGING mode, enable charging.
			If Qcom platform want enable this feature, need resubmit issue */
			bootargs = (char *)of_get_property(of_chosen, "bootargs", NULL);
			if (!bootargs)
				chg_err("%s: failed to get bootargs\n", __func__);
			else {
				chg_debug("%s: bootargs: %s\n", __func__, bootargs);
				if (strstr(bootargs, "oplus_ftm_mode=ftmaging")) {
					chg_debug("%s: ftmaging!\n", __func__);
					return false;
				} else {
					chg_debug("%s: not ftmaging!\n", __func__);
				}
			}
		} else {
			chg_err("%s: failed to get /chosen \n", __func__);
		}
		return true;
	} else {
		/*chg_debug(" boot_mode:%d, return false\n",boot_mode);*/
		return false;
	}
#else
	if (boot_mode == MSM_BOOT_MODE__RF ||
	    boot_mode == MSM_BOOT_MODE__WLAN ||
	    boot_mode == MSM_BOOT_MODE__FACTORY) {
		chg_debug(" boot_mode:%d, return\n", boot_mode);
		return true;
	} else {
		/*chg_debug(" boot_mode:%d, return false\n",boot_mode);*/
		return false;
	}
#endif
#else /*CONFIG_DISABLE_OPLUS_FUNCTION*/
	return false;
#endif /*CONFIG_DISABLE_OPLUS_FUNCTION*/
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
bool __attribute__((weak)) qpnp_is_charger_reboot(void);
bool __attribute__((weak)) qpnp_is_power_off_charging(void);
#endif /*CONFIG_DISABLE_OPLUS_FUNCTION*/
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
/* only for GKI compile */
bool __attribute__((weak)) qpnp_is_charger_reboot(void)
{
	return false;
}

bool __attribute__((weak)) qpnp_is_power_off_charging(void)
{
	return false;
}
#endif
#endif

bool oplus_is_power_off_charging(void)
{
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		return true;
	} else {
		return false;
	}
#else
	return qpnp_is_power_off_charging();
#endif
#else /*CONFIG_DISABLE_OPLUS_FUNCTION*/
	return false;
#endif /*CONFIG_DISABLE_OPLUS_FUNCTION*/
}

bool oplus_is_charger_reboot(void)
{
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#ifdef CONFIG_OPLUS_CHARGER_MTK
	/*TODO
	int charger_type;

	charger_type = oplus_chg_get_chg_type();
	if (charger_type == 5) {
		chg_debug("dont need check fw_update\n");
		return true;
	} else {
		return false;
	}*/
	return false;
#else
	return qpnp_is_charger_reboot();
#endif
#else /*CONFIG_DISABLE_OPLUS_FUNCTION*/
	return false;
#endif /*CONFIG_DISABLE_OPLUS_FUNCTION*/
}

struct timespec oplus_current_kernel_time(void)
{
	struct timespec ts;
	getnstimeofday(&ts);
	return ts;
}

bool oplus_is_ptcrb_version(void)
{
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
#ifndef CONFIG_OPLUS_CHARGER_MTK
	return (get_eng_version() == PTCRB);
#else
	return false;
#endif
#else /*CONFIG_DISABLE_OPLUS_FUNCTION*/
	return false;
#endif /*CONFIG_DISABLE_OPLUS_FUNCTION*/
}

int oplus_get_chg_spec_version(void)
{
	struct device_node *node;
	static int oplus_chg_spec_ver = -EINVAL;
	int rc;

	if (oplus_chg_spec_ver == -EINVAL) {
		node = of_find_node_by_path("/soc/oplus_chg_core");
		if (node != NULL) {
			rc = of_property_read_u32(node, "oplus,chg_spec_version",
						  &oplus_chg_spec_ver);
			if (rc < 0) {
				chg_err("get oplus,vooc_spec_version property error\n");
				oplus_chg_spec_ver = OPLUS_CHG_SPEC_VER_V3P6;
			}
			chg_info("oplus_chg_spec_ver = %d\n", oplus_chg_spec_ver);
		} else {
			chg_err("not found oplus_chg_core node\n");
			oplus_chg_spec_ver = OPLUS_CHG_SPEC_VER_V3P6;
			rc = -ENODEV;
		}
	}

	return oplus_chg_spec_ver;
}

#define DEFAULT_REGION_ID 0xFF
uint8_t oplus_chg_get_region_id(void)
{
	struct device_node *node;
	const char *bootparams = NULL;
	char *str;
	int tmp_region = 0;
	int ret = 0;
	static uint8_t region_id = 0xFF;
	static bool initialized = false;

	if (initialized)
		return region_id;

	node = of_find_node_by_path("/chosen");
	if (node) {
		ret = of_property_read_string(node, "bootargs", &bootparams);
		if (!bootparams || ret < 0) {
			chg_err("failed to get bootargs property");
			goto err;
		}

		str = strstr(bootparams, "oplus_region=");
		if (str) {
			str += strlen("oplus_region=");
			ret = get_option(&str, &tmp_region);
			if (ret == 1) {
				region_id = tmp_region & 0xFF;
				chg_info("oplus_region=0x%02x", region_id);
			}
		}
	}

err:
	initialized = true;
	return region_id;
}

struct region_list {
	int elem_count;
	u8 *oplus_region_list;
};

static bool find_id_in_region_list(u8 id, struct region_list *region_list)
{
	int index = 0;

	if (id == DEFAULT_REGION_ID || !region_list || !region_list->oplus_region_list)
		return false;

	for (index = 0; index < region_list->elem_count; index++) {
		if (id == (region_list->oplus_region_list[index] & 0xFF))
			return true;
	}

	return false;
}

unsigned int oplus_chg_get_nvid_support_flags(void)
{
	int len = 0;
	int rc = 0;
	int i = 0;
	struct device_node *node = of_find_node_by_path("/soc/oplus_chg_core");
	struct region_list region_list_arrry[REGION_INDEX_MAX] = {
		{ 0, NULL },
	};
	uint8_t region_id = oplus_chg_get_region_id();
	static unsigned int nvid_support_flags = 0;
	static bool initialized = false;

	if (initialized)
		return nvid_support_flags;

	for (i = 0; i < REGION_INDEX_MAX; i++) {
		rc = of_property_count_elems_of_size(node, oplus_region_list_index_str(i), sizeof(u8));
		if (rc > 0) {
			len = rc;
			region_list_arrry[i].oplus_region_list = kzalloc(len, GFP_KERNEL);
			if (!region_list_arrry[i].oplus_region_list) {
				chg_err("NOMEM for %s\n", oplus_region_list_index_str(i));
				continue;
			}
			rc = of_property_read_u8_array(node, oplus_region_list_index_str(i),
						       region_list_arrry[i].oplus_region_list, len);
			if (rc < 0) {
				len = 0;
				chg_err("parse %s failed, rc=%d\n", oplus_region_list_index_str(i), rc);
			}
		} else {
			len = 0;
			chg_err("parse %s_length failed, rc=%d\n", oplus_region_list_index_str(i), rc);
		}
		region_list_arrry[i].elem_count = len;
		if (len > 0 && find_id_in_region_list(region_id, &region_list_arrry[i]))
			nvid_support_flags |= BIT(i);
		if (region_list_arrry[i].oplus_region_list != NULL)
			kfree(region_list_arrry[i].oplus_region_list);
	}

	initialized = true;
	return nvid_support_flags;
}

bool oplus_chg_get_common_charge_icl_support_flags(void)
{
	struct device_node *node;
	static int common_charge_icl_support = -EINVAL;

	if (common_charge_icl_support == -EINVAL) {
		node = of_find_node_by_path("/soc/oplus_chg_core");
		if (node != NULL) {
			if (!of_property_read_bool(node, "oplus,common-charge-icl-support")) {
				common_charge_icl_support = 0;
				chg_err("get oplus,vooc_spec_version property error\n");
			} else {
				common_charge_icl_support = 1;
				chg_info("common_charge_icl_support = %d\n", common_charge_icl_support);
			}
		} else {
			chg_err("not found oplus_chg_core node\n");
			common_charge_icl_support = 0;
		}
	}

	return ((common_charge_icl_support == 1) ? true : false);
}

bool oplus_chg_get_boot_reset_adapter_support_flags(void)
{
	struct device_node *node;
	static int boot_reset_adapter_support = -EINVAL;

	if (boot_reset_adapter_support == -EINVAL) {
		node = of_find_node_by_path("/soc/oplus_chg_core");
		if (node != NULL) {
			if (!of_property_read_bool(node, "oplus,boot_reset_adapter_support")) {
				boot_reset_adapter_support = 0;
			} else {
				boot_reset_adapter_support = 1;
				chg_info("boot_reset_adapter_support = %d\n", boot_reset_adapter_support);
			}
		} else {
			chg_err("not found oplus_chg_core node\n");
			boot_reset_adapter_support = 0;
		}
	}

	return ((boot_reset_adapter_support == 1) ? true : false);
}

bool oplus_chg_get_fcs_support_flags(void)
{
	struct device_node *node;
	static int fcs_support = -EINVAL;

	if (fcs_support == -EINVAL) {
		node = of_find_node_by_path("/soc/oplus_chg_core");
		if (node != NULL) {
			if (!of_property_read_bool(node, "oplus,fcs_support")) {
				fcs_support = 0;
			} else {
				fcs_support = 1;
				chg_info("fcs_support = %d\n", fcs_support);
			}
		} else {
			chg_err("not found oplus_chg_core node\n");
			fcs_support = 0;
		}
	}
	return ((fcs_support == 1) ? true : false);
}

enum sn_match_type {
	SN_MATCH_REGION = 0,
	MAX_MATCH_TYPE,
};

struct sn_match_ops {
	const char *prop_name;
	u32 (*get_target_val)(void);
};

__maybe_unused static u32 get_region_target(void)
{
#ifdef CONFIG_DISABLE_OPLUS_FUNCTION
	return 0;
#else
	return get_Operator_Version();
#endif
}

__maybe_unused static const struct sn_match_ops sn_match_ops_table[MAX_MATCH_TYPE] = {
	[SN_MATCH_REGION] = {
		.prop_name = "region",
		.get_target_val = get_region_target,
	},
};

__maybe_unused static bool sn_prefix_match(const struct device_node *node, const char *actual_sn)
{
	struct property *prop;
	const char *prefix;
	size_t actual_len;
	size_t prefix_len;

	if (!actual_sn || !*actual_sn) {
		chg_info("invalid actual_sn: NULL or empty\n");
		return true;
	}

	actual_len = strlen(actual_sn);

	of_property_for_each_string(node, "sn", prop, prefix) {
		prefix_len = strlen(prefix);
		if (actual_len < prefix_len)
			continue;
		if (strncmp(actual_sn, prefix, prefix_len) == 0) {
			chg_info("sn matched: prefix=%s, actual=%s\n", prefix, actual_sn);
			return true;
		}
	}
	return false;
}

bool battery_sn_match(struct device_node *node, const char *actual_sn)
{
#ifdef CONFIG_DISABLE_OPLUS_FUNCTION
	chg_info("oplus function disabled, skip sn match\n");
	return true;
#else
	struct device_node *config_node = NULL;
	const struct sn_match_ops *ops;
	u32 target_val, match_type = MAX_MATCH_TYPE;
	struct device_node *child_node = NULL;
#if (KERNEL_VERSION(6, 10, 0) > LINUX_VERSION_CODE)
	struct property *prop;
	const __be32 *p;
#endif
	u32 curr_val;
	bool match = false;

	if (!node) {
		chg_err("invalid node: NULL\n");
		return false;
	}

	config_node = of_get_child_by_name(node, "sn-match-config");
	if (!config_node) {
		chg_err("no 'sn-match-config' child\n");
		return false;
	}

	if (of_property_read_u32(config_node, "match-type", &match_type) != 0) {
		chg_err("sn-match-config missing 'match-type' property\n");
		goto out;
	}

	if (match_type >= MAX_MATCH_TYPE) {
		chg_err("invalid match-type: %u\n", match_type);
		goto out;
	}

	ops = &sn_match_ops_table[match_type];
	if (!ops->prop_name || !ops->get_target_val) {
		chg_err("match-type %u not implemented\n", match_type);
		goto out;
	}

	target_val = ops->get_target_val();

	for_each_available_child_of_node(config_node, child_node) {
#if (KERNEL_VERSION(6, 10, 0) > LINUX_VERSION_CODE)
		of_property_for_each_u32(child_node, ops->prop_name, prop, p, curr_val) {
#else
		of_property_for_each_u32(child_node, ops->prop_name, curr_val) {
#endif
			if (curr_val == target_val) {
				if (sn_prefix_match(child_node, actual_sn))
					match = true;
				chg_info("sn_match=%d\n", match);
				goto out;
			}
		}
	}

	chg_info("sn not match\n");
out:
	of_node_put(child_node);
	of_node_put(config_node);
	return match;
#endif
}

struct device_node *oplus_get_node_by_type(struct device_node *father_node)
{
	char battery_type_str[OPLUS_BATTERY_TYPE_LEN] = { 0 };
	struct device_node *sub_node = NULL;
	struct device_node *node = father_node;
	int rc = oplus_gauge_get_battery_type_str(battery_type_str);
	if (rc == 0) {
		sub_node = of_get_child_by_name(father_node, battery_type_str);
		if (sub_node)
			node = sub_node;
	}
	return node;
}

struct device_node *oplus_get_node_by_child_gauge(struct device_node *father_node)
{
	struct device_node *node = of_find_node_by_path("/soc/oplus_chg_core");
	bool use_child = false;

	if (node != NULL) {
		use_child = of_property_read_bool(node, "oplus,gauge_ic_by_child_node");
		of_node_put(node);
	}

	if (!use_child)
		return father_node;

	return oplus_get_node_by_type(father_node);
}

#ifdef MODULE

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))

__attribute__((weak)) size_t __oplus_chg_module_start;
__attribute__((weak)) size_t __oplus_chg_module_end;

static int oplus_chg_get_module_num(void)
{
	size_t addr_size = (size_t)&__oplus_chg_module_end -
			   (size_t)&__oplus_chg_module_start;

	if (addr_size == 0)
		return 0;
	if (addr_size % sizeof(struct oplus_chg_module) != 0) {
		chg_err("oplus chg module address is error, please check oplus_chg_module.lds\n");
		return 0;
	}

	return (addr_size / sizeof(struct oplus_chg_module));
}

static struct oplus_chg_module *oplus_chg_find_first_module(void)
{
	size_t start_addr = (size_t)&__oplus_chg_module_start;
	return (struct oplus_chg_module *)READ_ONCE_NOCHECK(start_addr);
}

static int __init oplus_chg_modules_init(void)
{
	int module_num, i;
	struct oplus_chg_module *first_module;
	struct oplus_chg_module *oplus_module;
	int rc;

	module_num = oplus_chg_get_module_num();
	if (module_num == 0) {
		chg_err("oplus chg module not found, please check oplus_chg_module.lds\n");
		return 0;
	} else {
		chg_info("find %d oplus chg module\n", module_num);
	}

	first_module = oplus_chg_find_first_module();
	for (i = 0; i < module_num; i++) {
		oplus_module = &first_module[i];
		if ((oplus_module->magic == OPLUS_CHG_MODULE_MAGIC) &&
		    (oplus_module->chg_module_init != NULL)) {
			chg_info("%s init\n", oplus_module->name);
			rc = oplus_module->chg_module_init();
			if (rc < 0) {
				chg_err("%s init error, rc=%d\n",
					oplus_module->name, rc);
				goto module_init_err;
			}
		}
	}

	return 0;

module_init_err:
	for (i = i - 1; i >= 0; i--) {
		oplus_module = &first_module[i];
		if ((oplus_module->magic == OPLUS_CHG_MODULE_MAGIC) &&
		    (oplus_module->chg_module_exit != NULL))
			oplus_module->chg_module_exit();
	}

	return rc;
}

static void __exit oplus_chg_modules_exit(void)
{
	int module_num, i;
	struct oplus_chg_module *first_module;
	struct oplus_chg_module *oplus_module;

	module_num = oplus_chg_get_module_num();
	if (module_num == 0)
		return;

	first_module = oplus_chg_find_first_module();
	for (i = module_num - 1; i >= 0; i--) {
		oplus_module = &first_module[i];
		if ((oplus_module->magic == OPLUS_CHG_MODULE_MAGIC) &&
		    (oplus_module->chg_module_exit != NULL))
			oplus_module->chg_module_exit();
	}
}

#else /* (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)) */

oplus_chg_module_register_null(oplus_chg_normal);
oplus_chg_module_core_register_null(oplus_chg_core);
oplus_chg_module_early_register_null(oplus_chg_early);
oplus_chg_module_late_register_null(oplus_chg_late);

static size_t oplus_chg_get_flag_module(uint32_t magic)
{
	switch (magic) {
	case OPLUS_CHG_MODULE_NORMAL_MAGIC:
		return (size_t)&oplus_chg_normal_module;
	case OPLUS_CHG_MODULE_CORE_MAGIC:
		return (size_t)&oplus_chg_core_module;
	case OPLUS_CHG_MODULE_EARLY_MAGIC:
		return (size_t)&oplus_chg_early_module;
	case OPLUS_CHG_MODULE_LATE_MAGIC:
		return (size_t)&oplus_chg_late_module;
	default:
		return 0;
	}
}

static size_t oplus_chg_find_first_module(uint32_t magic)
{
	struct oplus_chg_module *tmp;
	size_t start_addr, next_addr;

	start_addr = oplus_chg_get_flag_module(magic);
	if (start_addr == 0)
		return 0;
	tmp = (struct oplus_chg_module *)start_addr;
	if (READ_ONCE_NOCHECK(tmp->magic) != magic) {
		chg_err("%s: magic error\n", tmp->name);
		return 0;
	}

	do {
		next_addr = start_addr - sizeof(struct oplus_chg_module);
		tmp = (struct oplus_chg_module *)next_addr;
		if (READ_ONCE_NOCHECK(tmp->magic) != magic)
			return start_addr;
		start_addr = next_addr;
	} while (true);

	return 0;
}

static size_t oplus_chg_find_last_module(uint32_t magic)
{
	struct oplus_chg_module *tmp;
	size_t start_addr, next_addr;

	start_addr = oplus_chg_get_flag_module(magic);
	if (start_addr == 0)
		return 0;
	tmp = (struct oplus_chg_module *)start_addr;
	if (READ_ONCE_NOCHECK(tmp->magic) != magic) {
		chg_err("%s: magic error\n", tmp->name);
		return 0;
	}

	do {
		next_addr = start_addr + sizeof(struct oplus_chg_module);
		tmp = (struct oplus_chg_module *)next_addr;
		if (READ_ONCE_NOCHECK(tmp->magic) != magic)
			return start_addr;
		start_addr = next_addr;
	} while (true);

	return 0;
}

static int oplus_chg_section_modules_init(uint32_t magic)
{
	int i = 0;
	struct oplus_chg_module *module;
	size_t start_addr, tmp;
	int rc;
	chg_module_init_t module_init_fn;
	chg_module_exit_t module_exit_fn;

	start_addr = oplus_chg_find_first_module(magic);
	if (start_addr == 0)
		return 0;

	do {
		tmp = start_addr + i * sizeof(struct oplus_chg_module);
		module = (struct oplus_chg_module *)tmp;
		if (READ_ONCE_NOCHECK(module->magic) != magic)
			return 0;
		module_init_fn = READ_ONCE_NOCHECK(module->chg_module_init);
		if (module_init_fn == NULL) {
			i++;
			continue;
		}
		chg_info("%s init\n", module->name);
		rc = module_init_fn();
		if (rc < 0) {
			chg_err("%s init error, rc=%d\n", module->name, rc);
			goto module_init_err;
		}
		i++;
	} while (true);

	return 0;

module_init_err:
	for (i = i - 1; i >= 0; i--) {
		tmp = start_addr + i * sizeof(struct oplus_chg_module);
		module = (struct oplus_chg_module *)tmp;
		module_exit_fn = READ_ONCE_NOCHECK(module->chg_module_exit);
		if (module_exit_fn == NULL)
			continue;
		module_exit_fn();
	}
	return rc;
}

static void oplus_chg_section_modules_exit(uint32_t magic)
{
	int i = 0;
	struct oplus_chg_module *module;
	size_t start_addr, tmp;
	chg_module_exit_t module_exit_fn;

	start_addr = oplus_chg_find_last_module(magic);
	if (start_addr == 0)
		return;

	do {
		tmp = start_addr - i * sizeof(struct oplus_chg_module);
		module = (struct oplus_chg_module *)tmp;
		if (READ_ONCE_NOCHECK(module->magic) != magic)
			return;
		i++;
		module_exit_fn = READ_ONCE_NOCHECK(module->chg_module_exit);
		if (module_exit_fn == NULL)
			continue;
		module_exit_fn();
	} while (true);
}

static int __init oplus_chg_modules_init(void)
{
	int rc;

	rc = oplus_chg_section_modules_init(OPLUS_CHG_MODULE_CORE_MAGIC);
	if (rc < 0)
		return rc;
	rc = oplus_chg_section_modules_init(OPLUS_CHG_MODULE_EARLY_MAGIC);
	if (rc < 0)
		goto early_init_err;
	rc = oplus_chg_section_modules_init(OPLUS_CHG_MODULE_NORMAL_MAGIC);
	if (rc < 0)
		goto normal_init_err;
	rc = oplus_chg_section_modules_init(OPLUS_CHG_MODULE_LATE_MAGIC);
	if (rc < 0)
		goto late_init_err;

	return 0;

late_init_err:
	oplus_chg_section_modules_exit(OPLUS_CHG_MODULE_NORMAL_MAGIC);
normal_init_err:
	oplus_chg_section_modules_exit(OPLUS_CHG_MODULE_EARLY_MAGIC);
early_init_err:
	oplus_chg_section_modules_exit(OPLUS_CHG_MODULE_CORE_MAGIC);
	return rc;
}

static void __exit oplus_chg_modules_exit(void)
{
	oplus_chg_section_modules_exit(OPLUS_CHG_MODULE_LATE_MAGIC);
	oplus_chg_section_modules_exit(OPLUS_CHG_MODULE_NORMAL_MAGIC);
	oplus_chg_section_modules_exit(OPLUS_CHG_MODULE_EARLY_MAGIC);
	oplus_chg_section_modules_exit(OPLUS_CHG_MODULE_CORE_MAGIC);
}

#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)) */

#endif /* MODULE */

static int __init oplus_chg_class_init(void)
{
#ifdef MODULE
#if __and(IS_MODULE(CONFIG_OPLUS_CHG), IS_MODULE(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return 0;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return 0;
#endif /* CONFIG_OPLUS_CHG_V2 */
	return oplus_chg_modules_init();
#else /* MODULE */
	return 0;
#endif /* MODULE */
}

static void __exit oplus_chg_class_exit(void)
{
#ifdef MODULE
#if __and(IS_MODULE(CONFIG_OPLUS_CHG), IS_MODULE(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return;
#endif /* CONFIG_OPLUS_CHG_V2 */
	oplus_chg_modules_exit();
#endif /* MODULE */
}

subsys_initcall(oplus_chg_class_init);
module_exit(oplus_chg_class_exit);

MODULE_DESCRIPTION("oplus charge management subsystem");
MODULE_LICENSE("GPL");
