load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/kleaf:hermetic_tools.bzl", "hermetic_genrule")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module",
    "oplus_ddk_get_kernel_version",
    "bazel_support_platform")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")
load(":kleaf-scripts/version.bzl", "version_compare")

load(":kleaf-scripts/prebuild.bzl", "oplus_chg_ic_prebuild")
load(":kleaf-scripts/targets.bzl", "oplus_modules_get_target_variant")
load(":kleaf-scripts/filter_target.bzl", "filter_deps_map")

conditional_ko_deps = {
    "CONFIG_OPLUS_ADSP_CHARGER": {
        True: [
            "//soc-repo:{target_variant}/drivers/soc/qcom/panel_event_notifier",
            "//soc-repo:{target_variant}/drivers/soc/qcom/qti_pmic_glink",
            "//soc-repo:{target_variant}/drivers/soc/qcom/pdr_interface",
            "//soc-repo:{target_variant}/drivers/soc/qcom/qmi_helpers",
            "//soc-repo:{target_variant}/drivers/remoteproc/rproc_qcom_common",
            "//soc-repo:{target_variant}/drivers/rpmsg/qcom_smd",
            "//soc-repo:{target_variant}/drivers/rpmsg/qcom_glink_smem",
            "//soc-repo:{target_variant}/drivers/rpmsg/qcom_glink",
            "//soc-repo:{target_variant}/kernel/trace/qcom_ipc_logging",
            "//soc-repo:{target_variant}/drivers/soc/qcom/minidump",
            "//soc-repo:{target_variant}/drivers/soc/qcom/smem",
            "//soc-repo:{target_variant}/drivers/soc/qcom/debug_symbol",
            "//soc-repo:{target_variant}/drivers/dma-buf/heaps/qcom_dma_heaps",
            "//soc-repo:{target_variant}/drivers/iommu/msm_dma_iommu_mapping",
            "//soc-repo:{target_variant}/drivers/soc/qcom/mem_buf/mem_buf_dev",
            "//soc-repo:{target_variant}/drivers/soc/qcom/secure_buffer",
            "//soc-repo:{target_variant}/drivers/firmware/qcom/qcom-scm",
            "//soc-repo:{target_variant}/drivers/virt/gunyah/gh_rm_drv",
            "//soc-repo:{target_variant}/drivers/virt/gunyah/gh_msgq",
            "//soc-repo:{target_variant}/drivers/virt/gunyah/gh_dbl",
            "//soc-repo:{target_variant}/arch/arm64/gunyah/gh_arm_drv",
        ],
    },
    "CONFIG_OPLUS_CHARGER_MTK": {
        True: [
                "//kernel_device_modules-{}/drivers/misc/mediatek/typec/tcpc:tcpc_class".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/misc/mediatek/typec/tcpc:tcpc_mt6375".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/misc/mediatek/usb/usb20:musb_hdrc".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/gpu/drm/mediatek/mediatek_v2:mediatek-drm".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:charger_class".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:adapter_class".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_charger_algorithm_class".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6357_battery".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6358_battery".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6375-battery".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6375-charger".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6379-battery".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mt6379-chg".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_2p_charger".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_battery_manager".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_chg_type_det".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_hvbpc".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pd_adapter".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pd_charging".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep20".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep40".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep45".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep50".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:mtk_pep50p".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:rt9490-charger".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:rt9758-charger".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/power/supply:rt9759".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/usb/mtu3:mtu3".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/regulator:mt6368-regulator".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/gpu/drm/mediatek/mediatek_v2:mtk_disp_notify".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/base/magtransfer:oplus_magcvr_notify".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/soc/oplus/device_info:device_info".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/soc/oplus/boot:oplus_bsp_boot_projectinfo".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/misc/mediatek/boot_common:mtk_boot_common".format(oplus_ddk_get_kernel_version()),
                "//kernel_device_modules-{}/drivers/base/kernelFwUpdate:oplus_bsp_fw_update".format(oplus_ddk_get_kernel_version()),
        ],
    },
    "CONFIG_OPLUS_CHG_TEST_KIT": {
        True: [
            "{target_variant}_test-kit"
        ],
    },
    "CONFIG_OPLUS_DYNAMIC_CONFIG": {
        True: [
            "{target_variant}_oplus_cfg"
        ],
    },
    "CONFIG_OPLUS_UFCS_CLASS": {
        True: [
            "{target_variant}_ufcs_class"
        ],
    },
    "CONFIG_DISABLE_OPLUS_FUNCTION": {
        False: [
            "//vendor/oplus/kernel/device_info/device_info/bazel:device_info",
            "//vendor/oplus/kernel/boot:oplus_bsp_bootmode",
            "//vendor/oplus/kernel/boot:oplus_bsp_boot_projectinfo",
            "//vendor/oplus/kernel/touchpanel/kernelFwUpdate/bazel:oplus_bsp_fw_update",
        ],
    },
    "CONFIG_OPLUS_MAGCVR_NOTIFY": {
        True: [
            "//vendor/oplus/kernel/device_info/magtransfer:oplus_magcvr_notify",
        ],
    },
    "CONFIG_OPLUS_FEATURE_FEEDBACK": {
        True: [
             "//vendor/oplus/kernel/dft/bazel:oplus_bsp_dft_kernel_fb",
        ],
    },
    "CONFIG_OPLUS_FEATURE_OLC": {
        True: [
             "//vendor/oplus/kernel/dft/bazel:oplus_bsp_dft_olc",
        ],
    },
}

conditional_hdr_deps = {
    "CONFIG_OPLUS_CHG_TEST_KIT": {
        True: [
            ":test_kit_headers"
        ],
    },
    "CONFIG_OPLUS_DYNAMIC_CONFIG": {
        True: [
            ":oplus_cfg_headers"
        ],
    },
    "CONFIG_OPLUS_UFCS_CLASS": {
        True: [
            ":ufcs_class_headers"
        ],
    },
    "CONFIG_OPLUS_MT6375_CHARGER": {
        True: [
            "//vendor/oplus/kernel/charger/bazel:oplus_chg_headers"
        ],
    },
    "CONFIG_DISABLE_OPLUS_FUNCTION": {
        False: [
            "//vendor/oplus/kernel/touchpanel/kernelFwUpdate/bazel:oplus_bsp_fw_update_headers",
        ],
    },
}

conditional_copt_deps = {
    "CONFIG_OPLUS_CHARGER_MTK": {
        True: [
                "-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/typec/tcpc/inc",
                "-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat",
                "-I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include",
                "-I$(DEVICE_MODULES_PATH)/drivers/power/supply",
                "-I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/mediatek_v2",
                "-I$(DEVICE_MODULES_PATH)/drivers/base/kernelFwUpdate"
        ],
    },
}

def define_oplus_chg_v2_module():
    module_list = []

    oplus_chg_ic_prebuild("oplus_chg_v2")

    ddk_headers(
        name = "oplus_chg_v2_headers",
        hdrs  = native.glob([
            "v2/include/*.h",
            "v2/config/**/*"
        ]),
        includes = [
            "v2/include",
        ]
    )

    target = oplus_modules_get_target_variant()
    ko_deps = filter_deps_map(target, conditional_ko_deps)
    hdr_deps = filter_deps_map(target, conditional_hdr_deps)
    copt_deps = filter_deps_map(target, conditional_copt_deps)
    kernel_version = oplus_ddk_get_kernel_version()

    if bazel_support_platform == "qcom":
        kconfig = None
        defconfig = None
        if version_compare(kernel_version, "6.12") :
            ddk_config = "//soc-repo:{}_config".format(target)
    else:
        kconfig = ":kconfig.oplus_chg.generated"
        defconfig = ":oplus_chg_{}_defconfig".format(target)
        if version_compare(kernel_version, "6.12") :
            ddk_config = None

    ddk_includes = [
        "v2",
        "v2/include",
        "v2/gauge_i2c_rst",
        "v2/ufcs/include",
        "v2/config",
        "v2/scripts",
        "v2/Makefile.json-build",
        "test-kit",
        "debug-kit",
        "config",
        "oplus_chg_v2_ic_cfg"
    ]

    ddk_srcs = native.glob([
        "v2/**/*.h",
        "v2/oplus_chg_core.c",
        "v2/oplus_battery_log.c",
        "v2/oplus_chg_gki.c",
        "v2/oplus_chg_voter.c",
        "v2/oplus_chg_wired.c",
        "v2/oplus_chg_comm.c",
        "v2/oplus_chg_plc.c",
        "v2/oplus_chg_vooc.c",
        "v2/oplus_configfs.c",
        "v2/oplus_chg_dual_chan.c",
        "v2/oplus_chg_cpa.c",
        "v2/oplus_impedance_check.c",
        "v2/oplus_chg_ufcs.c",
        "v2/oplus_chg_wls.c",
        "v2/oplus_smart_chg.c",
        "v2/oplus_chg_pps.c",
        "v2/oplus_batt_bal.c",
        "v2/oplus_chg_mutual.c",
        "v2/oplus_reverse_chg.c",
        "v2/oplus_chg_dual_cells_protection.c",
        "v2/gauge_ic/oplus_hal_bq27541.c",
        "v2/hal/oplus_chg_ic.c",
        "v2/hal/oplus_virtual_buck.c",
        "v2/hal/oplus_virtual_asic.c",
        "v2/hal/oplus_virtual_gauge.c",
        "v2/hal/oplus_virtual_voocphy.c",
        "v2/hal/oplus_virtual_cp.c",
        "v2/hal/oplus_virtual_rx.c",
        "v2/hal/oplus_hal_wls.c",
        "v2/hal/oplus_hal_vooc.c",
        "v2/hal/oplus_virtual_dpdm_switch.c",
        "v2/hal/oplus_virtual_pps.c",
        "v2/hal/oplus_virtual_ufcs.c",
        "v2/hal/oplus_virtual_platufcs.c",
        "v2/hal/oplus_virtual_batt_bal.c",
        "v2/hal/oplus_virtual_level_shift.c",
        "v2/hal/oplus_virtual_reverse_chg.c",
        "v2/hal/oplus_virtual_boost.c",
        "v2/mms/oplus_mms.c",
        "v2/mms/oplus_msg_filter.c",
        "v2/mms/oplus_mms_gauge.c",
        "v2/mms/oplus_mms_wired.c",
        "v2/mms/gauge/oplus_sili.c",
        "v2/mms/sec/oplus_sec.c",
        "v2/strategy/oplus_strategy.c",
        "v2/strategy/oplus_strategy_cgcl.c",
        "v2/strategy/oplus_strategy_inr_switch.c",
        "v2/strategy/oplus_strategy_pps_ufcs_curve.c",
        "v2/strategy/oplus_strategy_low_curr_full.c",
        "v2/strategy/oplus_strategy_pps_ufcs_curve_v2.c",
        "v2/strategy/oplus_strategy_cycle_current_derating.c",
        "v2/strategy/oplus_strategy_ddrc.c",
        "v2/strategy/oplus_strategy_ddrc_v2.c",
        "v2/strategy/oplus_strategy_battery_smooth.c",
        "v2/strategy/oplus_strategy_volt_fastchg_allow.c",
        "v2/strategy/oplus_strategy_pcc.c",
        "v2/strategy/oplus_strategy_pcc_v2.c",
        "v2/monitor/oplus_monitor_core.c",
        "v2/monitor/oplus_chg_track.c",
        "v2/monitor/oplus_chg_exception.c",
        "v2/plat_ufcs/plat_ufcs_notify.c",
        "v2/oplus_dischg_boost.c",
        "v2/oplus_gamepad.c"
    ]) + [
        ":oplus_chg_v2_ic_cfg"
    ]

    ddk_conditional_srcs = {
        "CONFIG_OPLUS_CHG_PARALLEL": {
            True: [
                "v2/oplus_parallel.c",
                "v2/hal/oplus_virtual_switching.c"
            ],
        },
        "CONFIG_OPLUS_GAUGE_MPC7022": {
            True: [
                "v2/gauge_ic/oplus_hal_mpc7022.c"
            ],
        },
        "CONFIG_OPLUS_GAUGE_SH366002": {
            True: [
                "v2/gauge_ic/oplus_hal_sh366002.c"
            ],
        },
        "CONFIG_OPLUS_GAUGE_BQ27Z561": {
            True: [
                "v2/gauge_ic/oplus_hal_bq27z561.c"
            ],
        },
        "CONFIG_OPLUS_GAUGE_NFG8011B": {
            True: [
                "v2/gauge_ic/oplus_hal_nfg8011b.c"
            ],
        },
        "CONFIG_OPLUS_GAUGE_SN28Z729": {
            True: [
                "v2/gauge_ic/oplus_hal_sn28z729.c"
            ],
        },
        "CONFIG_OPLUS_GAUGE_BQ28Z610_RA": {
            True: [
                "v2/gauge_ic/oplus_hal_bq28z610.c"
            ],
        },
        "CONFIG_OPLUS_CHG_IC_DEBUG": {
            True: [
                "v2/hal/debug/oplus_chg_ic_debug.c"
            ],
        },
        "CONFIG_OPLUS_STATE_RETENTION": {
            True: [
                "v2/oplus_chg_quirks.c",
                "v2/oplus_chg_state_retention.c"
            ],
        },
        "CONFIG_OPLUS_DA9313_CHARGER": {
            True: [
                "v2/charger_ic/oplus_da9313.c"
            ],
        },
        "CONFIG_OPLUS_ADSP_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_adsp.c"
            ],
        },
        "CONFIG_OPLUS_MP2762_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_mp2650.c"
            ],
        },
        "CONFIG_OPLUS_SGM41512_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_sgm41512.c"
            ],
        },
        "CONFIG_OPLUS_SY6974B_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_sy6974b.c"
            ],
        },
        "CONFIG_OPLUS_PD_MANAGER_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_pd_manager.c"
            ],
        },
        "CONFIG_OPLUS_MT6375_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_mtk6895S.c",
                "v2/charger_ic/oplus_hal_mt6375.c",
                "v2/gauge_ic/oplus_hal_mtk_platform_gauge.c"
            ],
        },
        "CONFIG_OPLUS_MT6835_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_mtk6895S.c",
                "v2/gauge_ic/oplus_hal_mtk_platform_gauge.c"
            ],
        },
        "CONFIG_OPLUS_MT6379_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_mtk6991V.c"
            ],
        },
        "CONFIG_OPLUS_TPS6128XD_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_tps6128xd.c"
            ],
        },
        "CONFIG_OPLUS_HL7603_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_hl7603.c"
            ],
        },
        "CONFIG_OPLUS_SY6603_BATT_BAL": {
            True: [
                "v2/batt_bal_ic/oplus_hal_sy6603.c"
            ],
        },
        "CONFIG_OPLUS_SC7637_LEVEL_SHIFT": {
            True: [
                "v2/level_shift_ic/oplus_hal_sc7637.c"
            ],
        },
        "CONFIG_OPLUS_CHG_AP_VOOCPHY": {
            True: [
                "v2/voocphy/oplus_ap_voocphy.c"
            ],
        },
        "CONFIG_OPLUS_CHG_ADSP_VOOCPHY": {
            True: [
                "v2/voocphy/oplus_adsp_voocphy.c"
            ],
        },
        "CONFIG_OPLUS_CHG_VOOCPHY_CHGLIB": {
            True: [
                "v2/voocphy/chglib/oplus_chglib.c"
            ],
        },
        "CONFIG_OPLUS_VOOCPHY_MASTER_SC8547": {
            True: [
                "v2/voocphy/phy/oplus_sc8547.c"
            ],
        },
        "CONFIG_OPLUS_VOOCPHY_SLAVE_SC8547": {
            True: [
                "v2/voocphy/phy/oplus_sc8547_slave.c"
            ],
        },
        "CONFIG_OPLUS_VOOCPHY_SC8517": {
            True: [
                "v2/voocphy/phy/oplus_sc8517.c"
            ],
        },
        "CONFIG_OPLUS_VOOCPHY_SC8527": {
            True: [
                "v2/voocphy/phy/oplus_sc8527.c"
            ],
        },
        "CONFIG_OPLUS_VOOCPHY_MAX77939": {
            True: [
                "v2/voocphy/phy/oplus_max77939.c"
            ],
        },
        "CONFIG_OPLUS_VOOCPHY_HL7138": {
            True: [
                "v2/voocphy/phy/oplus_hl7138.c"
            ],
        },
        "CONFIG_OPLUS_VOOCPHY_SLAVE_HL7138": {
            True: [
                "v2/voocphy/phy/oplus_hl7138_slave.c"
            ],
        },
        "CONFIG_OPLUS_VOOCPHY_SC8547A": {
            True: [
                "v2/voocphy/phy/oplus_sc8547a.c"
            ],
        },
        "CONFIG_OPLUS_UFCS_MASTER_NU2112A": {
            True: [
                "v2/ufcs_ic/oplus_hal_nu2112a.c"
            ],
        },
        "CONFIG_OPLUS_UFCS_SLAVE_NU2112A": {
            True: [
                "v2/ufcs_ic/oplus_hal_nu2112a_slave.c"
            ],
        },
        "CONFIG_OPLUS_UFCS_MASTER_NU2118A": {
            True: [
                "v2/ufcs_ic/oplus_hal_nu2118a.c"
            ],
        },
        "CONFIG_OPLUS_CHG_MOS_CTRL": {
            True: [
                "v2/switching_ic/oplus_mos_ctrl.c"
            ],
        },
        "CONFIG_OPLUS_UFCS_SC2201": {
            True: [
                "v2/ufcs_ic/oplus_hal_sc2201.c"
            ],
        },
        "CONFIG_OPLUS_UFCS_SC8547A": {
            True: [
                "v2/ufcs_ic/oplus_hal_sc8547a.c"
            ],
        },
        "CONFIG_OPLUS_PHY_SC8547D": {
            True: [
                "v2/ufcs_ic/oplus_hal_sc8547d.c"
            ],
        },
        "CONFIG_OPLUS_WIRELESS_NU1619": {
            True: [
                "v2/wireless_ic/oplus_hal_nu1619.c"
            ],
        },
        "CONFIG_OPLUS_WIRELESS_NU1669": {
            True: [
                "v2/wireless_ic/oplus_hal_nu1669.c"
            ],
        },
        "CONFIG_OPLUS_WIRELESS_P9415": {
            True: [
                "v2/wireless_ic/oplus_hal_p9415.c"
            ],
        },
        "CONFIG_OPLUS_WIRELESS_SC96257": {
            True: [
                "v2/wireless_ic/oplus_hal_sc96257.c"
            ],
        },
        "CONFIG_OPLUS_CHARGEPUMP_HL7227": {
            True: [
                "v2/chargepump_ic/oplus_hal_hl7227.c"
            ],
        },
        "CONFIG_OPLUS_BOOST_SC83107": {
            True: [
                "v2/boost_ic/oplus_sc83107.c"
            ],
        },
        "CONFIG_OPLUS_SEC_IC_SC5891": {
            True: [
                "v2/gauge_ic/sc5891/oplus_hal_sc5891.c",
                "v2/gauge_ic/sc5891/uecc_lib/uecc_wrapper.c",
                "v2/gauge_ic/sc5891/uecc_lib/sha256.c",
                "v2/gauge_ic/sc5891/uecc_lib/micro_ecc/uecc.c"
            ],
        },
        "CONFIG_OPLUS_CHARGER_MAXIM": {
            True: [
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/1wire_protocol.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/bignum.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/deep_cover_coproc_sw.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/ds28e30.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/ecc_generate_key.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/ecdsa_generic_api.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/ecdsa_high.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/sha256_stone.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/ucl_rng.c",
                "v2/gauge_ic/oplus_maxim/oplus_ds28e30/ucl_sha256.c",
                "v2/gauge_ic/oplus_maxim/oplus_maxim.c"
            ],
        },
        "CONFIG_OPLUS_CHARGER_OPTIGA": {
            True: [
                "v2/gauge_ic/oplus_optiga/ECC/Optiga_Ecc.c",
                "v2/gauge_ic/oplus_optiga/ECC/Optiga_Math.c",
                "v2/gauge_ic/oplus_optiga/Platform/board.c",
                "v2/gauge_ic/oplus_optiga/SWI/Optiga_Auth.c",
                "v2/gauge_ic/oplus_optiga/SWI/Optiga_Nvm.c",
                "v2/gauge_ic/oplus_optiga/SWI/Optiga_Swi.c",
                "v2/gauge_ic/oplus_optiga/oplus_optiga.c"
            ],
        },
        "CONFIG_OPLUS_SGM41515_CHARGER": {
            True: [
                "v2/charger_ic/oplus_hal_sgm41515.c"
            ],
        },
        "CONFIG_OPLUS_CHG_RECOVERY": {
            True: [
                "v2/recovery/oplus_chg_recovery.c",
            ],
        },
        "CONFIG_OPLUS_CHG_STATE_KEEP": {
            True: [
                "v2/recovery/state_keep/state_keep.c",
                "v2/recovery/state_keep/detection/wired_disconnect_detection.c",
                "v2/recovery/state_keep/detection/vooc_disconnect_detection.c",
                "v2/monitor/track/oplus_track_state_keep.c",
            ],
        },
        "CONFIG_OPLUS_DEBUG_AUTH": {
            True: [
                "v2/debug/oplus_debug_auth.c",
            ],
        },
    }


    ddk_copts = [
        "-Werror=parentheses",
        "-Werror=implicit-fallthrough",
        "-Werror=format"
    ] + copt_deps

    if version_compare(kernel_version, "6.12") :
        define_oplus_ddk_module(
            name = "{}_oplus_chg_v2".format(target),
            out = "oplus_chg_v2.ko",
            srcs = ddk_srcs,
            conditional_srcs = ddk_conditional_srcs,
            includes = ddk_includes,
            ko_deps = ko_deps,
            local_defines = [
                "OPLUS_CHG_KO_BUILD",
                "OPLUS_FEATURE_CHG_BASIC",
            ],
            conditional_defines = {
                "qcom": ["CONFIG_OPLUS_FEATURE_OLC=1"],
            },
            copts = ddk_copts,
            hdrs = hdr_deps,
            kconfig = kconfig,
            defconfig = defconfig,
            config = ddk_config,
        )
    else:
        define_oplus_ddk_module(
            name = "{}_oplus_chg_v2".format(target),
            out = "oplus_chg_v2.ko",
            srcs = ddk_srcs,
            conditional_srcs = ddk_conditional_srcs,
            includes = ddk_includes,
            ko_deps = ko_deps,
            local_defines = [
                "OPLUS_CHG_KO_BUILD",
                "OPLUS_FEATURE_CHG_BASIC"
            ],
            conditional_defines = {
            },
            copts = ddk_copts,
            hdrs = hdr_deps,
            kconfig = kconfig,
            defconfig = defconfig,
        )

    module_list.extend(filter_deps_map(target, {
        "CONFIG_OPLUS_CHG_V2": "{}_oplus_chg_v2".format(target)
    }))

    return module_list
