# Copyright (C), 2008-2030, OPPO Mobile Comm Corp., Ltd
### All rights reserved.
###
### File: - OplusKernelEnvConfig.mk
### Description:
###     you can get the oplus feature variables set in android side in this file
###     this file will add global macro for common oplus added feature
###     BSP team can do customzation by referring the feature variables
### Version: 1.0
### Date: 2020-03-18
### Author: Liang.Sun
###
### ------------------------------- Revision History: ----------------------------
### <author>                        <date>       <version>   <desc>
### ------------------------------------------------------------------------------
### Liang.Sun@TECH.Build              2020-03-18   1.0         Create this moudle
##################################################################################

-include nativefeatures.mk

###ifdef OPLUS_ARCH_INJECT
OPLUS_CONNECTIVITY_NATIVE_FEATURE_SET :=

##Add OPLUS Debug/Feature Macro Support for kernel/driver
##ifeq ($(OPLUS_FEATURE_TEST), yes)
## OPLUS_CONNECTIVITY_NATIVE_FEATURE_SET += OPLUS_FEATURE_TEST
##endif

ifeq ($(OPLUS_FEATURE_WIFI_MTUDETECT), yes)
OPLUS_CONNECTIVITY_NATIVE_FEATURE_SET += OPLUS_FEATURE_WIFI_MTUDETECT
endif

$(foreach myfeature,$(OPLUS_CONNECTIVITY_NATIVE_FEATURE_SET),\
    $( \
        $(eval KBUILD_CFLAGS += -D$(myfeature)) \
        $(eval KBUILD_CPPFLAGS += -D$(myfeature)) \
        $(eval CFLAGS_KERNEL += -D$(myfeature)) \
        $(eval CFLAGS_MODULE += -D$(myfeature)) \
    ) \
)
###endif OPLUS_ARCH_INJECT

ALLOWED_MCROS := OPLUS_FEATURE_FG_IO_OPT  \
OPLUS_FEATURE_SPECIALOPT \
OPLUS_FEATURE_PERFORMANCE  \
OPLUS_FEATURE_STORAGE_TOOL  \
OPLUS_FEATURE_UFS_DRIVER  \
OPLUS_FEATURE_UFS_SHOW_LATENCY \
OPLUS_FEATURE_UFSPLUS  \
OPLUS_FEATURE_SDCARD_INFO \
OPLUS_FEATURE_SDCARD_OMMC \
OPLUS_FEATURE_PADL_STATISTICS \
OPLUS_FEATURE_EMMC_SDCARD_OPTIMIZE  \
OPLUS_FEATURE_EMMC_SDCARD_COMPATIBLE \
OPLUS_FEATURE_EMMC_DRIVER  \
OPLUS_FEATURE_TASK_CPUSTATS \
OPLUS_FEATURE_HANS_FREEZE  \
OPLUS_FEATURE_SCHED_ASSIST  \
OPLUS_FEATURE_IOMONITOR  \
OPLUS_FEATURE_TP_BSPFWUPDATE  \
OPLUS_FEATURE_LOWMEM_DBG  \
OPLUS_FEATURE_QCOM_PMICWD  \
OPLUS_FEATURE_CHG_BASIC  \
OPLUS_FEATURE_FP_R_BASIC  \
OPLUS_FEATURE_NWPOWER  \
OPLUS_FEATURE_WIFI_BDF  \
OPLUS_FEATURE_CONNFCSOFT  \
OPLUS_FEATURE_AGINGTEST  \
OPLUS_FEATURE_SENSOR_SMEM  \
OPLUS_FEATURE_SSR  \
OPLUS_FEATURE_TP_BASIC  \
OPLUS_FEATURE_EDTASK_IMPROVE  \
OPLUS_FEATURE_WIFI_SLA  \
OPLUS_FEATURE_WIFI_ROUTERBOOST \
OPLUS_FEATURE_DATA_EVAL  \
OPLUS_FEATURE_DHCP  \
OPLUS_FEATURE_PHOENIX  \
OPLUS_FEATURE_DUMPDEVICE  \
OPLUS_FEATURE_SAUPWK  \
OPLUS_FEATURE_MEMORY_ISOLATE  \
OPLUS_FEATURE_MULTI_KSWAPD  \
OPLUS_FEATURE_WIFI_MTUDETECT  \
OPLUS_FEATURE_XTOWNER_INPUT  \
OPLUS_FEATURE_SELINUX_CONTROL_LOG  \
OPLUS_FEATURE_MULTI_FREEAREA  \
OPLUS_FEATURE_VIRTUAL_RESERVE_MEMORY  \
OPLUS_FEATURE_GPU_MINIDUMP  \
OPLUS_FEATURE_PROCESS_RECLAIM  \
OPLUS_FEATURE_ZRAM_OPT \
OPLUS_FEATURE_AUDIO_FTM \
OPLUS_FEATURE_AUDIODETECT \
OPLUS_FEATURE_SPEAKER_MUTE \
OPLUS_FEATURE_MM_FEEDBACK \
OPLUS_FEATURE_MI2S_SLAVE \
OPLUS_FEATURE_KTV \
OPLUS_FEATURE_QCOM_WATCHDOG \
OPLUS_FEATURE_MUTE_DETECT \
OPLUS_FEATURE_MEMLEAK_DETECT \
OPLUS_FEATURE_EXFAT_SUPPORT \
OPLUS_FEATURE_SDCARDFS_SUPPORT \
OPLUS_FEATURE_CAMERA_OIS  \
OPLUS_BUG_COMPATIBILITY \
OPLUS_BUG_STABILITY \
OPLUS_BUG_DEBUG \
OPLUS_ARCH_INJECT \
OPLUS_ARCH_EXTENDS \
COLOROS_EDIT \
OPLUS_FEATURE_DC \
OPLUS_FEATURE_POWERINFO_FTM \
OPLUS_FEATURE_POWERINFO_STANDBY \
OPLUS_FEATURE_POWERINFO_STANDBY_DEBUG \
OPLUS_FEATURE_POWERINFO_RPMH \
OPLUS_FEATURE_CAMERA_COMMON \
OPLUS_FEATURE_ADSP_RECOVERY \
OPLUS_FEATURE_MODEM_MINIDUMP \
OPLUS_FEATURE_THEIA \
OPLUS_FEATURE_POWER_CPUFREQ \
OPLUS_FEATURE_MIDAS \
OPLUS_FEATURE_WIFI_OPLUSWFD \
OPLUS_FEATURE_WIFI_DUALSTA_AP_BLACKLIST \
OPLUS_FEATURE_BOOT \
OPLUS_FEATURE_HANS_FREEZE \
OPLUS_FEATURE_HEALTHINFO

KBUILD_CFLAGS += -DOPLUS_FEATURE_KTV

KBUILD_CFLAGS += -DOPLUS_AUDIO_PA_BOOST_VOLTAGE

KBUILD_CFLAGS += -DOPLUS_FEATURE_ADSP_RECOVERY

$(foreach myfeature,$(ALLOWED_MCROS),\
         $(warning make $(myfeature) to be a macro here) \
         $(eval KBUILD_CFLAGS += -D$(myfeature)) \
         $(eval KBUILD_CPPFLAGS += -D$(myfeature)) \
         $(eval CFLAGS_KERNEL += -D$(myfeature)) \
         $(eval CFLAGS_MODULE += -D$(myfeature)) \
)

# BSP team can do customzation by referring the feature variables

ifeq ($(OPLUS_FEATURE_PADL_STATISTICS),yes)
export OPLUS_FEATURE_PADL_STATISTICS=yes
endif

ifeq ($(OPLUS_FEATURE_UFSPLUS),yes)
export OPLUS_FEATURE_UFSPLUS=yes
endif

ifeq ($(OPLUS_FEATURE_UFS_SHOW_LATENCY),yes)
export OPLUS_FEATURE_UFS_SHOW_LATENCY=yes
endif

ifeq ($(OPLUS_FEATURE_SECURE_GUARD),yes)
export CONFIG_OPLUS_SECURE_GUARD=y
KBUILD_CFLAGS += -DCONFIG_OPLUS_SECURE_GUARD
endif

ifeq ($(OPLUS_FEATURE_SECURE_ROOTGUARD),yes)
export CONFIG_OPLUS_ROOT_CHECK=y
KBUILD_CFLAGS += -DCONFIG_OPLUS_ROOT_CHECK
endif

ifeq ($(OPLUS_FEATURE_SECURE_MOUNTGUARD),yes)
export CONFIG_OPLUS_MOUNT_BLOCK=y
KBUILD_CFLAGS += -DCONFIG_OPLUS_MOUNT_BLOCK
endif

ifeq ($(OPLUS_FEATURE_SECURE_EXECGUARD),yes)
export CONFIG_OPLUS_EXECVE_BLOCK=y
KBUILD_CFLAGS += -DCONFIG_OPLUS_EXECVE_BLOCK
KBUILD_CFLAGS += -DCONFIG_OPLUS_EXECVE_REPORT
endif

ifeq ($(OPLUS_FEATURE_SECURE_KEVENTUPLOAD),yes)
export CONFIG_OPLUS_KEVENT_UPLOAD=y
KBUILD_CFLAGS += -DCONFIG_OPLUS_KEVENT_UPLOAD
endif

ifeq ($(OPLUS_FEATURE_SECURE_KEYINTERFACESGUARD),yes)
KBUILD_CFLAGS += -DOPLUS_DISALLOW_KEY_INTERFACES
endif

ifeq ($(OPLUS_FEATURE_POWERINFO_FTM),yes)
KBUILD_CFLAGS += -DOPLUS_FEATURE_POWERINFO_FTM
KBUILD_CFLAGS += -DCONFIG_OPLUS_POWERINFO_FTM
endif

ifeq ($(OPLUS_FEATURE_AOD_RAMLESS),yes)
KBUILD_CFLAGS += -DOPLUS_FEATURE_AOD_RAMLESS
KBUILD_CPPFLAGS += -DOPLUS_FEATURE_AOD_RAMLESS
CFLAGS_KERNEL += -DOPLUS_FEATURE_AOD_RAMLESS
CFLAGS_MODULE += -DOPLUS_FEATURE_AOD_RAMLESS
endif

ifeq ($(OPLUS_FEATURE_POWERINFO_STANDBY),yes)
#/* add for feature powerinfo standby */
KBUILD_CFLAGS += -DOPLUS_FEATURE_POWERINFO_STANDBY
KBUILD_CFLAGS += -DOPLUS_FEATURE_POWERINFO_STANDBY_DEBUG
KBUILD_CFLAGS += -DOPLUS_FEATURE_POWERINFO_RPMH

KBUILD_CFLAGS += -DCONFIG_OPLUS_POWER_QCOM
KBUILD_CFLAGS += -DCONFIG_OPLUS_WAKELOCK_PROFILER
KBUILD_CFLAGS += -DCONFIG_OPLUS_POWERINFO_STANDBY
KBUILD_CFLAGS += -DCONFIG_OPLUS_POWERINFO_STANDBY_DEBUG
KBUILD_CFLAGS += -DCONFIG_OPLUS_POWERINFO_RPMH

inner_mk_path := $(abspath $(lastword $(MAKEFILE_LIST)))
inner_mk_dir := $(shell dirname $(inner_mk_path))
inner_oplus_wakelock_dir1 := $(inner_mk_dir)/../vendor/oplus/kernel/wakelock_profiler
inner_oplus_wakelock_dir2 := $(inner_mk_dir)/../vendor/oplus/kernel/oplus_wakelock_profiler
inner_oplus_wakelock_dir1_exist := $(shell if [ -d $(inner_oplus_wakelock_dir1) ]; then echo "exist"; else echo "notexist"; fi;)
inner_oplus_wakelock_dir2_exist := $(shell if [ -d $(inner_oplus_wakelock_dir2) ]; then echo "exist"; else echo "notexist"; fi;)

inner_oplus_link_name := oplus_wakelock
inner_oplus_link_target := $(inner_mk_dir)/../kernel/msm-4.19/drivers/soc/oplus/$(inner_oplus_link_name)
ifeq ($(inner_oplus_wakelock_dir1_exist), exist)
$(warning "source is $(inner_oplus_wakelock_dir1)")
$(shell ln -sf $(inner_oplus_wakelock_dir1) $(inner_oplus_link_target))
else ifeq ($(inner_oplus_wakelock_dir2_exist), exist)
$(warning "source is $(inner_oplus_wakelock_dir2)")
$(shell ln -sf $(inner_oplus_wakelock_dir2) $(inner_oplus_link_target))
else
$(warning "source is nothing")
endif
endif
