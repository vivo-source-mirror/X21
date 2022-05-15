
#ifndef _UAPI__SOUND_SNDINFO_VIVO_H
#define _UAPI__SOUND_SNDINFO_VIVO_H

/* Mic infomation */
#define BBK_SND_BUILTIN_MIC_NUM_MASK	(7 << 0)
#define BBK_SND_BUILTIN_MIC_NUM_SHIFT	0
#define BBK_SND_ANC_MIC_SUPPORT_MASK	8
#define BBK_SND_ANC_MIC_SUPPORT_SHIFT	3
/* HiFi infomation */
#define BBK_SND_HIFI_SUPPORT_MASK	(1 << 4)
#define BBK_SND_HIFI_SUPPORT_SHIFT	4
#define BBK_SND_HIFI_ALWAYS_ON_MASK	(1 << 5)
#define BBK_SND_HIFI_ALWAYS_ON_SHIFT	5
#define BBK_SND_HIFI_DOUBLE_DAC_MASK	(1 << 6)
#define BBK_SND_HIFI_DOUBLE_DAC_SHIFT	6
#define BBK_SND_HIFI_ALWAYS_WITH_SWITCH_MASK	(1 << 17)	/*HiFi DAC always open,but has switch control hifi or normal*/
#define BBK_SND_HIFI_ALWAYS_WITH_SWITCH_SHIFT	17

/* SmartPA infomation */
#define BBK_SND_PA_DRIVER_TYPE_MASK  (1 << 7)		/* 0 means user-space driver, 1 means alsa driver */
#define BBK_SND_PA_DRIVER_TYPE_SHIFT  7
#define BBK_SND_SMARTPA_NUM_MASK  (3 << 8)
#define BBK_SND_SMARTPA_NUM_SHIFT  8
#define BBK_SND_PA_MANUFACTURER_MASK  (3 << 10)		/* 0 means nxp, 1 means maxim */
#define BBK_SND_PA_MANUFACTURER_SHIFT  10
/* KTV support */
#define BBK_SND_KTV_SUPPORT_MASK	(1 << 12)
#define BBK_SND_KTV_SUPOORT_SHIFT	12
#define BBK_SND_KTV_SOLUTION_MASK	(3 << 13)		/* 0 means yamaha, 1 means qcom adsp solution */
#define BBK_SND_KTV_SOLUTION_SHIFT	13
/* FM infomation */
#define BBK_SND_FM_SUPPORT_MASK		(1 << 15)
#define BBK_SND_FM_SUPPORT_SHIFT	15
#define BBK_SND_FM_PCM_SPK_MASK		(0xFF << 0)
#define BBK_SND_FM_PCM_SPK_SHIFT	0
#define BBK_SND_FM_PCM_HP_MASK		(0xFF << 8)
#define BBK_SND_FM_PCM_HP_SHIFT		8

/* This bit for check data validation */
#define BBK_SND_INFO_READY_MASK		(1 << 31)
#define BBK_SND_INFO_READY_SHIFT	31

#define BBK_SND_CARD_MASK \
		( BBK_SND_BUILTIN_MIC_NUM_MASK | BBK_SND_ANC_MIC_SUPPORT_MASK \
			| BBK_SND_HIFI_SUPPORT_MASK | BBK_SND_HIFI_ALWAYS_ON_MASK \
			| BBK_SND_HIFI_DOUBLE_DAC_MASK | BBK_SND_PA_DRIVER_TYPE_MASK \
			| BBK_SND_SMARTPA_NUM_MASK | BBK_SND_PA_MANUFACTURER_MASK \
			| BBK_SND_KTV_SUPPORT_MASK | BBK_SND_KTV_SOLUTION_MASK \
			| BBK_SND_FM_SUPPORT_MASK )
#define BBK_FM_INFO_MASK        (BBK_SND_FM_PCM_SPK_MASK | BBK_SND_FM_PCM_HP_MASK)

#endif
