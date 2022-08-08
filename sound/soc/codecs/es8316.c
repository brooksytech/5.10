/*
 * es8316.c -- es8316 ALSA SoC audio driver
 * Copyright Everest Semiconductor Co.,Ltd
 *
 * Author: David Yang <yangxiaohua@everest-semi.com>
 *
 * Based on es8316.c
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include "es8316.h"

/* In slave mode at single speed, the codec is documented as accepting 5
 * MCLK/LRCK ratios, but we also add ratio 400, which is commonly used on
 * Intel Cherry Trail platforms (19.2MHz MCLK, 48kHz LRCK).
 */
#define NR_SUPPORTED_MCLK_LRCK_RATIOS 6
static const unsigned int supported_mclk_lrck_ratios[] = {
	256, 384, 400, 512, 768, 1024
};

#define ES8316_LRCK		48000

static const struct reg_default es8316_reg_defaults[] = {
	{0x00, 0x03}, {0x01, 0x03}, {0x02, 0x00}, {0x03, 0x20},
	{0x04, 0x11}, {0x05, 0x00}, {0x06, 0x11}, {0x07, 0x00},
	{0x08, 0x00}, {0x09, 0x01}, {0x0a, 0x00}, {0x0b, 0x00},
	{0x0c, 0xf8}, {0x0d, 0x3f}, {0x0e, 0x00}, {0x0f, 0x00},
	{0x10, 0x01}, {0x11, 0xfc}, {0x12, 0x28}, {0x13, 0x00},
	{0x14, 0x00}, {0x15, 0x33}, {0x16, 0x00}, {0x17, 0x00},
	{0x18, 0x88}, {0x19, 0x06}, {0x1a, 0x22}, {0x1b, 0x03},
	{0x1c, 0x0f}, {0x1d, 0x00}, {0x1e, 0x80}, {0x1f, 0x80},
	{0x20, 0x00}, {0x21, 0x00}, {0x22, 0xc0}, {0x23, 0x00},
	{0x24, 0x01}, {0x25, 0x08}, {0x26, 0x10}, {0x27, 0xc0},
	{0x28, 0x00}, {0x29, 0x1c}, {0x2a, 0x00}, {0x2b, 0xb0},
	{0x2c, 0x32}, {0x2d, 0x03}, {0x2e, 0x00}, {0x2f, 0x11},
	{0x30, 0x10}, {0x31, 0x00}, {0x32, 0x00}, {0x33, 0xc0},
	{0x34, 0xc0}, {0x35, 0x1f}, {0x36, 0xf7}, {0x37, 0xfd},
	{0x38, 0xff}, {0x39, 0x1f}, {0x3a, 0xf7}, {0x3b, 0xfd},
	{0x3c, 0xff}, {0x3d, 0x1f}, {0x3e, 0xf7}, {0x3f, 0xfd},
	{0x40, 0xff}, {0x41, 0x1f}, {0x42, 0xf7}, {0x43, 0xfd},
	{0x44, 0xff}, {0x45, 0x1f}, {0x46, 0xf7}, {0x47, 0xfd},
	{0x48, 0xff}, {0x49, 0x1f}, {0x4a, 0xf7}, {0x4b, 0xfd},
	{0x4c, 0xff}, {0x4d, 0x00}, {0x4e, 0x00}, {0x4f, 0xff},
	{0x50, 0x00}, {0x51, 0x00}, {0x52, 0x00}, {0x53, 0x00},
};

/* codec private data */
struct es8316_priv {
	struct regmap *regmap;
	unsigned int dmic_amic;
	unsigned int sysclk;
	unsigned int allowed_rates[NR_SUPPORTED_MCLK_LRCK_RATIOS];	
	struct snd_pcm_hw_constraint_list sysclk_constraints;
	struct clk *mclk;
	struct snd_soc_component *component;
	
		
	bool muted;

	int pwr_count;
};

/*
 * es8316_reset
 * write value 0xff to reg0x00, the chip will be in reset mode
 * then, writer 0x00 to reg0x00, unreset the chip
 */
static int es8316_reset(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8316_RESET, 0x3F);
	usleep_range(5000, 5500);
	return snd_soc_component_write(component, ES8316_RESET, 0x03);
}



static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(dac_vol_tlv, -9600, 50, 1);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(adc_vol_tlv, -9600, 50, 1);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(alc_max_gain_tlv, -650, 150, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(alc_min_gain_tlv, -1200, 150, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(alc_target_tlv, -1650, 150, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_RANGE(hpmixer_gain_tlv,
	0, 4, TLV_DB_SCALE_ITEM(-1200, 150, 0),
	8, 11, TLV_DB_SCALE_ITEM(-450, 150, 0),
);

static const SNDRV_CTL_TLVD_DECLARE_DB_RANGE(adc_pga_gain_tlv,
	0, 0, TLV_DB_SCALE_ITEM(-350, 0, 0),
	1, 1, TLV_DB_SCALE_ITEM(0, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(250, 0, 0),
	3, 3, TLV_DB_SCALE_ITEM(450, 0, 0),
	4, 7, TLV_DB_SCALE_ITEM(700, 300, 0),
	8, 10, TLV_DB_SCALE_ITEM(1800, 300, 0),
);


static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(hpout_vol_tlv, -4800, 1200, 0);

static const char * const ng_type_txt[] =
	{ "Constant PGA Gain", "Mute ADC Output" };
static const struct soc_enum ng_type =
	SOC_ENUM_SINGLE(ES8316_ADC_ALC_NG, 6, 2, ng_type_txt);

static const char * const adcpol_txt[] = { "Normal", "Invert" };

static const struct soc_enum adcpol =
	SOC_ENUM_SINGLE(ES8316_ADC_MUTE, 1, 2, adcpol_txt);
static const char *const dacpol_txt[] =
	{ "Normal", "R Invert", "L Invert", "L + R Invert" };
static const struct soc_enum dacpol =
	SOC_ENUM_SINGLE(ES8316_DAC_SET1, 0, 4, dacpol_txt);

static const struct snd_kcontrol_new es8316_snd_controls[] = {
	/* HP OUT VOLUME */
	SOC_DOUBLE_TLV("Headphone Playback Volume", ES8316_CPHP_ICAL_VOL,
		       4, 0, 3, 1, hpout_vol_tlv),
	/* HPMIXER VOLUME Control */
	SOC_DOUBLE_TLV("Headphone Mixer Volume", ES8316_HPMIX_VOL,
		       4, 0, 11, 0, hpmixer_gain_tlv),

	/* DAC Digital controls */
	SOC_DOUBLE_R_TLV("DAC Playback Volume", ES8316_DAC_VOLL,
			 ES8316_DAC_VOLR, 0, 0xC0, 1, dac_vol_tlv),

	SOC_SINGLE("Enable DAC Soft Ramp", ES8316_DAC_SET1, 4, 1, 1),
	SOC_SINGLE("DAC Soft Ramp Rate", ES8316_DAC_SET1, 2, 4, 0),

	SOC_ENUM("Playback Polarity", dacpol),
	SOC_SINGLE("DAC Notch Filter Switch", ES8316_DAC_SET2, 6, 1, 0),
	SOC_SINGLE("DAC Double Fs Switch", ES8316_DAC_SET2, 7, 1, 0),
	SOC_SINGLE("DAC Volume Control-LeR", ES8316_DAC_SET2, 2, 1, 0),
	SOC_SINGLE("DAC Stereo Enhancement", ES8316_DAC_SET3, 0, 7, 0),
	SOC_SINGLE("DAC Mono Mix Switch", ES8316_DAC_SET3, 3, 1, 0),

	/* +20dB D2SE PGA Control */
	SOC_SINGLE("Mic Boost Switch", ES8316_ADC_D2SEPGA, 0, 1, 0),

	/* 0-+24dB Lineinput PGA Control */
	SOC_SINGLE_TLV("ADC PGA Gain Volume", ES8316_ADC_PGAGAIN,
		       4, 10, 0, adc_pga_gain_tlv),

	/* ADC Digital  Control */
	SOC_SINGLE_TLV("ADC Capture Volume", ES8316_ADC_VOLUME,
		       0, 0xC0, 1, adc_vol_tlv),
	SOC_SINGLE("ADC Soft Ramp Switch", ES8316_ADC_MUTE, 4, 1, 0),
	SOC_ENUM("Capture Polarity", adcpol),
	SOC_SINGLE("ADC Double Fs Switch", ES8316_ADC_DMIC, 4, 1, 0),

	/* ADC ALC  Control */
	SOC_SINGLE("ALC Capture Switch", ES8316_ADC_ALC1, 6, 2, 0),
	SOC_SINGLE_TLV("ALC Capture Max Volume", ES8316_ADC_ALC1, 0, 28, 0,
		       alc_max_gain_tlv),
	SOC_SINGLE_TLV("ALC Capture Min Volume", ES8316_ADC_ALC2, 0, 28, 0,
		       alc_min_gain_tlv),
	SOC_SINGLE_TLV("ALC Capture Target Volume", ES8316_ADC_ALC3, 4, 10, 0,
		       alc_target_tlv),
	SOC_SINGLE("ALC Capture Hold Time", ES8316_ADC_ALC3, 0, 10, 0),
	SOC_SINGLE("ALC Capture Decay Time", ES8316_ADC_ALC4, 4, 10, 0),
	SOC_SINGLE("ALC Capture Attack Time", ES8316_ADC_ALC4, 0, 10, 0),
	SOC_SINGLE("ALC Capture Noise Gate Threshold", ES8316_ADC_ALC_NG, 0, 31, 0),
	SOC_ENUM("ALC Capture Noise Gate Type", ng_type),
	SOC_SINGLE("ALC Capture Noise Gate Switch", ES8316_ADC_ALC_NG, 5, 1, 0),

};

/* Analog Input Mux */
static const char * const es8316_analog_in_txt[] = {
		"lin1-rin1",
		"lin2-rin2",
		"lin1-rin1 with 20db Boost",
		"lin2-rin2 with 20db Boost"
};

static const unsigned int es8316_analog_in_values[] = { 0, 1, 2, 3 };

static const struct soc_enum es8316_analog_input_enum =
	SOC_VALUE_ENUM_SINGLE(ES8316_ADC_PDN_LINSEL, 4, 3,
			      ARRAY_SIZE(es8316_analog_in_txt),
			      es8316_analog_in_txt,
			      es8316_analog_in_values);

static const struct snd_kcontrol_new es8316_analog_in_mux_controls =
	SOC_DAPM_ENUM("Route", es8316_analog_input_enum);

/* Dmic MUX */
static const char * const es8316_dmic_txt[] = {
		"dmic disable",
		"dmic data at high level",
		"dmic data at low level",
};

static const unsigned int es8316_dmic_values[] = { 0, 1, 2 };

static const struct soc_enum es8316_dmic_src_enum =
	SOC_VALUE_ENUM_SINGLE(ES8316_ADC_DMIC, 0, 3,
			      ARRAY_SIZE(es8316_dmic_txt),
			      es8316_dmic_txt,
			      es8316_dmic_values);

static const struct snd_kcontrol_new es8316_dmic_src_controls =
	SOC_DAPM_ENUM("Route", es8316_dmic_src_enum);

/* hp mixer mux */
static const char * const es8316_hpmux_texts[] = {
	"lin1-rin1",
	"lin2-rin2",
	"lin-rin with Boost",
	"lin-rin with Boost and PGA"
};

static SOC_ENUM_SINGLE_DECL(es8316_left_hpmux_enum, ES8316_HPMIX_SEL,
	4, es8316_hpmux_texts);

static const struct snd_kcontrol_new es8316_left_hpmux_controls =
	SOC_DAPM_ENUM("Route", es8316_left_hpmux_enum);

static SOC_ENUM_SINGLE_DECL(es8316_right_hpmux_enum, ES8316_HPMIX_SEL,
	0, es8316_hpmux_texts);

static const struct snd_kcontrol_new es8316_right_hpmux_controls =
	SOC_DAPM_ENUM("Route", es8316_right_hpmux_enum);

/* headphone Output Mixer */
static const struct snd_kcontrol_new es8316_out_left_mix[] = {
	SOC_DAPM_SINGLE("LLIN Switch", ES8316_HPMIX_SWITCH, 6, 1, 0),
	SOC_DAPM_SINGLE("Left DAC Switch", ES8316_HPMIX_SWITCH, 7, 1, 0),
};

static const struct snd_kcontrol_new es8316_out_right_mix[] = {
	SOC_DAPM_SINGLE("RLIN Switch", ES8316_HPMIX_SWITCH, 2, 1, 0),
	SOC_DAPM_SINGLE("Right DAC Switch", ES8316_HPMIX_SWITCH, 3, 1, 0),
};

/* DAC data source mux */
static const char * const es8316_dacsrc_texts[] = {
	"LDATA TO LDAC, RDATA TO RDAC",
	"LDATA TO LDAC, LDATA TO RDAC",
	"RDATA TO LDAC, RDATA TO RDAC",
	"RDATA TO LDAC, LDATA TO RDAC",
};

static SOC_ENUM_SINGLE_DECL(es8316_dacsrc_mux_enum, ES8316_DAC_SET1,
	6, es8316_dacsrc_texts);

static const struct snd_kcontrol_new es8316_dacsrc_mux_controls =
	SOC_DAPM_ENUM("Route", es8316_dacsrc_mux_enum);

static const struct snd_soc_dapm_widget es8316_dapm_widgets[] = {
	/* Input Lines */
	SND_SOC_DAPM_SUPPLY("Bias", ES8316_SYS_PDN, 3, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Analog power", ES8316_SYS_PDN, 4, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Mic Bias", ES8316_SYS_PDN, 5, 1, NULL, 0),

	SND_SOC_DAPM_INPUT("DMIC"),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),

	SND_SOC_DAPM_MICBIAS("micbias", SND_SOC_NOPM,
			     0, 0),
	/* Input Mux */
	SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
			 &es8316_analog_in_mux_controls),

	SND_SOC_DAPM_PGA("Line input PGA", ES8316_ADC_PDN_LINSEL,
			 7, 1, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_SUPPLY("ADC Vref", ES8316_SYS_PDN, 1, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC bias", ES8316_SYS_PDN, 2, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC Clock", ES8316_CLKMGR_CLKSW, 3, 0, NULL, 0),
	SND_SOC_DAPM_ADC("Mono ADC", NULL, ES8316_ADC_PDN_LINSEL, 6, 1),

	/* Dmic MUX */
	SND_SOC_DAPM_MUX("Digital Mic Mux", SND_SOC_NOPM, 0, 0,
			 &es8316_dmic_src_controls),

	/* Digital Interface */
	SND_SOC_DAPM_AIF_OUT("I2S OUT", "I2S1 Capture",  1,
			     ES8316_SERDATA_ADC, 6, 0),

	SND_SOC_DAPM_AIF_IN("I2S IN", "I2S1 Playback", 0,
			    SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("DAC Source Mux", SND_SOC_NOPM, 0, 0,
			 &es8316_dacsrc_mux_controls),
	/*  DACs  */
	SND_SOC_DAPM_SUPPLY("DAC Vref", ES8316_SYS_PDN, 0, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC Clock", ES8316_CLKMGR_CLKSW, 2, 0, NULL, 0),
	SND_SOC_DAPM_DAC("Right DAC", NULL, ES8316_DAC_PDN, 0, 1),
	SND_SOC_DAPM_DAC("Left DAC", NULL, ES8316_DAC_PDN, 4, 1),

	/* Headphone Output Side */
	SND_SOC_DAPM_MUX("Left Headphone Mux", SND_SOC_NOPM, 0, 0,
			 &es8316_left_hpmux_controls),
	SND_SOC_DAPM_MUX("Right Headphone Mux", SND_SOC_NOPM, 0, 0,
			 &es8316_right_hpmux_controls),
	/* Output mixer  */
	SND_SOC_DAPM_MIXER("Left Headphone Mixer", ES8316_HPMIX_PDN,
			   5, 1, &es8316_out_left_mix[0],
			   ARRAY_SIZE(es8316_out_left_mix)),
	SND_SOC_DAPM_MIXER("Right Headphone Mixer", ES8316_HPMIX_PDN,
			   1, 1, &es8316_out_right_mix[0],
			   ARRAY_SIZE(es8316_out_right_mix)),
	SND_SOC_DAPM_MIXER("Left Headphone Mixer", SND_SOC_NOPM,
			   4, 1, &es8316_out_left_mix[0],
			   ARRAY_SIZE(es8316_out_left_mix)),
	SND_SOC_DAPM_MIXER("Right Headphone Mixer", SND_SOC_NOPM,
			   0, 1, &es8316_out_right_mix[0],
			   ARRAY_SIZE(es8316_out_right_mix)),

	/* Output charge pump */

	SND_SOC_DAPM_PGA("HPCP L", SND_SOC_NOPM,
			 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HPCP R", SND_SOC_NOPM,
			 0, 0, NULL, 0),

	SND_SOC_DAPM_PGA("HPCP L", ES8316_CPHP_OUTEN,
			     6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HPCP R", ES8316_CPHP_OUTEN,
			     2, 0, NULL, 0),

	/* Output Driver */
	SND_SOC_DAPM_PGA("HPVOL L", SND_SOC_NOPM,
			 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HPVOL R", SND_SOC_NOPM,
			 0, 0, NULL, 0),

	/* Output Driver */
	SND_SOC_DAPM_PGA("HPVOL L", ES8316_CPHP_OUTEN,
			     5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HPVOL R", ES8316_CPHP_OUTEN,
			     1, 0, NULL, 0),
	/* Output Lines */
	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),

};

static const struct snd_soc_dapm_route es8316_dapm_routes[] = {
	/*
	 * record route map
	 */
	{"MIC1", NULL, "micbias"},
	{"MIC2", NULL, "micbias"},
	{"DMIC", NULL, "micbias"},

	{"Differential Mux", "lin1-rin1", "MIC1"},
	{"Differential Mux", "lin2-rin2", "MIC2"},
	{"Line input PGA", NULL, "Differential Mux"},

	{"Mono ADC", NULL, "ADC Clock"},
	{"Mono ADC", NULL, "ADC Vref"},
	{"Mono ADC", NULL, "ADC bias"},
	{"Mono ADC", NULL, "Line input PGA"},

	/* It's not clear why, but to avoid recording only silence,
	 * the DAC clock must be running for the ADC to work.
	 */
	{"Mono ADC", NULL, "DAC Clock"},

	{"Digital Mic Mux", "dmic disable", "Mono ADC"},
	{"Digital Mic Mux", "dmic data at high level", "DMIC"},
	{"Digital Mic Mux", "dmic data at low level", "DMIC"},

	{"I2S OUT", NULL, "Digital Mic Mux"},
	/*
	 * playback route map
	 */
	{"DAC Source Mux", "LDATA TO LDAC, RDATA TO RDAC", "I2S IN"},
	{"DAC Source Mux", "LDATA TO LDAC, LDATA TO RDAC", "I2S IN"},
	{"DAC Source Mux", "RDATA TO LDAC, RDATA TO RDAC", "I2S IN"},
	{"DAC Source Mux", "RDATA TO LDAC, LDATA TO RDAC", "I2S IN"},

	{"Left DAC", NULL, "DAC Source Mux"},
	{"Right DAC", NULL, "DAC Source Mux"},

	{"Left Headphone Mux", "lin1-rin1", "MIC1"},
	{"Left Headphone Mux", "lin2-rin2", "MIC2"},
	{"Left Headphone Mux", "lin-rin with Boost", "Differential Mux"},
	{"Left Headphone Mux", "lin-rin with Boost and PGA", "Line input PGA"},

	{"Right Headphone Mux", "lin1-rin1", "MIC1"},
	{"Right Headphone Mux", "lin2-rin2", "MIC2"},
	{"Right Headphone Mux", "lin-rin with Boost", "Differential Mux"},
	{"Right Headphone Mux", "lin-rin with Boost and PGA", "Line input PGA"},

	{"Left Headphone Mixer", "LLIN Switch", "Left Headphone Mux"},
	{"Left Headphone Mixer", "Left DAC Switch", "Left DAC"},

	{"Right Headphone Mixer", "RLIN Switch", "Right Headphone Mux"},
	{"Right Headphone Mixer", "Right DAC Switch", "Right DAC"},

	{"HPCP L", NULL, "Left Headphone Mixer"},
	{"HPCP R", NULL, "Right Headphone Mixer"},

	{"HPVOL L", NULL, "HPCP L"},
	{"HPVOL R", NULL, "HPCP R"},

	{"HPOL", NULL, "HPVOL L"},
	{"HPOR", NULL, "HPVOL R"},
};

static int es8316_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);
	int i, ret;
	int count = 0;

	es8316->sysclk = freq;

	if (freq == 0) {
		es8316->sysclk_constraints.list = NULL;
		es8316->sysclk_constraints.count = 0;

		return 0;
	}

	ret = clk_set_rate(es8316->mclk, freq);
	if (ret)
		return ret;

	/* Limit supported sample rates to ones that can be autodetected
	 * by the codec running in slave mode.
	 */
	for (i = 0; i < NR_SUPPORTED_MCLK_LRCK_RATIOS; i++) {
		const unsigned int ratio = supported_mclk_lrck_ratios[i];

		if (freq % ratio == 0)
			es8316->allowed_rates[count++] = freq / ratio;
	}

	es8316->sysclk_constraints.list = es8316->allowed_rates;
	es8316->sysclk_constraints.count = count;

	return 0;
}
static int es8316_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 iface = 0;
	u8 adciface = 0;
	u8 daciface = 0;

	iface    = snd_soc_component_read(component, ES8316_IFACE);
	adciface = snd_soc_component_read(component, ES8316_ADC_IFACE);
	daciface = snd_soc_component_read(component, ES8316_DAC_IFACE);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x80;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface &= 0x7F;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		adciface &= 0xFC;
		daciface &= 0xFC;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		return -EINVAL;
	case SND_SOC_DAIFMT_LEFT_J:
		adciface &= 0xFC;
		daciface &= 0xFC;
		adciface |= 0x01;
		daciface |= 0x01;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		adciface &= 0xDC;
		daciface &= 0xDC;
		adciface |= 0x03;
		daciface |= 0x03;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		adciface &= 0xDC;
		daciface &= 0xDC;
		adciface |= 0x23;
		daciface |= 0x23;
		break;
	default:
		return -EINVAL;
	}

	/* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface    &= 0xDF;
		adciface &= 0xDF;
		daciface &= 0xDF;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface    |= 0x20;
		adciface |= 0x20;
		daciface |= 0x20;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface    |= 0x20;
		adciface &= 0xDF;
		daciface &= 0xDF;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface    &= 0xDF;
		adciface |= 0x20;
		daciface |= 0x20;
		break;
	default:
		return -EINVAL;
	}
	snd_soc_component_write(component, ES8316_IFACE, iface);
	snd_soc_component_write(component, ES8316_ADC_IFACE, adciface);
	snd_soc_component_write(component, ES8316_DAC_IFACE, daciface);
	return 0;
}

static int es8316_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);
	bool playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	if (es8316->sysclk_constraints.list)
		snd_pcm_hw_constraint_list(substream->runtime, 0,
					   SNDRV_PCM_HW_PARAM_RATE,
					   &es8316->sysclk_constraints);

	snd_soc_component_write(component, ES8316_RESET, 0xC0);
	snd_soc_component_write(component, ES8316_SYS_PDN, 0x00);
	/* es8316: both playback and capture need dac mclk */
	snd_soc_component_update_bits(component, ES8316_CLKMGR_CLKSW,
			    ES8316_CLKMGR_MCLK_DIV_MASK |
			    ES8316_CLKMGR_DAC_MCLK_MASK,
			    ES8316_CLKMGR_MCLK_DIV_NML |
			    ES8316_CLKMGR_DAC_MCLK_EN);
	es8316->pwr_count++;

	if (playback) {
		snd_soc_component_write(component, ES8316_SYS_LP1, 0x3F);
		snd_soc_component_write(component, ES8316_SYS_LP2, 0x1F);
		snd_soc_component_write(component, ES8316_HPMIX_SWITCH, 0x88);
		snd_soc_component_write(component, ES8316_HPMIX_PDN, 0x00);
		snd_soc_component_write(component, ES8316_HPMIX_VOL, 0xBB);
		snd_soc_component_write(component, ES8316_CPHP_PDN2, 0x10);
		snd_soc_component_write(component, ES8316_CPHP_LDOCTL, 0x30);
		snd_soc_component_write(component, ES8316_CPHP_PDN1, 0x02);
		snd_soc_component_write(component, ES8316_DAC_PDN, 0x00);
		snd_soc_component_write(component, ES8316_CPHP_OUTEN, 0x66);
		snd_soc_component_update_bits(component, ES8316_CLKMGR_CLKSW,
				    ES8316_CLKMGR_DAC_MCLK_MASK |
				    ES8316_CLKMGR_DAC_ANALOG_MASK,
				    ES8316_CLKMGR_DAC_MCLK_EN |
				    ES8316_CLKMGR_DAC_ANALOG_EN);
		msleep(50);
	} else {
		snd_soc_component_update_bits(component,
				    ES8316_GPIO_SEL,0x02,0x2);

		snd_soc_component_update_bits(component,
				    ES8316_ADC_PDN_LINSEL, 0xC0, 0x20);
		snd_soc_component_update_bits(component, ES8316_CLKMGR_CLKSW,
				    ES8316_CLKMGR_ADC_MCLK_MASK |
				    ES8316_CLKMGR_ADC_ANALOG_MASK,
				    ES8316_CLKMGR_ADC_MCLK_EN |
				    ES8316_CLKMGR_ADC_ANALOG_EN);
	}

	return 0;
}

static void es8316_pcm_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	bool playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	if (playback) {
		snd_soc_component_write(component, ES8316_CPHP_OUTEN, 0x00);
		snd_soc_component_write(component, ES8316_DAC_PDN, 0x11);
		snd_soc_component_write(component, ES8316_CPHP_LDOCTL, 0x03);
		snd_soc_component_write(component, ES8316_CPHP_PDN2, 0x22);
		snd_soc_component_write(component, ES8316_CPHP_PDN1, 0x06);
		snd_soc_component_write(component, ES8316_HPMIX_SWITCH, 0x00);
		snd_soc_component_write(component, ES8316_HPMIX_PDN, 0x33);
		snd_soc_component_write(component, ES8316_HPMIX_VOL, 0x00);
		snd_soc_component_write(component, ES8316_SYS_PDN, 0x00);
		snd_soc_component_write(component, ES8316_SYS_LP1, 0xFF);
		snd_soc_component_write(component, ES8316_SYS_LP2, 0xFF);
		snd_soc_component_update_bits(component, ES8316_CLKMGR_CLKSW,
				    ES8316_CLKMGR_DAC_ANALOG_MASK,
				    ES8316_CLKMGR_DAC_ANALOG_DIS);
	} else {
		snd_soc_component_update_bits(component,
				    ES8316_GPIO_SEL,0x02,0x0);

		snd_soc_component_write(component, ES8316_ADC_PDN_LINSEL, 0xc0);
		snd_soc_component_update_bits(component, ES8316_CLKMGR_CLKSW,
				    ES8316_CLKMGR_ADC_MCLK_MASK |
				    ES8316_CLKMGR_ADC_ANALOG_MASK,
				    ES8316_CLKMGR_ADC_MCLK_DIS |
				    ES8316_CLKMGR_ADC_ANALOG_DIS);
	}

	/*if (--es8316->pwr_count == 0) {
		if (!gpiod_get_value(es8316->hp_det_gpio))
			snd_soc_component_write(component, ES8316_SYS_PDN, 0x3F);
		snd_soc_component_write(component, ES8316_CLKMGR_CLKSW, 0xF3);
	}*/
}

static int es8316_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);	
	int val = 0;
	int i;
	
	/* Validate supported sample rates that are autodetected from MCLK */
	for (i = 0; i < NR_SUPPORTED_MCLK_LRCK_RATIOS; i++) {
		const unsigned int ratio = supported_mclk_lrck_ratios[i];

		if (es8316->sysclk % ratio != 0)
			continue;
		if (es8316->sysclk / ratio == params_rate(params))
			break;
	}
	if (i == NR_SUPPORTED_MCLK_LRCK_RATIOS)
		return -EINVAL;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		val = ES8316_DACWL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val = ES8316_DACWL_20;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		val = ES8316_DACWL_18;
		break;		
	case SNDRV_PCM_FORMAT_S24_LE:
		val = ES8316_DACWL_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val = ES8316_DACWL_32;
		break;
	default:
		val = ES8316_DACWL_16;
		break;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	snd_soc_component_update_bits(component, ES8316_SERDATA_DAC,
				    ES8316_DACWL_MASK, val);
	else
	snd_soc_component_update_bits(component, ES8316_SERDATA_ADC,
				    ES8316_ADCWL_MASK, val);

	return 0;
}

static int es8316_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);

	es8316->muted = mute;
	
	if (mute) {
		usleep_range(18000, 20000);
		snd_soc_component_write(component, ES8316_DAC_SET1, 0x20);
	} else {
		snd_soc_component_write(component, ES8316_DAC_SET1, 0x00);
		usleep_range(18000, 20000);
	}
	return 0;
}

static int es8316_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		if (IS_ERR(es8316->mclk))
			break;

		if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_ON) {
			clk_disable_unprepare(es8316->mclk);
		} else {
			ret = clk_prepare_enable(es8316->mclk);
			if (ret)
				return ret;
		}
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		snd_soc_component_write(component, ES8316_CPHP_OUTEN, 0x00);
		snd_soc_component_write(component, ES8316_DAC_PDN, 0x11);
		snd_soc_component_write(component, ES8316_CPHP_LDOCTL, 0x03);
		snd_soc_component_write(component, ES8316_CPHP_PDN2, 0x22);
		snd_soc_component_write(component, ES8316_CPHP_PDN1, 0x06);
		snd_soc_component_write(component, ES8316_HPMIX_SWITCH, 0x00);
		snd_soc_component_write(component, ES8316_HPMIX_PDN, 0x33);
		snd_soc_component_write(component, ES8316_HPMIX_VOL, 0x00);
		snd_soc_component_write(component, ES8316_ADC_PDN_LINSEL, 0xC0);
		/*if (!gpiod_get_value(es8316->hp_det_gpio))
			snd_soc_component_write(component, ES8316_SYS_PDN, 0x3F);*/
		snd_soc_component_write(component, ES8316_SYS_LP1, 0x3F);
		snd_soc_component_write(component, ES8316_SYS_LP2, 0x1F);
		snd_soc_component_write(component, ES8316_RESET, 0x00);
		break;
	}
	return 0;
}

#define es8316_RATES SNDRV_PCM_RATE_8000_96000

#define es8316_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S18_3LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)


static const struct snd_soc_dai_ops es8316_ops = {
	.startup = es8316_pcm_startup,
	.hw_params = es8316_pcm_hw_params,
	.set_fmt = es8316_set_dai_fmt,
	.set_sysclk = es8316_set_dai_sysclk,
	.mute_stream = es8316_mute,
	.shutdown = es8316_pcm_shutdown,
};

static struct snd_soc_dai_driver es8316_dai = {
	.name = "ES8316 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8316_RATES,
		.formats = es8316_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8316_RATES,
		.formats = es8316_FORMATS,
	},
	.ops = &es8316_ops,
	.symmetric_rates = 1,
};

static int es8316_init_regs(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8316_RESET, 0x3f);
	usleep_range(5000, 5500);
	snd_soc_component_write(component, ES8316_RESET, 0x00);
	snd_soc_component_write(component, ES8316_SYS_VMIDSEL, 0xFF);
	msleep(30);
	snd_soc_component_write(component, ES8316_CLKMGR_CLKSEL, 0x08);
	snd_soc_component_write(component, ES8316_CLKMGR_ADCOSR, 0x32);
	snd_soc_component_write(component, ES8316_CLKMGR_ADCDIV1, 0x11);
	snd_soc_component_write(component, ES8316_CLKMGR_ADCDIV2, 0x00);
	snd_soc_component_write(component, ES8316_CLKMGR_DACDIV1, 0x11);
	snd_soc_component_write(component, ES8316_CLKMGR_DACDIV2, 0x00);
	snd_soc_component_write(component, ES8316_CLKMGR_CPDIV, 0x00);
	snd_soc_component_write(component, ES8316_SERDATA1, 0x04);
	snd_soc_component_write(component, ES8316_CLKMGR_CLKSW, 0x7F);
	snd_soc_component_write(component, ES8316_CAL_TYPE, 0x0F);
	snd_soc_component_write(component, ES8316_CAL_HPLIV, 0x90);
	snd_soc_component_write(component, ES8316_CAL_HPRIV, 0x90);
	snd_soc_component_write(component, ES8316_ADC_VOLUME, 0x00);
	snd_soc_component_write(component, ES8316_ADC_PDN_LINSEL, 0xc0);
	snd_soc_component_write(component, ES8316_ADC_D2SEPGA, 0x00);
	snd_soc_component_write(component, ES8316_ADC_DMIC, 0x08);
	snd_soc_component_write(component, ES8316_DAC_SET2, 0x20);
	snd_soc_component_write(component, ES8316_DAC_SET3, 0x00);
	snd_soc_component_write(component, ES8316_DAC_VOLL, 0x00);
	snd_soc_component_write(component, ES8316_DAC_VOLR, 0x00);
	snd_soc_component_write(component, ES8316_SERDATA_ADC, 0x00);
	snd_soc_component_write(component, ES8316_SERDATA_DAC, 0x00);
	snd_soc_component_write(component, ES8316_SYS_VMIDLOW, 0x11);
	snd_soc_component_write(component, ES8316_SYS_VSEL, 0xFC);
	snd_soc_component_write(component, ES8316_SYS_REF, 0x28);
	snd_soc_component_write(component, ES8316_SYS_LP1, 0x04);
	snd_soc_component_write(component, ES8316_SYS_LP2, 0x0C);
	snd_soc_component_write(component, ES8316_DAC_PDN, 0x11);
	snd_soc_component_write(component, ES8316_HPMIX_SEL, 0x00);
	snd_soc_component_write(component, ES8316_HPMIX_SWITCH, 0x88);
	snd_soc_component_write(component, ES8316_HPMIX_PDN, 0x00);
	snd_soc_component_write(component, ES8316_HPMIX_VOL, 0xBB);
	snd_soc_component_write(component, ES8316_CPHP_PDN2, 0x10);
	snd_soc_component_write(component, ES8316_CPHP_LDOCTL, 0x30);
	snd_soc_component_write(component, ES8316_CPHP_PDN1, 0x02);
	snd_soc_component_write(component, ES8316_CPHP_ICAL_VOL, 0x00);
	snd_soc_component_write(component, ES8316_GPIO_SEL, 0x00);
	snd_soc_component_write(component, ES8316_GPIO_DEBOUNCE, 0x02);
	snd_soc_component_write(component, ES8316_TESTMODE, 0xA0);
	snd_soc_component_write(component, ES8316_TEST1, 0x00);
	snd_soc_component_write(component, ES8316_TEST2, 0x00);
	snd_soc_component_write(component, ES8316_SYS_PDN, 0x00);
	snd_soc_component_write(component, ES8316_RESET, 0xC0);
	msleep(50);
	snd_soc_component_write(component, ES8316_ADC_PGAGAIN, 0x60);
	snd_soc_component_write(component, ES8316_ADC_D2SEPGA, 0x01);
	/* adc ds mode, HPF enable */
	snd_soc_component_write(component, ES8316_ADC_DMIC, 0x08);
	snd_soc_component_write(component, ES8316_ADC_ALC1, 0xcd);
	snd_soc_component_write(component, ES8316_ADC_ALC2, 0x08);
	snd_soc_component_write(component, ES8316_ADC_ALC3, 0xa0);
	snd_soc_component_write(component, ES8316_ADC_ALC4, 0x05);
	snd_soc_component_write(component, ES8316_ADC_ALC5, 0x06);
	snd_soc_component_write(component, ES8316_ADC_ALC_NG, 0x61);
	return 0;
}

static int es8316_suspend(struct snd_soc_component *component)
{
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);
    //    int ret;

        clk_disable_unprepare(es8316->mclk);        

	regcache_cache_only(es8316->regmap, true);
	regcache_mark_dirty(es8316->regmap);
        	
	return 0;
}

static int es8316_resume(struct snd_soc_component *component)
{
        struct regmap *regmap = dev_get_regmap(component->dev, NULL);
        struct es8316_priv *es8316;
        int ret;

        es8316 = snd_soc_component_get_drvdata(component);

        ret = clk_prepare_enable(es8316->mclk);
        if (ret) {
                dev_err(component->dev, "unable to enable clock\n");
                return ret;
}

	regcache_cache_only(es8316->regmap, false);
	snd_soc_component_cache_sync(component);
	return 0;
}

static int es8316_probe(struct snd_soc_component *component)
{
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	es8316->mclk = devm_clk_get(component->dev, "mclk");
	if (PTR_ERR(es8316->mclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	ret = clk_prepare_enable(es8316->mclk);
	if (ret)
		return ret;
	ret = snd_soc_component_read(component, ES8316_CLKMGR_ADCDIV2);
	if (!ret) {
		es8316_reset(component); /* UPDATED BY DAVID,15-3-5 */
		ret = snd_soc_component_read(component, ES8316_CLKMGR_ADCDIV2);
		if (!ret) {
			es8316_init_regs(component);
			snd_soc_component_write(component, ES8316_GPIO_SEL, 0x00);
			/* max debance time, enable interrupt, low active */
			snd_soc_component_write(component,
				      ES8316_GPIO_DEBOUNCE, 0xf3);

			/* es8316_set_bias_level(codec, SND_SOC_BIAS_OFF); */
			snd_soc_component_write(component, ES8316_CPHP_OUTEN, 0x00);
			snd_soc_component_write(component, ES8316_DAC_PDN, 0x11);
			snd_soc_component_write(component, ES8316_CPHP_LDOCTL, 0x03);
			snd_soc_component_write(component, ES8316_CPHP_PDN2, 0x22);
			snd_soc_component_write(component, ES8316_CPHP_PDN1, 0x06);
			snd_soc_component_write(component, ES8316_HPMIX_SWITCH, 0x00);
			snd_soc_component_write(component, ES8316_HPMIX_PDN, 0x33);
			snd_soc_component_write(component, ES8316_HPMIX_VOL, 0x00);
	/*	if (!gpiod_get_value(es8316->hp_det_gpio))*/
				snd_soc_component_write(component, ES8316_SYS_PDN,
					      0x3F);
			snd_soc_component_write(component, ES8316_SYS_LP1, 0xFF);
			snd_soc_component_write(component, ES8316_SYS_LP2, 0xFF);
			snd_soc_component_write(component, ES8316_CLKMGR_CLKSW, 0xF3);
			snd_soc_component_write(component, ES8316_ADC_PDN_LINSEL, 0xc0);
		}
		}

	return ret;
	}

static void es8316_remove(struct snd_soc_component *component)
{
	es8316_set_bias_level(component, SND_SOC_BIAS_OFF);

}

static const struct snd_soc_component_driver soc_component_dev_es8316 = {
	.probe			= es8316_probe,
	.remove			= es8316_remove,
	.suspend 		= es8316_suspend,
	.resume 		= es8316_resume,
	.set_bias_level		= es8316_set_bias_level,
	.controls		= es8316_snd_controls,
	.num_controls		= ARRAY_SIZE(es8316_snd_controls),
	.dapm_widgets		= es8316_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8316_dapm_widgets),
	.dapm_routes		= es8316_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es8316_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static const struct regmap_config es8316_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register	= ES8316_TEST3,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = es8316_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es8316_reg_defaults),
};

static int es8316_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct es8316_priv *es8316;
	int ret = -1;

	es8316 = devm_kzalloc(&i2c->dev, sizeof(struct es8316_priv), GFP_KERNEL);
	if (es8316 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es8316);

	es8316->regmap = devm_regmap_init_i2c(i2c, &es8316_regmap);
	if (IS_ERR(es8316->regmap)) {
		ret = PTR_ERR(es8316->regmap);
		dev_err(&i2c->dev, "Failed to init regmap: %d\n", ret);
				return ret;
	}

	ret = snd_soc_register_component(&i2c->dev, &soc_component_dev_es8316,
				      &es8316_dai, 1);

	return ret;
}


static const struct i2c_device_id es8316_i2c_id[] = {
	{"es8316", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, es8316_i2c_id);

static const struct of_device_id es8316_of_match[] = {
	{ .compatible = "everest,es8316", },
	{}
};
MODULE_DEVICE_TABLE(of, es8316_of_match);

static struct i2c_driver es8316_i2c_driver = {
	.driver = {
		.name			= "es8316",
		.of_match_table		= es8316_of_match,
	},
	.probe		= es8316_i2c_probe,
	.id_table	= es8316_i2c_id,
};
module_i2c_driver(es8316_i2c_driver);

MODULE_DESCRIPTION("Everest Semi ES8316 ALSA SoC Codec Driver");
MODULE_AUTHOR("David Yang <yangxiaohua@everest-semi.com>");
MODULE_LICENSE("GPL v2");
