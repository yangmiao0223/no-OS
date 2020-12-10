/***************************************************************************//**
 *   @file   iio_ad7124.c
 *   @brief  Implementation of iio_ad7124.c.
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "error.h"
#include "iio.h"
#include "iio_ad7124.h"
#include "util.h"

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

static ssize_t ad1724_iio_read_status(void *device, char *buf, size_t len,
				      const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_adc_ctrl(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel);
static ssize_t ad1724_iio_change_adc_ctrl(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_ioctrl1(void *device, char *buf, size_t len,
				       const struct iio_ch_info *channel);
static ssize_t ad1724_iio_change_ioctrl1(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_ioctrl2(void *device, char *buf, size_t len,
				       const struct iio_ch_info *channel);
static ssize_t ad1724_iio_change_ioctrl2(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_id(void *device, char *buf, size_t len,
				  const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_error(void *device, char *buf, size_t len,
				     const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_error_en(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel);
static ssize_t ad1724_iio_change_error_en(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_filter_3db(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_write_filter_3db(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_offset_chan(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_change_offset_chan(void *device, char *buf,
		size_t len, const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_raw_chan(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_odr_chan(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel);
static ssize_t ad1724_iio_change_odr_chan(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_read_scale_chan(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);
static ssize_t ad1724_iio_change_scale_chan(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel);

struct iio_attribute ad7124_filter_3db = {
	.name = "filter_low_pass_3db_frequency",
	.show = ad1724_iio_read_filter_3db,
	.store = ad1724_iio_write_filter_3db
};

struct iio_attribute ad7124_chan_offset = {
	.name = "offset",
	.show = ad1724_iio_read_offset_chan,
	.store = ad1724_iio_change_offset_chan
};

struct iio_attribute ad7124_chan_raw = {
	.name = "raw",
	.show = ad1724_iio_read_raw_chan,
	.store = NULL
};

struct iio_attribute ad7124_chan_sample_freq = {
	.name = "sampling_frequency",
	.show = ad1724_iio_read_odr_chan,
	.store = ad1724_iio_change_odr_chan
};

struct iio_attribute ad7124_chan_scale = {
	.name = "scale",
	.show = ad1724_iio_read_scale_chan,
	.store = ad1724_iio_change_scale_chan
};

struct iio_attribute *channel_attributes[] = {
	&ad7124_filter_3db,
	&ad7124_chan_offset,
	&ad7124_chan_raw,
	&ad7124_chan_sample_freq,
	&ad7124_chan_scale,
	NULL
};

#define AD7124_IIO_CHANN_DEF(nm, ch1, ch2, chi) \
	struct iio_channel ad7124_ch_##chi = { \
		.name = nm, \
		.ch_type = IIO_VOLTAGE, \
		.channel = ch1, \
		.channel2 = ch2, \
		.scan_type = NULL, \
		.attributes = channel_attributes, \
		.ch_out = 0, \
		.indexed = 1, \
		.diferential = true, \
	}

AD7124_IIO_CHANN_DEF("ch0", 0, 1, 0);
AD7124_IIO_CHANN_DEF("ch1", 2, 3, 1);
AD7124_IIO_CHANN_DEF("ch2", 4, 5, 2);
AD7124_IIO_CHANN_DEF("ch3", 6, 7, 3);
AD7124_IIO_CHANN_DEF("ch4", 8, 9, 4);
AD7124_IIO_CHANN_DEF("ch5", 10, 11, 5);
AD7124_IIO_CHANN_DEF("ch6", 12, 13, 6);
AD7124_IIO_CHANN_DEF("ch7", 14, 15, 7);

struct iio_channel *ad7124_channels[] = {
	&ad7124_ch_0,
	&ad7124_ch_1,
	&ad7124_ch_2,
	&ad7124_ch_3,
	&ad7124_ch_4,
	&ad7124_ch_5,
	&ad7124_ch_6,
	&ad7124_ch_7,
	NULL
};

struct iio_attribute ad7124_status = {
	.name = "ad7124_status",
	.show = ad1724_iio_read_status,
	.store = NULL
};

struct iio_attribute ad7124_adc_ctrl = {
	.name = "ad7124_adc_ctrl",
	.show = ad1724_iio_read_adc_ctrl,
	.store = ad1724_iio_change_adc_ctrl
};

struct iio_attribute ad7124_ioctrl1 = {
	.name = "ad7124_ioctrl1",
	.show = ad1724_iio_read_ioctrl1,
	.store = ad1724_iio_change_ioctrl1
};

struct iio_attribute ad7124_ioctrl2 = {
	.name = "ad7124_ioctrl2",
	.show = ad1724_iio_read_ioctrl2,
	.store = ad1724_iio_change_ioctrl2
};

struct iio_attribute ad7124_id = {
	.name = "ad7124_id",
	.show = ad1724_iio_read_id,
	.store = NULL
};

struct iio_attribute ad7124_error = {
	.name = "ad7124_error",
	.show = ad1724_iio_read_error,
	.store = NULL
};

struct iio_attribute ad7124_error_en = {
	.name = "ad7124_error_en",
	.show = ad1724_iio_read_error_en,
	.store = ad1724_iio_change_error_en
};

struct iio_attribute *ad7124_attributes[] = {
	&ad7124_status,
	&ad7124_adc_ctrl,
	&ad7124_ioctrl1,
	&ad7124_ioctrl2,
	&ad7124_id,
	&ad7124_error,
	&ad7124_error_en,
	NULL
};

/**
 * @brief Read and display channel offset.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_offset_chan(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value;

	ad7124_read_register(desc,
			     &desc->regs[(AD7124_OFFS0_REG + channel->ch_num)]);
	value = desc->regs[(AD7124_OFFS0_REG + channel->ch_num)].value;

	return snprintf(buf, len, "%"PRIX32"", value);
}

/**
 * @brief Change channel offset.
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_change_offset_chan(void *device, char *buf,
		size_t len, const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		reg_val;

	sscanf(buf, "%ld", &reg_val);

	desc->regs[(AD7124_OFFS0_REG + channel->ch_num)].value = reg_val;

	ad7124_write_register(desc,
			      desc->regs[(AD7124_OFFS0_REG + channel->ch_num)]);

	return len;
}

/**
 * @brief Read and display channel raw value.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_raw_chan(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	int32_t			value;

	ad7124_read_register(desc,
			     &desc->regs[(AD7124_CH0_MAP_REG + channel->ch_num)]);
	desc->regs[(AD7124_OFFS0_REG + channel->ch_num)].value |=
		AD7124_CH_MAP_REG_CH_ENABLE;
	ad7124_write_register(desc,
			      desc->regs[(AD7124_CH0_MAP_REG + channel->ch_num)]);

	ad7124_wait_for_conv_ready(desc, 10000);
	ad7124_read_data(desc, &value);

	desc->regs[(AD7124_OFFS0_REG + channel->ch_num)].value &=
		~AD7124_CH_MAP_REG_CH_ENABLE;
	ad7124_write_register(desc,
			      desc->regs[(AD7124_CH0_MAP_REG + channel->ch_num)]);

	return snprintf(buf, len, "%"PRIX32"", (uint32_t)value);
}

/**
 * @brief Read and display the device status register.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_status(void *device, char *buf, size_t len,
				      const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value;

	ad7124_read_register(desc, &desc->regs[AD7124_STATUS_REG]);
	value = desc->regs[AD7124_STATUS_REG].value;

	return snprintf(buf, len, "%"PRIX32"", value);
}

/**
 * @brief Read and display the device control register.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_adc_ctrl(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value;

	ad7124_read_register(desc, &desc->regs[AD7124_ADC_CTRL_REG]);
	value = desc->regs[AD7124_ADC_CTRL_REG].value;

	return snprintf(buf, len, "%"PRIX32"", value);
}

/**
 * @brief Change the device control register.
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_change_adc_ctrl(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint16_t		reg_val;

	sscanf(buf, "%hd", &reg_val);

	desc->regs[AD7124_ADC_CTRL_REG].value = reg_val;

	ad7124_write_register(desc, desc->regs[AD7124_ADC_CTRL_REG]);

	return len;
}

/**
 * @brief Read and display the device ioctrl1 register.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_ioctrl1(void *device, char *buf, size_t len,
				       const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value;

	ad7124_read_register(desc, &desc->regs[AD7124_IO_CTRL1_REG]);
	value = desc->regs[AD7124_IO_CTRL1_REG].value;

	return snprintf(buf, len, "%"PRIX32"", value);
}

/**
 * @brief Change the device ioctrl1 register.
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_change_ioctrl1(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		reg_val;

	sscanf(buf, "%ld", &reg_val);

	desc->regs[AD7124_IO_CTRL1_REG].value = reg_val;

	ad7124_write_register(desc, desc->regs[AD7124_IO_CTRL1_REG]);

	return len;
}

/**
 * @brief Read and display the device ioctrl2 register.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_ioctrl2(void *device, char *buf, size_t len,
				       const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value;

	ad7124_read_register(desc, &desc->regs[AD7124_IO_CTRL2_REG]);
	value = desc->regs[AD7124_IO_CTRL2_REG].value;

	return snprintf(buf, len, "%"PRIX32"", value);
}

/**
 * @brief Change the device ioctrl2 register.
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_change_ioctrl2(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint16_t		reg_val;

	sscanf(buf, "%hd", &reg_val);

	desc->regs[AD7124_IO_CTRL2_REG].value = reg_val;

	ad7124_write_register(desc, desc->regs[AD7124_IO_CTRL2_REG]);

	return len;
}

/**
 * @brief Read and display the device ID register.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_id(void *device, char *buf, size_t len,
				  const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value;

	ad7124_read_register(desc, &desc->regs[AD7124_ID_REG]);
	value = desc->regs[AD7124_ID_REG].value;

	return snprintf(buf, len, "%"PRIX32"", value);
}

/**
 * @brief Read and display the device error register.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_error(void *device, char *buf, size_t len,
				     const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value;

	ad7124_read_register(desc, &desc->regs[AD7124_ERR_REG]);
	value = desc->regs[AD7124_ERR_REG].value;

	return snprintf(buf, len, "%"PRIX32"", value);
}

/**
 * @brief Read and display the device error enable register.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_error_en(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value;

	ad7124_read_register(desc, &desc->regs[AD7124_ERREN_REG]);
	value = desc->regs[AD7124_ERREN_REG].value;

	return snprintf(buf, len, "%"PRIX32"", value);
}

/**
 * @brief Change the device error enable register.
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_change_error_en(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		reg_val;

	sscanf(buf, "%ld", &reg_val);

	desc->regs[AD7124_ERREN_REG].value = reg_val;

	ad7124_write_register(desc, desc->regs[AD7124_ERREN_REG]);

	return len;
}


/**
 * @brief Get the AD7124 reference clock.
 * @param [in] dev - Pointer to the application handler.
 * @param [out] f_clk - Pointer to the clock frequency container.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
static int32_t ad7124_8pmdz_fclk_logic_get(struct ad7124_dev *dev,
		float *f_clk)
{
	int32_t ret;
	const float	f_clk_fp = 614400,
			f_clk_mp = 153600,
			f_clk_lp = 76800;
	struct ad7124_st_reg *reg_map;

	reg_map = dev->regs;

	ret = ad7124_read_register(dev, &reg_map[AD7124_ADC_Control]);
	if (ret != SUCCESS)
		return FAILURE;

	switch ((reg_map[AD7124_ADC_Control].value &
		 AD7124_ADC_CTRL_REG_POWER_MODE(3)) >> 6) {
	case 0:
		*f_clk = f_clk_lp;
		break;
	case 1:
		*f_clk = f_clk_mp;
		break;
	case 2:
	case 3:
		*f_clk = f_clk_fp;
		break;
	default:
		return FAILURE;
	}

	return SUCCESS;
}

/**
 * @brief Get the filter coefficient for the sample rate.
 * @param [in] dev - Pointer to the application handler.
 * @param [in] ch_no - Channel number.
 * @param [out] flt_coff - Pointer to the filter coefficient container.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
static int32_t ad7124_8pmdz_fltcoff_logic_get(struct ad7124_dev *dev,
		int16_t ch_no, uint16_t *flt_coff)
{
	struct ad7124_st_reg *reg_map;
	uint16_t power_mode;

	reg_map = dev->regs;

	ad7124_read_register(dev, &reg_map[AD7124_ADC_Control]);

	power_mode = (reg_map[AD7124_ADC_Control].value &
		      AD7124_ADC_CTRL_REG_POWER_MODE(3)) >> 6;

	ad7124_read_register(dev, &reg_map[AD7124_Filter_0 + ch_no]);

	*flt_coff = 32;
	if (reg_map[AD7124_Filter_0 + ch_no].value & AD7124_FILT_REG_SINGLE_CYCLE) {
		if ((reg_map[AD7124_Filter_0 + ch_no].value &
		     AD7124_FILT_REG_FILTER(7)) ==
		    AD7124_FILT_REG_FILTER(0))
			*flt_coff *= 4;
		if ((reg_map[AD7124_Filter_0 + ch_no].value &
		     AD7124_FILT_REG_FILTER(7)) ==
		    AD7124_FILT_REG_FILTER(2))
			*flt_coff *= 3;
	}
	if ((reg_map[AD7124_Filter_0 + ch_no].value & AD7124_FILT_REG_FILTER(7)) ==
	    AD7124_FILT_REG_FILTER(4)) {
		if (power_mode == 0)
			*flt_coff *= 11;
		else
			*flt_coff *= 19;
	}
	if ((reg_map[AD7124_Filter_0 + ch_no].value & AD7124_FILT_REG_FILTER(7)) ==
	    AD7124_FILT_REG_FILTER(5)) {
		if (power_mode == 0)
			*flt_coff *= 10;
		else
			*flt_coff *= 18;
	}

	return SUCCESS;
}

/**
 * @brief Calculate ODR of the device.
 * @param [in] dev - Pointer to the application handler.
 * @param [in] ch_no - Channel number.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
static float _ad7124_iio_get_odr(struct ad7124_dev *dev, int16_t ch_no)
{
	float f_clk;
	struct ad7124_st_reg *reg_map;
	uint16_t fs_value, flt_coff;

	reg_map = dev->regs;

	ad7124_8pmdz_fclk_logic_get(dev, &f_clk);

	ad7124_read_register(dev, &reg_map[AD7124_Filter_0 + ch_no]);

	fs_value = reg_map[AD7124_Filter_0 + ch_no].value & AD7124_FILT_REG_FS(0x7FF);

	if ((reg_map[AD7124_Filter_0 + ch_no].value & AD7124_FILT_REG_FILTER(7)) ==
	    AD7124_FILT_REG_FILTER(7)) {
		switch ((reg_map[AD7124_Filter_0 + ch_no].value &
			 AD7124_FILT_REG_POST_FILTER(7)) >> 17) {
		case 2:
			return 27.27;
		case 3:
			return 25;
		case 5:
			return 20;
		case 6:
			return 16.7;
		default:
			return FAILURE;
		}
	}

	ad7124_8pmdz_fltcoff_logic_get(dev, ch_no, &flt_coff);

	return (f_clk / (float)(flt_coff * fs_value));
}

/**
 * @brief Set ODR of the device.
 * @param [in] dev - Pointer to the application handler.
 * @param [in] odr - New ODR of the device.
 * @param [in] ch_no - Channel number.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
static float _ad7124_iio_set_odr(struct ad7124_dev *dev, float odr,
				 int16_t ch_no)
{

	float f_clk;
	uint16_t flt_coff, fs_value;
	struct ad7124_st_reg *reg_map;

	reg_map = dev->regs;

	ad7124_8pmdz_fclk_logic_get(dev, &f_clk);

	ad7124_8pmdz_fltcoff_logic_get(dev, ch_no, &flt_coff);

	fs_value = (uint16_t)(f_clk / (flt_coff * odr));
	if (fs_value == 0)
		fs_value = 1;
	if (fs_value > 2047)
		fs_value = 2047;

	ad7124_read_register(dev, &reg_map[AD7124_Filter_0 + ch_no]);
	reg_map[AD7124_Filter_0 + ch_no].value &= ~AD7124_FILT_REG_FS(0x7FF);
	reg_map[AD7124_Filter_0 + ch_no].value |= AD7124_FILT_REG_FS(fs_value);

	ad7124_write_register(dev, reg_map[AD7124_Filter_0 + ch_no]);

	return SUCCESS;
}

/**
 * @brief Calculate and display the channel filter cutoff frequency.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_filter_3db(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{

	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		value, odr;

	odr = (uint32_t)_ad7124_iio_get_odr(desc, channel->ch_num);
	ad7124_read_register(desc,
			     &desc->regs[(AD7124_FILT0_REG + channel->ch_num)]);
	value = (desc->regs[(AD7124_FILT0_REG + channel->ch_num)].value >> 21) & 0x7;
	switch (value) {
	case 0:
	case 4:
		value = odr * 262 / 1000;
		break;
	case 2:
	case 5:
		value = odr * 230 / 1000;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, len, "%"PRId32"", value);
}

/**
 * @brief Change the channel filter cutoff frequency.
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_write_filter_3db(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		sinc4_3db_odr, sinc3_3db_odr, freq;
	uint32_t		new_filter, new_odr;

	sscanf(buf, "%ld", &freq);

	sinc4_3db_odr = (freq * 1000) / 230;
	sinc3_3db_odr = (freq * 1000) / 262;

	if (sinc4_3db_odr > sinc3_3db_odr) {
		new_filter = 2;
		new_odr = sinc3_3db_odr;
	} else {
		new_filter = 0;
		new_odr = sinc4_3db_odr;
	}

	desc->regs[(AD7124_FILT0_REG + channel->ch_num)].value &=
		~AD7124_FILT_REG_FILTER(~0);
	desc->regs[(AD7124_FILT0_REG + channel->ch_num)].value |=
		AD7124_FILT_REG_FILTER(new_filter);
	ad7124_write_register(desc,
			      desc->regs[(AD7124_FILT0_REG + channel->ch_num)]);

	_ad7124_iio_set_odr(desc, (float)new_odr, channel->ch_num);

	return len;
}

/**
 * @brief Calculate and display the channel ODR.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_odr_chan(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		odr;

	odr = (uint32_t)_ad7124_iio_get_odr(desc, channel->ch_num);

	return snprintf(buf, len, "%"PRId32"", odr);
}

/**
 * @brief Change the channel ODR.
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_change_odr_chan(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		new_odr;

	sscanf(buf, "%ld", &new_odr);

	_ad7124_iio_set_odr(desc, (float)new_odr, channel->ch_num);

	return len;
}

/**
 * @brief Calculate and display the channel LSB voltage value.
 * @param device - Device driver descriptor.
 * @param buf - Output buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_read_scale_chan(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	uint32_t		vref_mv = 2500, adc_bit_no = 24;
	uint32_t		pga_bits, bipolar;
	float			lsb_val;

	ad7124_read_register(desc, &desc->regs[AD7124_CFG0_REG + channel->ch_num]);
	pga_bits = desc->regs[AD7124_CFG0_REG + channel->ch_num].value & 0x7;
	bipolar = (desc->regs[AD7124_CFG0_REG + channel->ch_num].value >> 11) & 0x1;
	if (bipolar != 0)
		lsb_val = vref_mv / pow(2, (adc_bit_no + pga_bits - 1));
	else
		lsb_val = vref_mv / pow(2, (adc_bit_no + pga_bits));

	return snprintf(buf, len, "%f", lsb_val);
}

/**
 * @brief Find closest possible channel gain from a free input one.
 * @param [in] new_gain - Input gain.
 * @return New PGA setting based on the gain.
 */
static uint32_t ad7124_iio_find_closest_gain(uint32_t new_gain)
{
	uint32_t new_pga_bits;
	uint32_t old_diff = 0xFFFFFFFF, new_diff;
	uint32_t i;

	for (i = 0; i < 8; i++) {
		new_diff = abs(pow(2, i) - new_gain);
		if (new_diff < old_diff) {
			old_diff = new_diff;
			new_pga_bits = i;
		}
	}

	return new_pga_bits;
}

/**
 * @brief Change the channel LSB voltage value.
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer (not used in this case).
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
static ssize_t ad1724_iio_change_scale_chan(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	struct ad7124_dev	*desc = (struct ad7124_dev *)device;
	float			new_scale;
	uint32_t		vref_mv = 2500, adc_bit_no = 24;
	uint32_t		bipolar;
	float			lsb_val;
	uint32_t		new_gain;

	sscanf(buf, "%f", &new_scale);

	ad7124_read_register(desc, &desc->regs[AD7124_CFG0_REG + channel->ch_num]);
	bipolar = (desc->regs[AD7124_CFG0_REG + channel->ch_num].value >> 11) & 0x1;
	if (bipolar != 0)
		lsb_val = vref_mv / pow(2, (adc_bit_no - 1));
	else
		lsb_val = vref_mv / pow(2, adc_bit_no);

	new_gain = lsb_val / new_scale;

	new_gain = ad7124_iio_find_closest_gain(new_gain);
	desc->regs[AD7124_CFG0_REG + channel->ch_num].value &= ~AD7124_CFG_REG_PGA(~0);
	desc->regs[AD7124_CFG0_REG + channel->ch_num].value |= AD7124_CFG_REG_PGA(
				new_gain);
	ad7124_write_register(desc, desc->regs[AD7124_CFG0_REG + channel->ch_num]);

	return len;
}

/**
 * @brief Transfer data from device into RAM.
 * @param iio_inst - Physical instance of a iio_axi_adc device.
 * @param bytes_count - Number of bytes to transfer.
 * @param ch_mask - Opened channels mask.
 * @return bytes_count or negative value in case of error.
 */
static ssize_t iio_ad7124_transfer_dev_to_mem(void *iio_inst,
		size_t bytes_count,
		uint32_t ch_mask)
{
	struct iio_ad7124_desc	*iio_desc = (struct iio_ad7124_desc *)iio_inst;
	struct ad7124_dev	*desc = (struct ad7124_dev *)iio_desc->ad7124_driver_handler;
	ssize_t ret, sample_no;
	uint8_t ch_bulk = 3 * hweight8(ch_mask);
	int32_t data_temp;
	uint32_t i, j = 0, chan_idx;

	if ((bytes_count % ch_bulk) != 0)
		return -EINVAL;

	sample_no = bytes_count / ch_bulk;

	while (1) {
		for (i = 0; i < hweight8(ch_mask); i++) {
			ret = ad7124_wait_for_conv_ready(desc, 100000);
			if (ret != SUCCESS)
				return -ETIMEDOUT;
			ret = ad7124_read_data(desc, &data_temp);
			if (ret != SUCCESS)
				return -EIO;
			chan_idx = desc->regs[AD7124_Status].value &
				   AD7124_STATUS_REG_CH_ACTIVE(0xF);
			if ((chan_idx != 0) && (i == 0)) {
				i--;
				continue;
			}
			iio_desc->ddr_base_addr[j + i * 3] = *((uint8_t *)&data_temp + 1);
			iio_desc->ddr_base_addr[j + i * 3 + 1] = *((uint8_t *)&data_temp + 2);
			iio_desc->ddr_base_addr[j + i * 3 + 2] = *((uint8_t *)&data_temp + 3);
		}
		j++;
		if (j == sample_no)
			break;
	}

	return bytes_count;
}

/**
 * @brief Read chunk of data from RAM to pbuf.
 * Call "iio_axi_adc_transfer_dev_to_mem" first.
 * This function is probably called multiple times by libtinyiiod after a
 * "iio_axi_adc_transfer_dev_to_mem" call, since we can only read "bytes_count"
 * bytes at a time.
 * @param iio_inst - Physical instance of a iio_axi_adc device.
 * @param pbuf - Buffer where value is stored.
 * @param offset - Offset to the remaining data after reading n chunks.
 * @param bytes_count - Number of bytes to read.
 * @param ch_mask - Opened channels mask.
 * @return bytes_count or negative value in case of error.
 */
static ssize_t iio_ad7124_read_dev(void *iio_inst, char *pbuf, size_t offset,
				   size_t bytes_count, uint32_t ch_mask)
{
	struct iio_ad7124_desc	*iio_desc = (struct iio_ad7124_desc *)iio_inst;
	uint32_t i, j = 0, current_ch = 0;
	uint32_t *pbuf32;
	size_t samples;
	uint8_t ch_bulk = 3 * hweight8(ch_mask);

	if (!iio_inst)
		return FAILURE;

	if (!pbuf)
		return FAILURE;

	if ((bytes_count % ch_bulk) != 0)
		return -EINVAL;

	pbuf32 = (uint32_t*)pbuf;
	samples = bytes_count / ch_bulk;

	offset = (offset * iio_desc->iio_ad7124_device->num_ch) / hweight8(ch_mask);

	for (i = 0; i < samples * iio_desc->iio_ad7124_device->num_ch; i++) {
		if (ch_mask & BIT(current_ch)) {
			pbuf32[j] = *(uint16_t*)(iio_desc->ddr_base_addr + offset + i * 2);
			j++;
		}

		if (current_ch + 1 < iio_desc->iio_ad7124_device->num_ch)
			current_ch++;
		else
			current_ch = 0;
	}

	return bytes_count;
}

/**
 * @brief Initialize the AD7124 IIO driver.
 * @param [out] desc - Pointer to the driver handler.
 * @param [in] init_param - Pointer to the initialization structure.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t iio_ad7124_init(struct iio_ad7124_desc **desc,
			struct iio_ad7124_init_param *init_param)
{
	struct iio_ad7124_desc *ldesc;

	ldesc = calloc(1, sizeof(*ldesc));
	if (!ldesc)
		return FAILURE;

	ldesc->iio_ad7124_device = calloc(1,
					  sizeof(*ldesc->iio_ad7124_device));
	if (!ldesc->iio_ad7124_device) {
		free(ldesc);
		return -ENOMEM;
	}

	ldesc->iio_ad7124_device->num_ch = 8;
	ldesc->iio_ad7124_device->channels = ad7124_channels;
	ldesc->iio_ad7124_device->attributes = ad7124_attributes;
	ldesc->iio_ad7124_device->debug_attributes = NULL;
	ldesc->iio_ad7124_device->buffer_attributes = NULL;
	ldesc->iio_ad7124_device->transfer_dev_to_mem = iio_ad7124_transfer_dev_to_mem;
	ldesc->iio_ad7124_device->read_data = iio_ad7124_read_dev;
	ldesc->iio_ad7124_device->transfer_mem_to_dev = NULL;
	ldesc->iio_ad7124_device->write_data = NULL;

	ldesc->ad7124_driver_handler =
		init_param->ad7124_driver_handler;
	ldesc->ddr_base_addr = init_param->ddr_base_addr;

	*desc = ldesc;

	return SUCCESS;
}

/**
 * @brief Free memory allocated by iio_ad7124_init().
 * @param [in] desc - Pointer to the driver handler.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t iio_ad7124_remove(struct iio_ad7124_desc *desc)
{
	if (!desc)
		return FAILURE;

	free(desc->iio_ad7124_device);
	free(desc);

	return SUCCESS;
}

