/***************************************************************************//**
 *   @file   iio_ad7799.h
 *   @brief  Header file of AD7799 iio.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
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

#ifndef IIO_AD7799_H
#define IIO_AD7799_H

#include "iio_types.h"

/** AD7799 IIO register read */
ssize_t ad7799_iio_reg_read(void *device, char *buf, size_t len,
			    const struct iio_ch_info *channel);
/** AD7799 IIO Channel read */
ssize_t ad7799_iio_channel_read(void *device, char *buf, size_t len,
				const struct iio_ch_info *channel);
/** AD7799 IIO Gain write read */
ssize_t ad7799_iio_gain_read(void *device, char *buf, size_t len,
			     const struct iio_ch_info *channel);
/** AD7799 IIO Gain write */
ssize_t ad7799_iio_gain_write(void *device, char *buf, size_t len,
			      const struct iio_ch_info *channel);

/** ADC IIO channel attribute*/
static struct iio_attribute ad7799_iio_channel_attr = {
	.name = "volts",
	.show = ad7799_iio_channel_read,
	.store = NULL
};

/** ADC IIO channel attributes*/
static struct iio_attribute *ad7799_iio_channel_attributes[] = {
	&ad7799_iio_channel_attr,
	NULL,
};

/** ADC IIO Channel 1 attributes */
static struct iio_channel ad7799_iio_channel_1 = {
	.name = "channel0",
	.scan_index = 0,
	.scan_type = {
		.sign = 'u',
		.realbits = 12,
		.storagebits = 24,
		.shift = 0,
		.is_big_endian = false
	},
	.attributes = ad7799_iio_channel_attributes,
	.ch_out = false,
};

/** ADC IIO Channel 2 attributes */
static struct iio_channel ad7799_iio_channel_2 = {
	.name = "channel1",
	.scan_index = 1,
	.scan_type = {
		.sign = 'u',
		.realbits = 12,
		.storagebits = 24,
		.shift = 0,
		.is_big_endian = false
	},
	.attributes = ad7799_iio_channel_attributes,
	.ch_out = false,
};

/** ADC IIO Channel 3 attributes */
static struct iio_channel ad7799_iio_channel_3 = {
	.name = "channel2",
	.scan_index = 2,
	.scan_type = {
		.sign = 'u',
		.realbits = 12,
		.storagebits = 24,
		.shift = 0,
		.is_big_endian = false
	},
	.attributes = ad7799_iio_channel_attributes,
	.ch_out = false,
};

/** IIO ADC Channels */
static struct iio_channel *ad7799_iio_channels[] = {
	&ad7799_iio_channel_1,
	&ad7799_iio_channel_2,
	&ad7799_iio_channel_3,
	NULL,
};

/** Direct register access IIO attribute */
static struct iio_attribute ad7799_iio_reg_attr = {
	.name = "direct_reg_access",
	.show = ad7799_iio_reg_read,
	.store = NULL,
};

/** Debug IIO attributes */
static struct iio_attribute *ad7799_iio_debug_attributes[] = {
	&ad7799_iio_reg_attr,
	NULL,
};

/** Gain IIO attributes */
static struct iio_attribute ad7799_iio_gain_attr = {
	.name = "gain",
	.show = ad7799_iio_gain_read,
	.store = ad7799_iio_gain_write,
};

/** IIO attributes */
static struct iio_attribute *ad7799_iio_attributes[] = {
	&ad7799_iio_gain_attr,
	NULL,
};

/** IIO Descriptor */
struct iio_device ad7799_iio_descriptor = {
	.num_ch = 3,
	.channels = ad7799_iio_channels,
	.attributes = ad7799_iio_attributes,
	.debug_attributes = ad7799_iio_debug_attributes,
	.buffer_attributes = NULL,
	.transfer_dev_to_mem = NULL,
	.transfer_mem_to_dev = NULL,
	.read_data = NULL,
	.write_data = NULL
};

#endif //IIO_AD7799_H
