/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ms5837.h
 *
 * Shared defines for the ms5837 driver.
 */

#pragma once

#include <string.h>

#include <drivers/device/i2c.h>
#include <drivers/device/device.h>
#include <drivers/device/spi.h>
#include <lib/cdev/CDev.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>
#include <uORB/uORB.h>

#include "board_config.h"

#define CMD_METADATA_PMODE		0x12
#define CMD_PRANGE_MIN_MSB		0x13
#define CMD_PRANGE_MIN_LSB		0x14
#define CMD_PRANGE_MAX_MSB		0x15
#define CMD_PRANGE_MAX_LSB		0x16

#define CMD_REQUEST_MEASUREMENT 		0xAC

#define BAR100_ADDRESS_1		0x40	/* address select pins pulled high (PX4FMU series v1.6+) */


namespace bar100
{
} /* namespace */

/* interface factories */
extern device::Device *BAR100_spi_interface(uint32_t devid, uint8_t busnum, int bus_frequency,
		spi_mode_e spi_mode);
extern device::Device *BAR100_i2c_interface(uint32_t devid, uint8_t busnum,
		int bus_frequency);

typedef device::Device *(*BAR100_constructor)(uint32_t devid, uint8_t busnum);
