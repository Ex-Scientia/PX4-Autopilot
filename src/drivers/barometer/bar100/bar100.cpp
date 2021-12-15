/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file bar100.cpp
 * Driver for the bar100 barometric pressure sensor connected via I2C or SPI.
 */

#include "BAR100.hpp"
#include "bar100.h"

#include <cdev/CDev.hpp>

#include <cstring>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <string.h>




BAR100::BAR100(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_px4_barometer(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	
}

BAR100::~BAR100()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
BAR100::init()
{
	int ret;

	while (true) {
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		const sensor_baro_s &brp = _px4_barometer.get();

		if (brp.pressure < 520.0f) {
			/* This is likely not this device, abort */
			ret = -EINVAL;
			break;
		}

		_interface->set_device_type(DRV_BARO_DEVTYPE_BAR100);
		_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_BAR100);

		ret = OK;

		break;
	}

	if (ret == 0) {
		start();
	}

	return ret;
}

void
BAR100::start()
{
	// reset the report ring and state machine
	_collect_phase = false;

	// schedule a cycle to start things	
	ScheduleNow();
}

void
BAR100::RunImpl()
{
	if (_collect_phase) {
		collect();
	} else {
		measure();
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(BAR100_CONVERSION_INTERVAL);
}

int
BAR100::measure()
{
	perf_begin(_measure_perf);

	_collect_phase = true;

	unsigned addr = CMD_REQUEST_MEASUREMENT;
	/*
	 * Send the command to begin measuring.	
	 * */
	int ret = _interface->ioctl(1,addr);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	_px4_barometer.set_error_count(perf_event_count(_comms_errors));

	perf_end(_measure_perf);

	return ret;
}

int
BAR100::collect()
{
	uint32_t raw;
  
	perf_begin(_sample_perf);

	_collect_phase = false;

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {
    
    int32_t SENSi = 0;
	  int32_t OFFi = 0;
	  int32_t Ti = 0;
		
    /* temperature offset (in ADC units) */
    int32_t dT = (int32_t)raw;

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
    int32_t TEMP = 2000l+int64_t(dT)*8388608LL;

		/* base sensor scale/offset values */
			/* Perform MS5837 Caculation */
			/* Similar to MS5611 but with second order temperature compensation for high temperatures. Have fun in warm water! */
      _SENS = 32768l+(dT)/256l;
		  _OFF = 65536l+(dT)/128l;

			/* MS5837 temperature compensation */
			  if (TEMP < 2000) {
          Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
			    OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			    SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
				  
          if (TEMP < -1500) {
            OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				    SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
				  }
			  } else if (TEMP >= 2000) {
		  	  Ti = 2*(dT*dT)/(137438953472LL);
			    OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			    SENSi = 0;
        }
    
    _OFF -= OFFi;           //Calculate pressure and temp second order
	  _SENS -= SENSi;

	  TEMP = (TEMP-Ti);

		float temperature = TEMP / 100.0f;
		_px4_barometer.set_temperature(temperature);

	} else {
		float pressure;
		int32_t P;

		P = (((raw*_SENS)/2097152l-_OFF)/8192l);
      	pressure = P/10.0f;		/* convert to millibar */

		_px4_barometer.update(timestamp_sample, pressure);
      
  }

	perf_end(_sample_perf);

	return OK;
}

void BAR100::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	char device_name[10] {};
	strncpy(device_name, "bar100", sizeof(device_name));
	printf("device:         %s\n", device_name);
}

namespace bar100
{

/**
 * MS5837 crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0;

	/* save the read crc */
	crc_read = n_prom[0] >> 12;

	/* remove CRC byte */
	n_prom[0] = ((n_prom[0]) & 0x0FFF);
  n_prom[7] = 0;

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt%2 == 1) {
			n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = ((n_rem >> 12) & 0x000F);

	/* return true if CRCs match */
	return (crc_read) == (n_rem ^ 0x00);
}

} // namespace bar100
