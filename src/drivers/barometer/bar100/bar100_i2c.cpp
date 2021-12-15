/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ms5837_i2c.cpp
 *
 * I2C interface for MS5837
 */

#include <drivers/device/i2c.h>

#include "bar100.h"

class BAR100_I2C : public device::I2C
{
public:
	BAR100_I2C(uint8_t bus, int bus_frequency);
	~BAR100_I2C() override = default;

	int	read(unsigned offset, void *data, unsigned count) override;
	virtual int ioctl(unsigned operation, unsigned &arg);
protected:
	int	probe() override;

private:
	int		_probe_address(uint8_t address);

	/**
	 * Send a reset command to the MS5837.
	 *
	 * This is required after any bus reset.
	 */
	int		_reset();

	/**
	 * Send a measure command to the MS5837.
	 *
	 * @param addr		Which address to use for the measure operation.
	 */
	int		_measure(unsigned addr);

};

device::Device *
BAR100_i2c_interface(uint32_t devid, uint8_t busnum, int bus_frequency)
{
	return new BAR100_I2C(busnum, bus_frequency);
}

BAR100_I2C::BAR100_I2C(uint8_t bus, int bus_frequency) :
	I2C(DRV_BARO_DEVTYPE_BAR100, MODULE_NAME, bus, 0, bus_frequency)
{
}

int
BAR100_I2C::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[3];

	/* read the most recent measurement */
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, &buf[0], 3);

	if (ret == PX4_OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[2];
		cvt->b[1] = buf[1];
		cvt->b[2] = buf[0];
		cvt->b[3] = 0;
	}

	return ret;
}

int
BAR100_I2C::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	printf("%d\n",operation);
	ret = _measure(arg);

	return ret;
}

int
BAR100_I2C::probe()
{
	if (PX4_OK == _probe_address(BAR100_ADDRESS_1)) {

		return PX4_OK;
	}

	_retries = 1;

	return -EIO;
}

int
BAR100_I2C::_probe_address(uint8_t address)
{
	/* select the address we are going to try */
	set_device_address(address);

	/* send reset command */
	if (PX4_OK != _reset()) {
		return -EIO;
	}

	return PX4_OK;
}

int
BAR100_I2C::_measure(unsigned addr)
{
	uint8_t cmd = addr;
	return transfer(&cmd, 1, nullptr, 0);
}

