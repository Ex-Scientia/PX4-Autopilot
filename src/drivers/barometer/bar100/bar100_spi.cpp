/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file ms5837_spi.cpp
 *
 * SPI interface for MS5837
 */

#include <drivers/device/spi.h>
#include "bar100.h"

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)


class BAR100_SPI : public device::SPI
{
public:
	BAR100_SPI(uint8_t bus, uint32_t device,  int bus_frequency, spi_mode_e spi_mode);
	virtual ~BAR100_SPI() = default;

	virtual int	init();
	virtual int	read(unsigned offset, void *data, unsigned count);
	virtual int	ioctl(unsigned operation, unsigned &arg);

private:

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

	/**
	 * Read a 16-bit register value.
	 *
	 * @param reg		The register to read.
	 */
	uint16_t	_reg16(unsigned reg);

	/**
	 * Wrapper around transfer() that prevents interrupt-context transfers
	 * from pre-empting us. The sensor may (does) share a bus with sensors
	 * that are polled from interrupt context (or we may be pre-empted)
	 * so we need to guarantee that transfers complete without interruption.
	 */
	int		_transfer(uint8_t *send, uint8_t *recv, unsigned len);
};

device::Device *
BAR100_spi_interface(uint32_t devid, uint8_t busnum, int bus_frequency, spi_mode_e spi_mode)
{
	return new BAR100_SPI(busnum, devid, bus_frequency, spi_mode);
}

BAR100_SPI::BAR100_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_BARO_DEVTYPE_BAR100, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
}

int
BAR100_SPI::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		PX4_DEBUG("SPI init failed");
		goto out;
	}

	/* send reset command */
	ret = _reset();

	if (ret != OK) {
		PX4_DEBUG("reset failed");
		goto out;
	}

out:
	return ret;
}

int
BAR100_SPI::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[4] = { 0 | DIR_WRITE, 0, 0, 0 };

	/* read the most recent measurement */
	int ret = _transfer(&buf[0], &buf[0], sizeof(buf));

	if (ret == OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[3];
		cvt->b[1] = buf[2];
		cvt->b[2] = buf[1];
		cvt->b[3] = 0;

		ret = count;
	}

	return ret;
}

int
BAR100_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	printf("%d\n", operation);
	ret = _measure(arg);

	return ret;
}

int
BAR100_SPI::_measure(unsigned addr)
{
	uint8_t cmd = addr | DIR_WRITE;

	return _transfer(&cmd, nullptr, 1);
}

uint16_t
BAR100_SPI::_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	_transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

int
BAR100_SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	return transfer(send, recv, len);
}
