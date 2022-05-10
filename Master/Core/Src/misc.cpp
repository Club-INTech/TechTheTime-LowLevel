#include "misc.h"

#include <order/misc.h>
#include <rpc/slave_2a.hpp>

static I2C_HandleTypeDef *slave_i2c;

#if INTECH_TARGET == 2
static upd::byte_t buf[32], *p;
void write_byte(upd::byte_t byte) {
	*p++ = byte;
}
#endif // INTECH_TARGET == 2

extern "C" void Misc_Init(I2C_HandleTypeDef *_slave_i2c) {
	slave_i2c = _slave_i2c;
}

//
// ORDERS
//

extern "C" void Misc_Set_Pump(uint8_t device, uint8_t state) {
	if (__HAL_I2C_GET_FLAG(slave_i2c, I2C_FLAG_BUSY))
		return;

#if INTECH_TARGET == 1
	uint8_t ocode = 1 << 7 | (state ? 1 : 0) << 6 | device;
	HAL_I2C_Master_Transmit_IT(slave_i2c, 1 << 1, &ocode, sizeof ocode);
#elif INTECH_TARGET == 2
	p = buf;
	auto key = rpc::slave_2a::keyring.get<Misc_Set_Pump>();
	key(device, state) >> write_byte;
	HAL_I2C_Master_Transmit_IT(slave_i2c, 1 << 1, buf, p - buf);
#endif // INTECH_TARGET == 1, INTECH_TARGET == 2
}

extern "C" void Misc_Set_Valve(uint8_t device, uint8_t state) {
	if (__HAL_I2C_GET_FLAG(slave_i2c, I2C_FLAG_BUSY))
		return;

#if INTECH_TARGET == 1
	uint8_t ocode = (state ? 1 : 0) << 6 | device;
	HAL_I2C_Master_Transmit_IT(slave_i2c, 1 << 1, &ocode, sizeof ocode);
#elif INTECH_TARGET == 2
	p = buf;
	auto key = rpc::slave_2a::keyring.get<Misc_Set_Valve>();
	key(device, state) >> write_byte;
	HAL_I2C_Master_Transmit_IT(slave_i2c, 1 << 1, buf, p - buf);
#endif // INTECH_TARGET == 1, INTECH_TARGET == 2
}

extern "C" void Misc_Set_Servo(uint8_t device, uint16_t value) {
	if (__HAL_I2C_GET_FLAG(slave_i2c, I2C_FLAG_BUSY))
		return;

#if INTECH_TARGET == 2
	p = buf;
	auto key = rpc::slave_2a::keyring.get<Misc_Set_Servo>();
	key(device, value) >> write_byte;
	HAL_I2C_Master_Transmit_IT(slave_i2c, 1 << 1, buf, p - buf);
#endif // INTECH_TARGET == 2
}
