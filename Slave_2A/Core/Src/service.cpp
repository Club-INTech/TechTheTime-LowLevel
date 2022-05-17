#include "service.h"

#include <rpc/slave_2a.hpp>
#include <k2o/dispatcher.hpp>

#include "main.h"

static I2C_HandleTypeDef *master_i2c;

static TIM_HandleTypeDef *servo_tim1;
static TIM_HandleTypeDef *servo_tim2;

static uint8_t buf[32];
static k2o::dispatcher dispatcher { rpc::slave_2a::keyring };

extern "C" void Service_Init(I2C_HandleTypeDef *_master_i2c,
		TIM_HandleTypeDef *_servo_tim1, TIM_HandleTypeDef *_servo_tim2) {
	master_i2c = _master_i2c;
	servo_tim1 = _servo_tim1;
	servo_tim2 = _servo_tim2;

	HAL_TIM_PWM_Start(servo_tim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(servo_tim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(servo_tim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(servo_tim1, TIM_CHANNEL_4);
}

extern "C" void Service_Process(void) {
	auto read_buf = [p = buf]() mutable {
		return *p++;
	};
	auto noop = [](upd::byte_t) {
	};
	if (HAL_I2C_Slave_Receive(master_i2c, buf, 4, 1e3) != HAL_OK)
		return;
	dispatcher(read_buf, noop);
}

//
// ORDERS
//

void Misc_Set_Pump(uint8_t device, uint8_t state) {
	constexpr static GPIO_TypeDef *device_ports[] = { Pump1_GPIO_Port,
	Pump2_GPIO_Port };
	constexpr static uint16_t device_pins[] = { Pump1_Pin, Pump2_Pin };

	HAL_GPIO_WritePin(device_ports[device], device_pins[device],
			state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Misc_Set_Valve(uint8_t device, uint8_t state) {
	constexpr static GPIO_TypeDef *device_ports[] = { Valve1_GPIO_Port,
	Valve2_GPIO_Port };
	constexpr static uint16_t device_pins[] = { Valve1_Pin, Valve2_Pin };

	HAL_GPIO_WritePin(device_ports[device], device_pins[device],
			state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Misc_Set_Servo(uint8_t device, uint16_t value) {
	constexpr
	static unsigned device_channels[] = { TIM_CHANNEL_1, TIM_CHANNEL_2,
	TIM_CHANNEL_3, TIM_CHANNEL_4 };
	constexpr auto servo_duty_cycle_min = 10000, servo_duty_cycle_max = 15000;

	__HAL_TIM_SET_COMPARE(device < 2 ? servo_tim1 : servo_tim2,
			device_channels[device],
			servo_duty_cycle_min + value * (servo_duty_cycle_max - servo_duty_cycle_min) / UINT16_MAX);
}
