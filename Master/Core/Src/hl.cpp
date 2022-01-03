#include "hl.h"

#include <k2o/dispatcher.hpp>
#include <order/motion.h>
#include <order/controller.h>
#include <rpc/master.hpp>
#include <rpc/controller.hpp>


static uint8_t header[]={0xff,0xff,0xff};
static uint8_t byte_stuff = 0x00;
static const uint32_t timeout = 1000;

static UART_HandleTypeDef *huart_ptr = NULL;
static uint8_t write_stuff_counter=0;
static uint8_t read_stuff_counter=0;
static k2o::dispatcher dispatcher{rpc::master::keyring};

uint8_t has_received_order = 0;

static void write_byte(uint8_t byte) {
	if (byte == 0xff){
		write_stuff_counter++;
	}
	else {
		write_stuff_counter=0;
	}
	HAL_UART_Transmit(huart_ptr, &byte, 1, timeout);
	if (write_stuff_counter ==2){
		write_stuff_counter=0;
		HAL_UART_Transmit(huart_ptr, &byte_stuff, 1, timeout);
	}
}

static uint8_t read_byte() {
	uint8_t byte;
	while (HAL_UART_Receive(huart_ptr, &byte, 1, timeout) != HAL_OK)

	if (byte == 0xff){
		read_stuff_counter++;
	}
	else {
		read_stuff_counter=0;
	}

	if (read_stuff_counter ==2){
		uint8_t discarded_byte;
		read_stuff_counter=0;
		while (HAL_UART_Receive(huart_ptr, &discarded_byte, 1, timeout) != HAL_OK);
	}

	return byte;
}

extern "C" void HL_Init(UART_HandleTypeDef *handler_ptr) {
	huart_ptr = handler_ptr;
}

static uint32_t the_counter = 0;
extern "C" void HL_Send_Measure(void) {
	auto report_measure_key = rpc::controller::keyring.get<controller_report_measure>();
	HAL_UART_Transmit(huart_ptr, header, sizeof(header), timeout);
	report_measure_key(the_counter, 0, 0) >> write_byte;
	the_counter += 1000000ull;
}

extern "C" void HL_Interrupt(void) {
	uint8_t header_counter = 0;
	while (header_counter < 3) {
		uint8_t byte = 0x00;
		while (HAL_UART_Receive(huart_ptr, &byte, 1, timeout) != HAL_OK);
		if (byte == 0xff) {
			header_counter++;
		}
		else {
			header_counter = 0;
		}
	}
	dispatcher(read_byte, write_byte);
}

extern "C" void motion_set_translation_setpoint(uint32_t) {
	has_received_order = 1;
}
