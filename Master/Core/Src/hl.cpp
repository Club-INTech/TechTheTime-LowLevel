#include "hl.hpp"

#include <utility>

#include "stm32g4xx_hal_uart.h"

#include <k2o/dispatcher.hpp>
#include <order/motion.h>
#include <order/controller.h>
#include <rpc/def.hpp>
#include <rpc/master.hpp>

#define RX_BUF_SIZE 128

static void HL_Interrupt(UART_HandleTypeDef *);
static void HL_Interrupt_Get_Order(UART_HandleTypeDef *);
static void HL_Interrupt_Call_Order(UART_HandleTypeDef *);
static void HL_Reset_Interrupt(size_t = 0);

static UART_HandleTypeDef *huart_ptr = NULL;

static uint8_t response_header[] = {rpc::header[0], rpc::header[1], rpc::header[2], rpc::RESPONSE};
static uint8_t byte_stuff = 0x00;
static const uint32_t timeout = 1000;

static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t read_stuff_counter = 0;
static uint8_t write_stuff_counter=0;
uint8_t indice =0;

uint8_t byte = 0x00; //receive byte from HL_INTERRUPT
uint8_t count = 0; //counts number of time you receive 0xff
uint8_t get_header =0;



static k2o::dispatcher dispatcher{rpc::master::keyring};
static k2o::order *hanging_order = NULL;

extern "C" void HL_Init(UART_HandleTypeDef *handler_ptr) {
	huart_ptr = handler_ptr;
	HL_Reset_Interrupt();
}

extern "C" void HL_Write_Byte(uint8_t byte) {
	if (byte == 0xff){
		write_stuff_counter++;
	}
	else {
		write_stuff_counter=0;
	}
	HAL_UART_Transmit(huart_ptr, &byte, 1, timeout);
	if (write_stuff_counter == 3){
		write_stuff_counter=0;
		HAL_UART_Transmit(huart_ptr, &byte_stuff, 1, timeout);
	}
}

//
// Interrupt routines
//

void HL_Interrupt(UART_HandleTypeDef *) {
	if (byte == rpc::header[0]) {

		count++;
		if (count == sizeof rpc::header) {
			count = 0;
			HAL_UART_Receive_IT(huart_ptr, rx_buf, 3);
			HAL_UART_RegisterCallback(huart_ptr, HAL_UART_RX_COMPLETE_CB_ID, &HL_Interrupt_Get_Order);
			return ;
		}
	} else {
		count = 0;
	}
	HAL_UART_Receive_IT(huart_ptr, &byte, 1);
}

void HL_Interrupt_Get_Order(UART_HandleTypeDef *) {
	// Réccupère l'ordre à appeler en fonction de la valeur reçue
	auto read_byte = [ptr = rx_buf]() mutable {
		if (*ptr == rpc::header[0]) {
			read_stuff_counter++;
		} else {
			read_stuff_counter = 0;
		}
		if (read_stuff_counter == sizeof rpc::header) {
			ptr++;
			read_stuff_counter = 0;
		}
		return *ptr++;
	};

	auto handle_order = [](k2o::order &order) {
			hanging_order = &order;
			HAL_UART_RegisterCallback(huart_ptr, HAL_UART_RX_COMPLETE_CB_ID, &HL_Interrupt_Call_Order);
			HAL_UART_Receive_IT(huart_ptr, &byte,1);
	};

	dispatcher.get_order(read_byte)
			.map(handle_order)
			.map_error(HL_Reset_Interrupt);
}


void HL_Interrupt_Call_Order(UART_HandleTypeDef *) {
	// Exécute 'hanging_order' avec les octets reçus
	auto read_byte = [ptr = rx_buf]() mutable { return *ptr++; };

	if (byte == rpc::header[0]){
		count ++;
	} else {
		count = 0;
	}
	if (count == sizeof rpc::header){
		count = 0;
	} else {
		rx_buf[indice] = byte;
		indice ++;
	}

	if(indice == hanging_order->input_size()){
		indice=0;
		HAL_UART_Transmit(huart_ptr, response_header, sizeof response_header, timeout);
		(*hanging_order)(read_byte, HL_Write_Byte);
		HL_Reset_Interrupt();
	} else {
		HAL_UART_Receive_IT(huart_ptr, &byte, 1);
	}
}

void HL_Reset_Interrupt(size_t) {
	read_stuff_counter = 0;
	HAL_UART_RegisterCallback(huart_ptr, HAL_UART_RX_COMPLETE_CB_ID, HL_Interrupt);
	HAL_UART_Receive_IT(huart_ptr, &byte, 1);
}

//
// Order implementations
//

extern "C" void motion_set_translation_setpoint(uint32_t) {
}

