extern "C" {

#include "hl.h"

} // extern "C"
#include "hl.hpp"

#include <utility>

#include "stm32g4xx_hal_uart.h"

#include <k2o/dispatcher.hpp>
#include <order/motion.h>
#include <order/controller.h>
#include <order/dxl.h>
#include <rpc/def.hpp>
#include <rpc/master.hpp>

#define RX_BUF_SIZE 128

static void HL_Interrupt(UART_HandleTypeDef *);
static void HL_Interrupt_Get_Order(UART_HandleTypeDef *);
static void HL_Interrupt_Call_Order(UART_HandleTypeDef *);
static void HL_Reset_Interrupt(size_t = 0);

static UART_HandleTypeDef *huart_ptr = NULL;

static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t indice =0;
static size_t read_stuff_counter = 0;
static size_t write_stuff_counter = 0;
static const uint32_t timeout = 1000;
static uint8_t byte = 0x00; //receive byte from HL_INTERRUPT
static uint8_t count = 0; //counts number of time you receive 0xff
static uint8_t stuffing_byte = ~rpc::header[0];

static k2o::dispatcher dispatcher{rpc::master::keyring};
static k2o::order *hanging_order = NULL;

extern "C" void HL_Init(UART_HandleTypeDef *handler_ptr) {
	huart_ptr = handler_ptr;
	HL_Reset_Interrupt();
}

void HL_Start_Frame(rpc::Frame_Type frame_type) {
	auto type_byte = static_cast<uint8_t>(frame_type);

	write_stuff_counter = 0;
	HAL_UART_Transmit(huart_ptr, &stuffing_byte, 1, timeout);
	HAL_UART_Transmit(huart_ptr, rpc::header, sizeof rpc::header, timeout);
	HAL_UART_Transmit(huart_ptr, &type_byte, 1, timeout);
}

void HL_Write_Byte(upd::byte_t byte) {
	if (byte == rpc::header[0]) {
		write_stuff_counter++;
	} else {
		write_stuff_counter = 0;
	}
	HAL_UART_Transmit(huart_ptr, &byte, 1, timeout);
	if (write_stuff_counter == sizeof rpc::header - 1) {
		write_stuff_counter = 0;
		HAL_UART_Transmit(huart_ptr, &stuffing_byte, 1, timeout);
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
			if (order.input_size() > 0) {
				hanging_order = &order;
				HAL_UART_RegisterCallback(huart_ptr, HAL_UART_RX_COMPLETE_CB_ID, &HL_Interrupt_Call_Order);
				HAL_UART_Receive_IT(huart_ptr, &byte, 1);
			} else {
				HL_Start_Frame(rpc::Frame_Type::RESPONSE);
				order([]() { return uint8_t{}; }, HL_Write_Byte);
				HL_Reset_Interrupt();
			}
	};

	if (read_byte() != static_cast<uint8_t>(rpc::Frame_Type::REQUEST)) {
		HL_Reset_Interrupt();
		return;
	}

	dispatcher.get_order(read_byte)
			.map(handle_order)
			.map_error(HL_Reset_Interrupt);
}


void HL_Interrupt_Call_Order(UART_HandleTypeDef *) {
	if (count == sizeof rpc::header - 1) {
		count = 0;
	} else {
		rx_buf[indice] = byte;
		indice ++;
	}
	if (byte == rpc::header[0]){
		count ++;
	} else {
		count = 0;
	}

	if(indice == hanging_order->input_size()){
		auto read_byte = [ptr = rx_buf]() mutable { return *ptr++; };

		HL_Start_Frame(rpc::Frame_Type::RESPONSE);
		(*hanging_order)(read_byte, HL_Write_Byte);
		HL_Reset_Interrupt();
	} else {
		HAL_UART_Receive_IT(huart_ptr, &byte, 1);
	}
}

void HL_Reset_Interrupt(size_t) {
	read_stuff_counter = 0;
	indice = 0;
	HAL_UART_RegisterCallback(huart_ptr, HAL_UART_RX_COMPLETE_CB_ID, HL_Interrupt);
	HAL_UART_Receive_IT(huart_ptr, &byte, 1);
}
