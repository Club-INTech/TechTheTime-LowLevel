#include "dxl.h"

uint8_t Pos0 = 0;
uint8_t Pos1 = 0;
uint8_t Pos2 = 0;
uint8_t Pos3 = 0;

static UART_HandleTypeDef *handler_dxl = NULL;

void DXL_Init(UART_HandleTypeDef *_handler_dxl) {
	handler_dxl = _handler_dxl;
}

void DXL_Transmit(uint8_t *packet, uint16_t size_packet) {
	HAL_UART_Transmit(handler_dxl, packet, size_packet, 1000);
}

void DXL_Torque_On(uint8_t id) {
	uint8_t Torque_On[] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01, 0x00, 0x00};
	update_crc_packet(Torque_On, 13);
	HAL_UART_Transmit(handler_dxl, Torque_On, 13, 1000);
}

void DXL_Torque_Off(uint8_t id) {
	uint8_t Torque_Off[] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00};
	update_crc_packet(Torque_Off, 13);
	HAL_UART_Transmit(handler_dxl, Torque_Off, 13, 1000);
}

void DXL_Position(uint8_t id, uint32_t goal_position){
	Pos0 = (goal_position & 0x000000FF);
	/*
	Pos1 = (goal_position >> 8) & 0x00FF;
	Pos2 = (goal_position >> 16) & 0x00FF;
	Pos3 = (goal_position >> 24) & 0x00FF;
	*/
	Pos1 = (goal_position & 0x0000FF00) >> 8;
	Pos2 = (goal_position & 0x00FF0000) >> 16;
	Pos3 = (goal_position & 0xFF000000) >> 24;
	uint8_t Moov[] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x09, 0x00, 0x03, 0x74, 0x00, Pos0, Pos1, Pos2, Pos3, 0x00, 0x00};
	update_crc_packet(Moov, 16);
	HAL_UART_Transmit(handler_dxl, Moov, 16, 1000);
}

void DXL_Light_On(uint8_t id) {
	uint8_t Light_On[] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03, 0x41, 0x00, 0x01, 0x00, 0x00};
	update_crc_packet(Light_On, 13);
	HAL_UART_Transmit(handler_dxl, Light_On, 13, 1000);
}

void DXL_Light_Off(uint8_t id) {
	uint8_t Light_Off[] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03, 0x41, 0x00, 0x00, 0x00, 0x00};
	update_crc_packet(Light_Off, 13);
	HAL_UART_Transmit(handler_dxl, Light_Off, 13, 1000);
}

void DXL_Update_Id(uint8_t previous_id, uint8_t next_id) {
	uint8_t Id[] = {0xFF, 0xFF, 0xFD, 0x00, previous_id, 0x06, 0x00, 0x03, 0x07, 0x00, next_id, 0x00, 0x00};
	update_crc_packet(Id, 13);
	HAL_UART_Transmit(handler_dxl, Id, 13, 1000);
}

