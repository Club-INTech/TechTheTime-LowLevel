#include "dxl.h"
#include "order/dxl.h"

#include "crc_calculation.h"

uint8_t Pos0 = 0;
uint8_t Pos1 = 0;
uint8_t Pos2 = 0;
uint8_t Pos3 = 0;

static UART_HandleTypeDef *handler_dxl = NULL;

void DXL_Init(UART_HandleTypeDef *_handler_dxl) {
	handler_dxl = _handler_dxl;
#if INTECH_TARGET == 1
	uint8_t Torque_On_All[] = { 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x27, 0x00, 0x83,
			0x40, 0x00, 0x01, 0x00, 0x02, 0x01, 0x03, 0x01, 0x04, 0x01, 0x05,
			0x01, 0x06, 0x01, 0x07, 0X01, 0x08, 0X01, 0X09, 0x01, 0x0A, 0x01,
			0x0B, 0x01, 0x0C, 0x01, 0x0D, 0x01, 0x0E, 0x01, 0x0F, 0x01, 0x10,
			0x01, 0x11, 0x01, 0x00, 0x00 };
#elif INTECH_TARGET == 2
	uint8_t Torque_On_All[] = {
			0xff, 0xff, 0xfd, 0x00, // header
			0xfe, // broadcast
			0, 0, // packet length
			0x83, // read write
			0x40, 0x00, // torque enable
			1, 0, // data length
			3, 1, // [3] <- 1
			4, 1, // [4] <- 1
			5, 1, // [5] <- 1
			0, 0 // CRC
	};
	Torque_On_All[5] = sizeof Torque_On_All - 7;
#endif // INTECH_TARGET == 1, INTECH_TARGET == 2
	update_crc_packet(Torque_On_All, sizeof(Torque_On_All));
	DXL_Transmit(Torque_On_All, sizeof(Torque_On_All));
}

void DXL_Transmit(uint8_t *packet, uint16_t size_packet) {
	HAL_UART_Transmit(handler_dxl, packet, size_packet, 1000);
}

void DXL_Torque_On(uint8_t id) {
	uint8_t Torque_On[] = { 0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03, 0x40,
			0x00, 0x01, 0x00, 0x00 };
	update_crc_packet(Torque_On, 13);
	HAL_UART_Transmit(handler_dxl, Torque_On, 13, 1000);
	HAL_Delay(100);
}

void DXL_Torque_Off(uint8_t id) {
	uint8_t Torque_Off[] = { 0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03, 0x40,
			0x00, 0x00, 0x00, 0x00 };
	update_crc_packet(Torque_Off, 13);
	HAL_UART_Transmit(handler_dxl, Torque_Off, 13, 1000);
}

void DXL_Position(uint8_t id, uint32_t goal_position) {
	Pos0 = (goal_position & 0x000000FF);
	Pos1 = (goal_position & 0x0000FF00) >> 8;
	Pos2 = (goal_position & 0x00FF0000) >> 16;
	Pos3 = (goal_position & 0xFF000000) >> 24;
	uint8_t Moov[] = { 0xFF, 0xFF, 0xFD, 0x00, id, 0x09, 0x00, 0x03, 0x74, 0x00,
			Pos0, Pos1, Pos2, Pos3, 0x00, 0x00 };
	update_crc_packet(Moov, 16);
	HAL_UART_Transmit(handler_dxl, Moov, 16, 1000);
}


void DXL_Position_Angle(uint8_t id, int32_t goal_angle){

	if ( goal_angle < -180  || goal_angle > 180) {
		return;
	}
	else {
		if ( id == 3) {
			goal_angle =  - goal_angle + 180;
		}
		else if (id == 4 ) {
			goal_angle = goal_angle + 140;
		}
		else if ( id == 5) {
			goal_angle = - goal_angle + 255;
		}
		else {
			if ( id % 2 == 0 ) {
				goal_angle = 135 - goal_angle;
			}
			else {
				goal_angle = 180 + goal_angle;
			}
		}
		uint32_t goal_position = round(11.375 * goal_angle);
		Pos0 = (goal_position & 0x000000FF);
		Pos1 = (goal_position & 0x0000FF00) >> 8;
		Pos2 = (goal_position & 0x00FF0000) >> 16;
		Pos3 = (goal_position & 0xFF000000) >> 24;
		uint8_t Moov[] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x09, 0x00, 0x03, 0x74, 0x00, Pos0, Pos1, Pos2, Pos3, 0x00, 0x00};
		update_crc_packet(Moov, 16);
		HAL_UART_Transmit(handler_dxl, Moov, 16, 1000);
	}
}

void DXL_Light_On(uint8_t id) {
	uint8_t Light_On[] = { 0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03, 0x41,
			0x00, 0x01, 0x00, 0x00 };
	update_crc_packet(Light_On, 13);
	HAL_UART_Transmit(handler_dxl, Light_On, 13, 1000);
}

void DXL_Light_Off(uint8_t id) {
	uint8_t Light_Off[] = { 0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03, 0x41,
			0x00, 0x00, 0x00, 0x00 };
	update_crc_packet(Light_Off, 13);
	HAL_UART_Transmit(handler_dxl, Light_Off, 13, 1000);
}

void DXL_Update_Id(uint8_t previous_id, uint8_t next_id) {
	DXL_Torque_Off(previous_id);
	uint8_t Id[] = { 0xFF, 0xFF, 0xFD, 0x00, previous_id, 0x06, 0x00, 0x03,
			0x07, 0x00, next_id, 0x00, 0x00 };
	update_crc_packet(Id, 13);
	HAL_UART_Transmit(handler_dxl, Id, 13, 1000);
}

void DXL_Sync_Position(uint8_t *tab, uint8_t size_tab) {
	uint16_t crc_accum = 0;
	uint8_t number_dxl = tab[0];
	//uint8_t size_packet = 14 + 3 * number_dxl;
	uint16_t len_packet = 7 + 3 * number_dxl;
	uint8_t len_packet_1 = (len_packet & 0x00FF);
	uint8_t len_packet_2 = (len_packet & 0xFF00) >> 8;
	uint8_t Packet_Head[12] = { 0xFF, 0xFF, 0xFD, 0x00, 0xFE, len_packet_1,
			len_packet_2, 0x83, 0x74, 0x00, 0x02, 0x00 };
	crc_accum = update_crc(crc_accum, Packet_Head, 12);
	DXL_Transmit(Packet_Head, 12);
	for (int i = 0; i < number_dxl; i++) {
		uint16_t pos = tab[number_dxl + 1 + i];
		uint8_t pos_1 = (pos & 0x00FF);
		uint8_t pos_2 = (pos & 0xFF00) >> 8;
		uint8_t Packet_DXL[3] = { tab[1 + i], pos_1, pos_2 };
		crc_accum = update_crc(crc_accum, Packet_DXL, 3);
		DXL_Transmit(Packet_DXL, 3);
	};
	uint8_t crc_1 = (crc_accum & 0x00FF);
	uint8_t crc_2 = (crc_accum & 0xFF00) >> 8;
	uint8_t Packet_CRC[2] = { crc_1, crc_2 };
	DXL_Transmit(Packet_CRC, 2);
}
