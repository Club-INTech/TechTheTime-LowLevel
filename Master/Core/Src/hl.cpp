#include "hl.h"

#include "stm32g4xx_hal_uart.h"

#include <k2o/dispatcher.hpp>
#include <order/motion.h>
#include <order/controller.h>
#include <rpc/def.hpp>
#include <rpc/master.hpp>
#include <rpc/controller.hpp>

#define RX_BUF_SIZE 128

static void HL_Interrupt(UART_HandleTypeDef *);
static void HL_Interrupt_Get_Order(UART_HandleTypeDef *);
static void HL_Interrupt_Call_Order(UART_HandleTypeDef *);
static void HL_Reset_Interrupt(size_t = 0);

static UART_HandleTypeDef *huart_ptr = NULL;

static uint8_t byte_stuff = 0x00;
static const uint32_t timeout = 1000;

static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t read_stuff_counter = 0;
static uint8_t write_stuff_counter=0;

static k2o::dispatcher dispatcher{rpc::master::keyring};
static k2o::order *hanging_order = NULL;

static void HL_Write_Byte(uint8_t byte) {
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

extern "C" void HL_Init(UART_HandleTypeDef *handler_ptr) {
	huart_ptr = handler_ptr;

}

static uint32_t dummy_time_us = 0;
extern "C" void HL_Send_Measure(void) {
	auto report_measure_key = rpc::controller::keyring.get<controller_report_measure>();
	HAL_UART_Transmit(huart_ptr, rpc::header, sizeof(rpc::header), timeout);
	report_measure_key(dummy_time_us, 0, 0) >> HL_Write_Byte;
	dummy_time_us += 1000000ull;
}

extern "C" void motion_set_translation_setpoint(uint32_t) {
}

//
// Interrupt routines
//

void HL_Interrupt(UART_HandleTypeDef *) {
	// Se charge de trouver une entête et de se placer au début d'une trame
	// C'est la même logique que la fonction HL_Interrupt en mode bloquant, sauf qu'il faut traiter les octets dès qu'ils sont reçus.
	// Cette fonction doit être maintenu en tant qu'interruption tant qu'aucune entête complète n'a été trouvée.
	// Lorsqu'une trame a été trouvée, la fonction 'HL_Interrupt_Get_Order' doit être appelé lorsque 2 octets ont été reçus pour réccupérer l'ordre à exécuter.
	//
	// ASTUCES :
	//	- HAL_UART_Receive_IT(huart_ptr, rx_buf, 1) permet de rappeller la routine d'interruption lorsqu'un octet a été reçu et le place dans 'rx_buf'.
	//	- HAL_UART_RegisterCallback(huart_ptr, HAL_UART_RX_COMPLETE_CB_ID, [pointeur vers fonction]) permet de changer la fonction appellée lorsque le nombre d'octets demandés a été reçus.
}

void HL_Interrupt_Get_Order(UART_HandleTypeDef *) {
	// Réccupère l'ordre à appeller en fonction de la valeur reçu
	// Un pointeur vers l'ordre doit être maintenu dans 'hanging_order'.
	// A la fin de cette fonction, la fonction 'HL_Interrupt_Call_Order' doit être appellé à chaque octet reçu jusqu'à ce que suffisament d'octets soient reçus pour exécuter l'ordre
	//
	// ASTUCES :
	//	- La syntaxe pour réccupérer l'ordre est un peu particulière :
	//		dispatcher.get_order(read_byte).map([fonction en cas de succès]).map_error(HL_Reset_Interrupt);
	//	  La fonction en question n'est appellée que lorsqu'un ordre valide a été réccupéré.
	//	  Elle doit vérifier la signature suivante : void(k2o::order &order)
	//	  En cas d'erreur, l'état de 'huart_ptr' est modifié en conséquence automatiquement (pas la peine donc de l'implémenter)
	//	- Les trame de réponse sont ignorées (i.e. celle qui ne sont pas de type rpc::REQUEST).
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
}

void HL_Interrupt_Call_Order(UART_HandleTypeDef *) {
	// Exécute 'hanging_order' avec les octets reçus
	// Une fois la fonction exécutée, la fonction 'HL_Interrupt' est réassignée en tant que routine d'interruption.
	//
	// ASTUCES :
	//	- Il faut traiter les octets un par un, en se chargeant du bourrage d'octet.
	//	- La syntaxe suivante permet de réaliser l'ordre : (*hanging_order)(read_byte, write_byte)
	//	  Après cette ligne, les octets à envoyer sont stockés dans 'tx_buf'.
	//  - Ne pas oublier de renvoyer les octets dans 'tx_buf' (en mode bloquant) avec l'entête dédiée au réponse ! (la variable 'response_header')
	//	- Ne pas oublier d'insérer l'octet indiquant le type de la trame (i.e. rpc::REQUEST).
	auto read_byte = [ptr = rx_buf]() mutable { return *ptr++; };
}

void HL_Reset_Interrupt(size_t) {
	read_stuff_counter = 0;
	HAL_UART_RegisterCallback(huart_ptr, HAL_UART_RX_COMPLETE_CB_ID, HL_Interrupt);
}
