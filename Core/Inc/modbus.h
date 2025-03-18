/*
 * modbus.h
 *
 *  Created on: Oct 15, 2024
 *      Author: Victor Kalenda
 */

#include <stdint.h>

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

/*
 * Choose what modbus features/capabilities you would like to include for your project
 * MB_MASTER: Include modbus master functions and capabilities
 * MB_SLAVE: Include modbus slave functions and capabilities
 */
#define MB_SLAVE

#define RX_BUFFER_SIZE 125

// Error Codes
#define MB_SUCCESS 			0x00
typedef enum baud_rate_e
{
	BAUD_RATE_2400 = 1,
	BAUD_RATE_4800,
	BAUD_RATE_9600,
	BAUD_RATE_19200,
	BAUD_RATE_38400,
	BAUD_RATE_57600,
	BAUD_RATE_115200,
	BAUD_RATE_128000,
	BAUD_RATE_256000,
}baud_rate_t;



// Modbus Master Functions --------------------------------------------------------------------
#ifdef MB_MASTER
int8_t set_transmit_buffer(uint8_t index, uint16_t value);
uint16_t get_response_buffer(uint8_t index);
int8_t read_holding_registers(uint16_t read_address, uint16_t read_quantity, uint8_t id);
int8_t write_multiple_registers(uint16_t write_address, uint16_t write_quantity, uint8_t id);
int8_t modbus_mic(uint8_t id, uint8_t function_code, uint8_t size);
uint8_t response_received();
void set_response_interval(uint32_t delay);
uint32_t get_response_interval();
#endif

// Modbus Slave Functions ---------------------------------------------------------------------
#ifdef MB_SLAVE
int8_t return_holding_registers(uint8_t *tx_len);
int8_t edit_multiple_registers(uint8_t *tx_len);
int8_t modbus_exception(int8_t exception_code);
#endif

// General Modbus Functions -------------------------------------------------------------------
int8_t modbus_send(uint8_t size);
uint8_t modbus_rx();
int8_t modbus_set_rx();
void store_rx_buffer();
int8_t monitor_modbus();
int8_t modbus_reset();

// General Modbus Control Functions ------------------------------------------------------------
int8_t modbus_startup();
int8_t modbus_shutdown();
int8_t modbus_change_baud_rate();
int8_t modbus_set_baud_rate(uint8_t baud_rate);
int8_t modbus_get_baud_rate(uint8_t *baud_rate);

// Low Level Functions -------------------------------------------------------------------------
uint8_t get_rx_buffer(uint8_t index);
int8_t handle_modbus_error(int8_t error_code);

#endif /* INC_MODBUS_H_ */
