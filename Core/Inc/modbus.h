/*
 * modbus.h
 *
 *  Created on: Oct 15, 2024
 *      Author: Victor Kalenda
 */

#include <stdint.h>

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

typedef enum baud_rate_e
{
	// Most sensors are capable of 2400 baud, however the STM32 is limited between 2442 bps - 10 000 000 bps
	BAUD_RATE_4800 = 2,
	BAUD_RATE_9600,
	BAUD_RATE_19200,
	BAUD_RATE_38400,
	BAUD_RATE_57600,
	BAUD_RATE_115200,
	BAUD_RATE_128000,
	BAUD_RATE_256000,
}baud_rate_t;

#define RX_BUFFER_SIZE 125

// Error Codes
#define MB_SUCCESS 			0x00

// Modbus Buffer Functions --------------------------------------------------------------------
uint16_t get_response_buffer(uint8_t index);
uint8_t get_rx_buffer(uint8_t index);
int8_t set_tx_buffer(uint8_t index, uint16_t value);

// Modbus Master Functions --------------------------------------------------------------------
int8_t read_holding_registers(uint16_t read_address, uint16_t read_quantity, uint8_t id);
int8_t write_multiple_registers(uint16_t write_address, uint16_t write_quantity, uint8_t id);
int8_t modbus_mic(uint8_t id, uint8_t function_code, uint8_t size);
void set_response_interval(uint32_t delay);
uint32_t get_response_interval();
int8_t modbus_poll_for_response(uint8_t size, uint16_t *rx_len);

// Modbus Slave Functions ---------------------------------------------------------------------
int8_t return_holding_registers();
int8_t edit_multiple_registers();
int8_t modbus_exception(int8_t exception_code);


// General Modbus Functions -------------------------------------------------------------------
int8_t modbus_send(uint8_t *data, uint8_t size);
uint8_t modbus_rx();
int8_t modbus_set_rx();
void store_rx_buffer();



// General Modbus Control Functions ------------------------------------------------------------
int8_t modbus_change_baud_rate();
int8_t modbus_set_baud_rate(uint8_t baud_rate);
int8_t modbus_get_baud_rate(uint8_t *baud_rate);
uint8_t significant_error(int8_t status);

#endif /* INC_MODBUS_H_ */
