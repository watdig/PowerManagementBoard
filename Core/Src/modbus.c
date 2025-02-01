/*
 * modbus.c
 *
 *  Created on: Oct 15, 2024
 *      Author: Victor Kalenda
 *
 */

#include "modbus.h"
#include "error_codes.h"
#include "main.h"
#include <stdint.h>
#include <string.h>

// Macros
#define TX_BUFFER_SIZE  RX_BUFFER_SIZE
#define MODBUS_TX_BUFFER_SIZE 256
#define MODBUS_RX_BUFFER_SIZE  256
#define high_byte(value) ((value >> 8) & 0xFF)
#define low_byte(value) (value & 0xFF)
#define word(value1, value2) (((value1 >> 8) & 0xFF) | (value2 & 0xFF))

// Buffers
uint8_t modbus_rx_buffer[MODBUS_RX_BUFFER_SIZE];
uint8_t modbus_tx_buffer[MODBUS_TX_BUFFER_SIZE];
uint8_t rx_chunk[MODBUS_RX_BUFFER_SIZE - 6];
uint16_t tx_buffer[TX_BUFFER_SIZE]; // Master
uint16_t rx_buffer[RX_BUFFER_SIZE]; // Master

// Timing Variables
uint32_t response_interval = 1000;
uint32_t time = 0;
volatile uint32_t chunk_time = 0;

// Interrupt Handling Variables
volatile uint16_t start_index = 0;
volatile uint16_t chunk_start_i = 0;
volatile uint16_t chunk_end_i = 0;
volatile uint16_t modbus_header = 1;
volatile uint8_t rx_int = 0;
volatile uint8_t tx_int = 0;

// External Variables
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint16_t holding_register_database[];

// Private Functions
uint16_t crc_16(uint8_t *data, uint8_t size);
int8_t handle_chunk_miss();
void handle_range(uint16_t holding_register);

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

// Table of CRC values for low-order byte
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/*
 * Modbus reception handler function
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		chunk_start_i = chunk_end_i;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (chunk_start_i + Size > MODBUS_RX_BUFFER_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = MODBUS_RX_BUFFER_SIZE - chunk_start_i;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)(modbus_rx_buffer + chunk_start_i), rx_chunk, datatocopy);  // copy data in that remaining space

			chunk_end_i = (Size - datatocopy);  // update the position
			memcpy ((uint8_t *)modbus_rx_buffer, (uint8_t *)(rx_chunk + datatocopy), chunk_end_i);  // copy the remaining data
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)(modbus_rx_buffer + chunk_start_i), rx_chunk, Size);
			chunk_end_i = Size + chunk_start_i;
		}

		if(modbus_header)
		{
			// Log the time for chunk miss error handling
			chunk_time = HAL_GetTick();

			start_index = chunk_start_i;
			modbus_header = 0;

			// Setup the DMA to receive the # message bytes + crc + 1 in the event that the # bytes is in the message
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_chunk, (uint16_t)(((rx_chunk[4] << 8) | rx_chunk[5])*2 + 2 + 1));
			__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		}
		else
		{
			/*
			 * This is where the message officially completes being received
			 * For Masters: Don'ts set up a reception, modbus stays in idle until you transmit a command again
			 * For Slaves: Don't set up reception since you will need to transmit a response first
			 * Use modbus_set_rx(); as the user to re-enable receive mode
			 */
			modbus_header = 1;
			rx_int = 1;
		}
	}
}

// Transmit Interrupt Handler
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	tx_int = 1;
}


// Chunk Miss Error Handling Function
int8_t handle_chunk_miss()
{
	if(modbus_header == 0)
	{
		if(HAL_GetTick() - chunk_time > 10)
		{
			// TODO: Log the chunk miss as an error
			modbus_header = 1;
			int8_t status = HAL_UART_Abort_IT(&huart1);
			if(status == HAL_OK)
			{
				status = modbus_set_rx();
			}
			return status;
		}
	}
	return HAL_OK;
}



// Modbus Buffer Functions --------------------------------------------------------------------

/**
	Modbus get the data received through modbus
*/
uint16_t get_response_buffer(uint8_t index)
{
	if (index < RX_BUFFER_SIZE - 1)
	{
		// get the value in the uart recieve buffer
		return rx_buffer[index];
	}
	return 0xFFFF;
}

/*
	Modbus get the raw message received through UART
 */
uint8_t get_rx_buffer(uint8_t index)
{
	if (index < MODBUS_RX_BUFFER_SIZE - 1)
	{
		return ((start_index + index) > (MODBUS_RX_BUFFER_SIZE - 1))?
				modbus_rx_buffer[(start_index + index) - MODBUS_RX_BUFFER_SIZE] :
				modbus_rx_buffer[start_index + index];
	}
	return 0xFF;
}

/**
	Place data in transmit buffer
*/
int8_t set_tx_buffer(uint8_t index, uint16_t value)
{
	if (index < TX_BUFFER_SIZE)
	{
		tx_buffer[index] = value;
		return MB_SUCCESS;
	}
	else
	{
		return RANGE_ERROR;
	}
}







// Modbus Master Functions --------------------------------------------------------------------

/**
	Modbus Master reading holding registers
*/
int8_t read_holding_registers(uint16_t read_address, uint16_t read_quantity, uint8_t id)
{
	if(read_quantity > RX_BUFFER_SIZE)
	{
		return MB_MEMORY_ERROR;
	}
	uint8_t index = 0;
	modbus_tx_buffer[index++] = id; // Append Modbus ID

	modbus_tx_buffer[index++] = 0x03; // Append Function Code
	// Append the Read Address (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(read_address);
	modbus_tx_buffer[index++] = low_byte(read_address);
	// Append the quantity of registers to be read (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(read_quantity);
	modbus_tx_buffer[index++] = low_byte(read_quantity);

	int8_t status = modbus_send(modbus_tx_buffer, index);
	if(status != HAL_OK)
	{
		return status;
	}
	// Wait for a response
	uint16_t rx_len = 0;
	status = modbus_poll_for_response(3 + read_quantity * 2 + 2, &rx_len);
	if(status != HAL_OK)
	{
		return status;
	}

	status = modbus_mic(id, 0x03, rx_len);

	store_rx_buffer();

	return status;
}

/**
	Modbus master Write multiple holding registers function
*/
int8_t write_multiple_registers(uint16_t write_address, uint16_t write_quantity, uint8_t id)
{
	uint8_t index = 0;
	// Append Modbus ID
	modbus_tx_buffer[index++] = id;
	// Append Function
	modbus_tx_buffer[index++] = 0x10;
	// Append the Write Address (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(write_address);	modbus_tx_buffer[index++] = low_byte(write_address);
	// Append the quantity of registers to be written (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(write_quantity);
	modbus_tx_buffer[index++] = low_byte(write_quantity);
	// Append the tx_buffer (high byte then low byte
	for(uint8_t i = 0; i < write_quantity; i++)
	{
		modbus_tx_buffer[index++] = high_byte(tx_buffer[i]);
		modbus_tx_buffer[index++] = low_byte(tx_buffer[i]);
	}

	int8_t status = modbus_send(modbus_tx_buffer, index);
	if(status != HAL_OK)
	{
		return status;
	}
	// Wait for a response
	uint16_t rx_len = 0;
	status = modbus_poll_for_response(8, &rx_len);
	if(status != HAL_OK)
	{
		return status;
	}
	return modbus_mic(id, 0x10, rx_len);
}

/*
	Modbus Master message integrity check
 */
int8_t modbus_mic(uint8_t id, uint8_t function_code, uint8_t size)
{
	// Check the slave ID
	if(get_rx_buffer(0) != id)
	{
		return MB_SLAVE_ID_MISMATCH;
	}
	// Check the function code
	if((get_rx_buffer(1) & 0x7F) != function_code)
	{
		return MB_FUNCTION_MISMATCH;
	}

	// Check the modbus exception codes within the response if there is some sort of execution error
	if(((get_rx_buffer(1) >> 7) & 0x01))
	{
		return get_rx_buffer(2) + 0x03;
	}

	// Check the CRC
	if(size >= 5)
	{
		uint16_t crc = 0; //= crc_16(modbus_rx_buffer, size - 2); TODO, FIX CRC FUNCTION FOR MASTER NODE
		if((low_byte(crc) != get_rx_buffer(size - 2)) || (high_byte(crc) != get_rx_buffer(size - 1)))
		{
			return MB_INVALID_CRC;
		}
	}
	return MB_SUCCESS;
}

/*
	Modbus Master wait for a response from a slave after a request
 */
int8_t modbus_poll_for_response(uint8_t size, uint16_t *rx_len)
{
	int8_t status = HAL_OK;

	status = modbus_set_rx();
	if(status != HAL_OK)
	{
		return status;
	}
	while(!rx_int && (HAL_GetTick()) - time < response_interval);
	if(rx_int)
	{
		rx_int = 0;
		return HAL_OK;
	}
	else
	{
		return HAL_TIMEOUT;
	}
}

/*
	Modbus Master set the timeout for how long a slave has to respond to a request
 */
void set_response_interval(uint32_t delay)
{
	response_interval = delay;
}

uint32_t get_response_interval()
{
	return response_interval;
}

/*
	Master Modbus rx_buffer allocation function
 */
void store_rx_buffer()
{
	// Store the messages data in the rx_buffer
	for(uint8_t i = 0; i < get_rx_buffer(2); i++)
	{
		if(i < RX_BUFFER_SIZE)
		{
			rx_buffer[i] = (get_rx_buffer(2 * i + 3) << 8) | get_rx_buffer(2 * i + 4);
		}
		// rx_buffer_len = i;
	}
}


// Modbus Slave Functions ---------------------------------------------------------------------

/*
	Modbus Slave Return Multiple holding registers
 */
int8_t return_holding_registers()
{
	// Handle Error Checking
	uint16_t first_register_address = (get_rx_buffer(2) << 8) | get_rx_buffer(3);

	// Get the number of registers requested by the master
	uint16_t num_registers = (get_rx_buffer(4) << 8) | get_rx_buffer(5);

	if(num_registers > RX_BUFFER_SIZE || num_registers < 1) // 125 is the limit according to modbus protocol
	{
		return modbus_exception(MB_ILLEGAL_DATA_VALUE);
	}

	uint16_t last_register_address = first_register_address + (num_registers - 1);

	if(last_register_address > NUM_HOLDING_REGISTERS)
	{
		return modbus_exception(MB_ILLEGAL_DATA_ADDRESS);
	}

	// Return register values

	modbus_tx_buffer[0] = get_rx_buffer(0); // Append Slave id
	modbus_tx_buffer[1] = get_rx_buffer(1); // Append Function Code
	modbus_tx_buffer[2] = num_registers * 2; // Append number of bytes
	uint8_t index = 3;

	// Append the Register Values
	for(uint8_t i = 0; i < num_registers; i++)
	{
		modbus_tx_buffer[index++] = high_byte(holding_register_database[first_register_address + i]);
		modbus_tx_buffer[index++] = low_byte(holding_register_database[first_register_address + i]);
	}

	return modbus_send(modbus_tx_buffer, index);
}

/*
	Modbus Slave Edit Multiple holding registers
 */
int8_t edit_multiple_registers()
{
	// Handle Error Checking
	uint16_t first_register_address = (get_rx_buffer(2) << 8) | get_rx_buffer(3);

	uint16_t num_registers = (get_rx_buffer(4) << 8) | get_rx_buffer(5);

	if(num_registers > 125 || num_registers < 1) // 125 is the limit according to modbus protocol
	{
		return modbus_exception(MB_ILLEGAL_DATA_VALUE);
	}

	uint16_t last_register_address = first_register_address + (num_registers - 1);

	if(last_register_address > NUM_HOLDING_REGISTERS)
	{
		return modbus_exception(MB_ILLEGAL_DATA_ADDRESS);
	}

	if((first_register_address <= GPIO_READ) && (last_register_address >= GPIO_READ))
	{
		// Ensure that sensor values are restricted to read-only
		return modbus_exception(MB_ILLEGAL_FUNCTION);
	}

	// Edit holding registers
	modbus_tx_buffer[0] = get_rx_buffer(0); // Append Slave id
	modbus_tx_buffer[1] = get_rx_buffer(1); // Append Function Code
	// Append the Write Address (high byte then low byte)
	modbus_tx_buffer[2] = get_rx_buffer(2);
	modbus_tx_buffer[3] = get_rx_buffer(3);
	// Append the quantity of registers to be written (high byte then low byte)
	modbus_tx_buffer[4] = get_rx_buffer(4);
	modbus_tx_buffer[5] = get_rx_buffer(5);
	uint8_t index = 6;

	for(uint8_t i = 0; i < num_registers; i++)
	{
		holding_register_database[first_register_address + i] = (get_rx_buffer(2 * i + 7) << 8) | get_rx_buffer(2 * i + 8);

		// Handle the range boundaries of each writable register
		handle_range(first_register_address + i);
	}

	// TIMING WORKAROUND START
	HAL_Delay(1);
	// TIMING WORKAROUND END

	int8_t status = modbus_send(modbus_tx_buffer, index);

	if(status == HAL_OK)
	{
		// Special Case Modbus Baud Rate Modification
		if((first_register_address <= 1) && last_register_address >= 1)
		{
			return modbus_change_baud_rate();
		}
	}
	return status;
}

/*
	Modbus Slave Exception handler
 */
int8_t modbus_exception(int8_t exception_code)
{
	modbus_tx_buffer[0] = get_rx_buffer(0);
	modbus_tx_buffer[1] = get_rx_buffer(1) | 0x80;
	modbus_tx_buffer[2] = exception_code - 3; // Subtract 3 to match the modbus defined error code value

	return modbus_send(modbus_tx_buffer, 3);
}

/*
 * Modbus Slave Data Value Range Handler
 */
void handle_range(uint16_t holding_register)
{
	switch(holding_register)
	{
		case MODBUS_ID:
		{
			if(holding_register_database[holding_register] > 0xFF)
			{
				holding_register_database[holding_register] = 0xFF;
			}
			break;
		}
		case MB_BAUD_RATE:
		{
			if(holding_register_database[holding_register] < 2)
			{
				holding_register_database[holding_register] = 2;
			}
			else if(holding_register_database[holding_register] > 9)
			{
				holding_register_database[holding_register] = 9;
			}
			break;
		}
	}
}



// General Modbus Functions -------------------------------------------------------------------

/*
	General Modbus send function
 */
int8_t modbus_send(uint8_t *data, uint8_t size)
{
	// Append CRC (low byte then high byte)
	uint16_t crc = crc_16(modbus_tx_buffer, size);
	modbus_tx_buffer[size] = low_byte(crc);
	modbus_tx_buffer[size + 1] = high_byte(crc);

	int8_t status = HAL_OK;
	status = HAL_UART_Transmit_IT(&huart1, modbus_tx_buffer, size + 2);
	if(status != HAL_OK)
	{
		return status;
	}
	time = HAL_GetTick();
	while(!tx_int && ((HAL_GetTick()) - time < 100));
	if(tx_int)
	{
		tx_int = 0;
		return HAL_OK;
	}
	else
	{
		return HAL_TIMEOUT;
	}
}

/*
	General Modbus check for reception function
 */
uint8_t modbus_rx()
{
	if(rx_int)
	{
		rx_int = 0;
		return 1;
	}
	if(handle_chunk_miss() != HAL_OK)
	{
		// TODO: log the error when startup the UART back up
	}
	return rx_int;
}

/*
	General Modbus set chip in receive mode
 */
int8_t modbus_set_rx()
{
	int8_t status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_chunk, 6);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

	return status;
}







// General Modbus Control Functions ------------------------------------------------------------

int8_t modbus_change_baud_rate()
{
	int8_t status = 0;

	switch(holding_register_database[1])
	{
		case BAUD_RATE_4800:
		{
			huart1.Init.BaudRate = 4800;
			break;
		}
		case BAUD_RATE_9600:
		{
			huart1.Init.BaudRate = 9600;
			break;
		}
		case BAUD_RATE_19200:
		{
			huart1.Init.BaudRate = 19200;
			break;
		}
		case BAUD_RATE_38400:
		{
			huart1.Init.BaudRate = 38400;
			break;
		}
		case BAUD_RATE_57600:
		{
			huart1.Init.BaudRate = 57600;
			break;
		}
		case BAUD_RATE_115200:
		{
			huart1.Init.BaudRate = 115200;
			break;
		}
		case BAUD_RATE_128000:
		{
			huart1.Init.BaudRate = 128000;
			break;
		}
		case BAUD_RATE_256000:
		{
			huart1.Init.BaudRate = 256000;
			break;
		}
		default:
		{
			holding_register_database[1] = BAUD_RATE_9600;
			huart1.Init.BaudRate = 9600;
			status = UART_SetConfig(&huart1);
			if(status == HAL_OK)
			{
				//HAL_UART_Abort_IT(&huart1);
			}
			return MB_ILLEGAL_DATA_VALUE;
			break;
		}

	}
	status = UART_SetConfig(&huart1);
	if(status == HAL_OK)
	{
		//status = HAL_UART_Abort_IT(&huart1);
	}

	if(status != HAL_OK)
	{
		return status;
	}

	//status = HAL_UART_Receive_IT(huart, pData, Size)

	return status;
}

int8_t modbus_set_baud_rate(uint8_t baud_rate)
{
	int8_t status = HAL_OK;
	/* Designed to hold baud rate in emulated EEPROM
	if(ee.modbus_baud_rate != baud_rate)
	{
		ee.modbus_baud_rate = baud_rate;

		if(!EE_Write())
		{
			osMutexRelease(eeprom_mutexHandle);
			return EE_WRITE_ERROR;
		}
	}
	*/
	return status;
}

int8_t modbus_get_baud_rate(uint8_t* baud_rate)
{
	int8_t status = HAL_OK;

	/* Designed to hold baud rate in emulated EEPROM
	*baud_rate = ee.modbus_baud_rate;
	*/

	return status;
}

uint8_t significant_error(int8_t status)
{
  switch(status)
  {
    case MB_ILLEGAL_FUNCTION ... MB_SLAVE_ERROR:
    {
      return 1;
    }
    case MB_ACK:
    {
      return 0;
    }
    case MB_SLAVE_BUSY ... MB_NEGATIVE_ACK:
    {
      return 1;
    }
    case MB_MEMORY_ERROR:
    {
      return 0;
    }
    case MB_GATEWAY_PATH_ERROR ... MB_FUNCTION_MISMATCH:
    {
      return 1;
    }
    case MB_INVALID_CRC:
    {
      return 0;
    }
    default:
    {
      return 0;
    }
  }
}

// CRC Generation Function
uint16_t crc_16(uint8_t *data, uint8_t size)
{
	uint8_t crc_hi = 0xFF;
	uint8_t crc_low = 0xFF;
	 unsigned int i; /* will index into CRC lookup */

	/* pass through message buffer */
	while (size--)
	{
		i = crc_low ^ *data++; /* calculate the CRC  */
		crc_low = crc_hi ^ table_crc_hi[i];
		crc_hi = table_crc_lo[i];
	}

	return (crc_hi << 8 | crc_low);
}
