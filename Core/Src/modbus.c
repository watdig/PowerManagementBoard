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

// Buffer variables
uint8_t modbus_rx_buffer[MODBUS_RX_BUFFER_SIZE];
uint8_t modbus_tx_buffer[MODBUS_TX_BUFFER_SIZE];
#ifdef MB_MASTER
uint16_t tx_buffer[TX_BUFFER_SIZE];
uint16_t response_buffer[RX_BUFFER_SIZE];

// Master Response variables
uint8_t target_id = 0;
uint8_t target_function_code = 0;
uint16_t expected_rx_len = 0;
uint8_t response_rx = 0;

// Timing Variables
uint32_t rx_time = 0;
uint32_t response_interval = 1000;
#endif // MB_MASTER
uint32_t tx_time = 0;
volatile uint32_t chunk_time = 0;

// Interrupt Handling Variables
volatile uint16_t modbus_header = 1;
volatile uint8_t uart_rx_int = 0;
volatile uint8_t uart_tx_int = 1;
volatile uint8_t uart_err_int = 0;

// External Variables
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
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
	if(modbus_header)
	{
		// Log the time for chunk miss error handling
		chunk_time = HAL_GetTick();
		modbus_header = 0;

		// Setup the DMA to receive the # message bytes + crc + 1 in the event that the # bytes is in the message
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &modbus_rx_buffer[6], (uint16_t)(((modbus_rx_buffer[4] << 8) | modbus_rx_buffer[5])*2 + 2 + 1));
		__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
	}
	else
	{
		/*
		 * This is where the message officially completes being received
		 * For Masters: Don't set up a reception, modbus stays in idle until you transmit a command again
		 * For Slaves: Don't set up reception since you will need to transmit a response first
		 * Use modbus_set_rx(); as the user to re-enable receive mode
		 */
		modbus_header = 1;
		uart_rx_int = 1;
#ifdef MB_SLAVE
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer, 6);
		__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
#endif
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_tx_int = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uart_err_int = 1;
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_MASK);
}


// Modbus Master Functions --------------------------------------------------------------------
#ifdef MB_MASTER
uint16_t get_response_buffer(uint8_t index)
{
	if (index < RX_BUFFER_SIZE - 1)
	{
		// get the value in the uart recieve buffer
		return response_buffer[index];
	}
	return 0xFFFF;
}

int8_t set_transmit_buffer(uint8_t index, uint16_t value)
{
	if (index < TX_BUFFER_SIZE)
	{
		tx_buffer[index] = value;
		return MB_SUCCESS;
	}
	else
	{
		return handle_modbus_error(RANGE_ERROR);
	}
}

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

	int8_t status = modbus_send(index);
	if(status != HAL_OK)
	{
		return status;
	}
	// Setup the master to expect a response
	target_id = id;
	target_function_code = 0x03;
	expected_rx_len = 3 + read_quantity * 2 + 2; // This will enable rx timeout monitoring
	return modbus_set_rx();
}

int8_t write_multiple_registers(uint16_t write_address, uint16_t write_quantity, uint8_t id)
{
	uint8_t index = 0;
	// Append Modbus ID
	modbus_tx_buffer[index++] = id;
	// Append Function
	modbus_tx_buffer[index++] = 0x10;
	// Append the Write Address (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(write_address);
	modbus_tx_buffer[index++] = low_byte(write_address);
	// Append the quantity of registers to be written (high byte then low byte)
	modbus_tx_buffer[index++] = high_byte(write_quantity);
	modbus_tx_buffer[index++] = low_byte(write_quantity);
	// Append the tx_buffer (high byte then low byte
	for(uint8_t i = 0; i < write_quantity; i++)
	{
		modbus_tx_buffer[index++] = high_byte(tx_buffer[i]);
		modbus_tx_buffer[index++] = low_byte(tx_buffer[i]);
	}

	int8_t status = modbus_send(index);
	if(status != HAL_OK)
	{
		return status;
	}
	// Wait for a response
	target_id = id;
	target_function_code = 0x10;
	expected_rx_len = 8;
	return modbus_set_rx();
}

int8_t modbus_mic(uint8_t id, uint8_t function_code, uint8_t size)
{
	// Check the slave ID
	if(get_rx_buffer(0) != id)
	{
		return handle_modbus_error(MB_SLAVE_ID_MISMATCH);
	}
	// Check the function code
	if((get_rx_buffer(1) & 0x7F) != function_code)
	{
		return handle_modbus_error(MB_FUNCTION_MISMATCH);
	}

	// Check the modbus exception codes within the response if there is some sort of execution error
	if(((get_rx_buffer(1) >> 7) & 0x01))
	{
		return get_rx_buffer(2) + 0x03;
	}

	// Check the CRC
	if(size >= 5)
	{
		uint16_t crc = crc_16(modbus_rx_buffer, size - 2);
		if((low_byte(crc) != get_rx_buffer(size - 2)) || (high_byte(crc) != get_rx_buffer(size - 1)))
		{
			return handle_modbus_error(MB_INVALID_CRC);
		}
	}
	return MB_SUCCESS;
}

uint8_t response_received()
{
	if(response_rx)
	{
		response_rx = 0;
		return 1;
	}
	return 0;
}

void set_response_interval(uint32_t delay)
{
	response_interval = delay;
}

uint32_t get_response_interval()
{
	return response_interval;
}

void store_rx_buffer()
{
	// Store the messages data in the response_buffer
	for(uint8_t i = 0; i < get_rx_buffer(2); i++)
	{
		if(i < RX_BUFFER_SIZE)
		{
			response_buffer[i] = (get_rx_buffer(2 * i + 3) << 8) | get_rx_buffer(2 * i + 4);
		}
	}
}
#endif // MB_MASTER

// Modbus Slave Functions ---------------------------------------------------------------------

#ifdef MB_SLAVE
uint8_t modbus_rx()
{
	if(uart_rx_int)
	{
		uart_rx_int = 0;
		return 1;
	}
	return 0;
}

int8_t return_holding_registers(uint8_t* tx_len)
{
	(*tx_len) = 0;
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
	(*tx_len) = 3;

	// Append the Register Values
	for(uint8_t i = 0; i < num_registers; i++)
	{
		modbus_tx_buffer[(*tx_len)++] = high_byte(holding_register_database[first_register_address + i]);
		modbus_tx_buffer[(*tx_len)++] = low_byte(holding_register_database[first_register_address + i]);
	}

	return modbus_send((*tx_len));
}

int8_t edit_multiple_registers(uint8_t *tx_len)
{
	(*tx_len) = 0;

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

	// Protect Read only values
	if(((first_register_address >= GPIO_READ) && (first_register_address <= GPIO_READ)) ||
		 ((last_register_address >= GPIO_READ) && (last_register_address <= GPIO_READ)) ||
		 ((first_register_address < GPIO_READ  && last_register_address > GPIO_READ)))
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
	(*tx_len) = 6;

	for(uint8_t i = 0; i < num_registers; i++)
	{
		holding_register_database[first_register_address + i] = (get_rx_buffer(2 * i + 7) << 8) | get_rx_buffer(2 * i + 8);

		// Handle the range boundaries of each writable register
		handle_range(first_register_address + i);
	}

	// TIMING WORKAROUND START
//	HAL_Delay(1);
	// TIMING WORKAROUND END

	int8_t status = modbus_send((*tx_len));

	if(status == MB_SUCCESS)
	{
		// Special Case Modbus Baud Rate Modification
		if((first_register_address <= 1) && last_register_address >= 1)
		{
			return modbus_change_baud_rate();
		}
	}
	return status;
}

int8_t modbus_exception(int8_t exception_code)
{
	modbus_tx_buffer[0] = get_rx_buffer(0);
	modbus_tx_buffer[1] = get_rx_buffer(1) | 0x80;
	modbus_tx_buffer[2] = exception_code - 3; // Subtract 3 to match the modbus defined error code value

	return modbus_send(3);
}

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
			if(holding_register_database[holding_register] < BAUD_RATE_4800)
			{
				holding_register_database[holding_register] = BAUD_RATE_4800;
			}
			else if(holding_register_database[holding_register] > BAUD_RATE_256000)
			{
				holding_register_database[holding_register] = BAUD_RATE_256000;
			}
			break;
		}
		case MB_TRANSMIT_TIMEOUT:
		{
			if(holding_register_database[holding_register] < 5)
			{
				holding_register_database[holding_register] = 5;
			}
			else if(holding_register_database[holding_register] > 1000)
			{
				holding_register_database[holding_register] = 1000;
			}
			break;
		}
		case MB_TRANSMIT_RETRIES:
		{
			if(holding_register_database[holding_register] > 5)
			{
				holding_register_database[holding_register] = 5;
			}
			break;
		}
		case MB_ERRORS:
		{
			if(holding_register_database[holding_register] > 0x3FF)
			{
				holding_register_database[holding_register] = 0x3FF;
			}
			break;
		}
	}
}
#endif // MB_SLAVE

// General Modbus Functions -------------------------------------------------------------------

int8_t modbus_send(uint8_t size)
{
	int8_t status = HAL_OK;
	// Append CRC (low byte then high byte)
	uint16_t crc = crc_16(modbus_tx_buffer, size);
	modbus_tx_buffer[size] = low_byte(crc);
	modbus_tx_buffer[size + 1] = high_byte(crc);

	uart_tx_int = 0; // This will enable tx timeout monitoring
	tx_time = HAL_GetTick();
	status = HAL_UART_Transmit_DMA(&huart1, modbus_tx_buffer, size + 2);
	__HAL_DMA_DISABLE_IT(huart1.hdmatx, DMA_IT_HT);
	return status;
}

int8_t modbus_reset()
{
	int8_t status = 0;
	// Reset interrupt variables to default state
	uart_tx_int = 1;
	uart_rx_int = 0;
	status = HAL_UART_Abort(&huart1);
	status |= HAL_UART_DeInit(&huart1);
	__USART1_FORCE_RESET();
	HAL_Delay(100);
	__USART1_RELEASE_RESET();
	status = HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0);
	status |= HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
	status |= HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
	status |= HAL_UARTEx_DisableFifoMode(&huart1);
	status |= modbus_set_rx();
	if(status != HAL_OK)
	{
		return handle_modbus_error(MB_FATAL_ERROR);
	}
	return status;
}

int8_t modbus_set_rx()
{
	int8_t status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus_rx_buffer, 6);
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

	return status;
}

int8_t monitor_modbus()
{
	int8_t status = MB_SUCCESS;

	// Chunk miss handling
	status = handle_chunk_miss();
	if(status != MB_SUCCESS)
	{
		status = modbus_reset();
		if(status != MB_SUCCESS)
		{
			return status;
		}
		return handle_modbus_error(MB_UART_ERROR);
	}

	// Uart error handling
	if(uart_err_int)
	{
		uart_err_int = 0;
		status = modbus_reset();
		if(status != MB_SUCCESS)
		{
			return status;
		}
		return handle_modbus_error(MB_UART_ERROR);
	}

	// TX timeout handling
	if(!uart_tx_int)
	{
		if(HAL_GetTick() - tx_time >= holding_register_database[MB_TRANSMIT_TIMEOUT])
		{
			uart_tx_int = 1;
			return handle_modbus_error(MB_TX_TIMEOUT);
		}
		status = HAL_BUSY;
	}

#ifdef MB_MASTER
	// RX timeout handling
	if(expected_rx_len > 0)
	{
		if(uart_rx_int)
		{
			uart_rx_int = 0;
			status = modbus_mic(target_id, target_function_code, expected_rx_len);
			target_id = 0;
			target_function_code = 0;
			expected_rx_len = 0;
			if(status == MB_SUCCESS)
			{
				response_rx = 1;
				if(target_function_code == 0x03)
				{
					store_rx_buffer();
				}
				return status;
			}
			return handle_modbus_error(MB_UART_ERROR);
		}
		else
		{
			if(HAL_GetTick() - rx_time >= response_interval)
			{
				target_id = 0;
				target_function_code = 0;
				expected_rx_len = 0;
				return handle_modbus_error(MB_RX_TIMEOUT);
			}
			status = HAL_BUSY;
		}
	}
#endif
	return status;
}

// General Modbus Control Functions ------------------------------------------------------------

int8_t modbus_startup()
{
	int8_t status = HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0);
	if(status != HAL_OK)
	{
		return status;
	}
	status = HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
	if(status != HAL_OK)
	{
		return status;
	}
	status = HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
	if(status != HAL_OK)
	{
		return status;
	}
	status = HAL_UARTEx_DisableFifoMode(&huart1);
	return status;
}

int8_t modbus_shutdown()
{
	int8_t status = HAL_UART_AbortReceive(&huart1);
	if(status != HAL_OK)
	{
		return status;
	}
	status = HAL_UART_DeInit(&huart1);

	return status;
}

int8_t modbus_change_baud_rate()
{
	int8_t status = 0;

	switch(holding_register_database[MB_BAUD_RATE])
	{
		case BAUD_RATE_2400:
		{
			huart1.Init.BaudRate = 2400;
			break;
		}
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
			holding_register_database[MB_BAUD_RATE] = BAUD_RATE_9600;
			huart1.Init.BaudRate = 9600;
			status = UART_SetConfig(&huart1);
			if(status == HAL_OK)
			{
				status = modbus_reset();
				if(status != HAL_OK)
				{
					return status;
				}
			}
			return handle_modbus_error(RANGE_ERROR);
		}
	}
	status = UART_SetConfig(&huart1);
	if(status == HAL_OK)
	{
		// Log error, reset UART
		status = modbus_reset();
		if(status != HAL_OK)
		{
			return status;
		}
	}

	return modbus_set_rx();
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


// Low Level Functions -------------------------------------------------------------------------
uint8_t get_rx_buffer(uint8_t index)
{
	if (index < MODBUS_RX_BUFFER_SIZE)
	{
		return modbus_rx_buffer[index];
	}
	return 0xFF;
}

int8_t handle_modbus_error(int8_t error_code)
{
	holding_register_database[MB_ERRORS] |= 1U << (error_code - RANGE_ERROR);
	return error_code;
}

// Private Functions ---------------------------------------------------------------------------

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

int8_t handle_chunk_miss()
{
	if(modbus_header == 0)
	{
		if(HAL_GetTick() - chunk_time > 10)
		{
			modbus_header = 1;
			int8_t status = HAL_UART_Abort(&huart1);
			if(status == HAL_OK)
			{
				status = modbus_set_rx();
			}
			return status;
		}
	}
	return MB_SUCCESS;
}

