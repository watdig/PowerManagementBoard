from pymodbus.client import ModbusSerialClient as ModbusClient
import logging
import time
import sys

# Set up logging
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.ERROR) 
logging.getLogger('pymodbus').setLevel(logging.ERROR)

# DEBUG : Logs everything (most detailed)
# INFO : Logs general information about execution
# WARNING : Logs warnings about potentially problematic situations
# ERROR : Logs exceptions or errors in the program
# CRITICAL: Logs critical issues in the program

# Connection parameters
PORT = 'COM3'
BAUDRATE = 9600
PARITY = 'N'
UNIT_ID = 1
STOPBITS = 1
BYTESIZE = 8
TIMEOUT = 2


def read_register(client, register, num_registers, modbus_id):
    try:
        # Read the register value synchronously
        result = client.read_holding_registers(register, num_registers, modbus_id)
        
        if result.isError():
            print(f"Error message: {result}")
            return result
        
        print("Returned values:", result.registers)
        return result
        
    except Exception as e:
        logger.error(f"Error reading register {register}: {e}")
        return None
        
        
def write_register(client, register, values, modbus_id):
    try:
        # Write the register value synchronously
        result = client.write_registers(register, values, modbus_id, False);
        
        if result.isError():
            print(f"Error message: {result}");
            return result
        
        print(f"Writing register values {values} to address {result.address}")
        return result
        
    except Exception as e:
        logger.error(f"Error writing register {register}: {e}")
        return None
    
def initialize():
    client = ModbusClient(
        port=PORT,
        baudrate=BAUDRATE,
        parity=PARITY,
        stopbits=STOPBITS,
        bytesize=BYTESIZE,
        timeout=TIMEOUT,
    )
    return client   
 
def main():
    client = initialize()
    if client.connect() is True:
        # Read the Modbus ID and use it for future communication
        response = read_register(client, 0, 1, 255)
        UNIT_ID = response.register[0]
        while True:
            print("Enter numbers separated by spaces, enter q to terminate program")
            user_input = input("Format: Read(0)/Write(1) | # Registers | Start Register | Data:")
            if user_input == "q":
                break
            command = list(map(int, user_input.split()))
            if len(command) < 3:
                print("Not Enough inputs")
            elif command[0] > 1:
                print("Read/Write input invalid")
            else:
                print("Input", command)
                if command[0] == 0:
                    response = read_register(client, command[2], data, UNIT_ID)
                else:
                    for i in range(len(command) - 3):
                        data.append(command[i])
                    response = write_register(client, command[2], data, UNIT_ID)
        client.close()

if __name__ == "__main__":
    main()
    
