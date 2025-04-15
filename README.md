# Power Management Board
This Firmware allows a host computer to communicate with a custom "PowerManagementBoard" PCB designed for the WatDig design team at the University of Waterloo. The system controls and relays sensor data about the state of the 480VAC and 120VAC power supplied to a Tunnel Boring Machine (TBM). A flow chart depicting the general design of the system can be found at the following link... https://lucid.app/lucidchart/40cd09a3-0b17-4176-88fb-b93ab9d76a61/edit?viewport_loc=-2870%2C-2245%2C5084%2C2400%2C0_0&invitationId=inv_9890a6aa-6289-44ab-988a-534f96138113

### System Overview
This system consists of 2 writable GPIO pins and 2 readable GPIO pins on a STM32C071CBT6 microcontroller. The 2 writeable GPIO pins turn on 480VAC and 120VAC power for the TBM. A watchdog timer has been implemented within the system, meaning that the user must issue a Modbus command within a user defined timeout period between 10 to 1000 milliseconds. The only requirement of the modbus command issued to the power management board is that the command must contain the correct modbus identification of the power management board. All data including this timeout period is contained within a "register_database" in the STM32 microcontroller, which is essentially just a global array that the host computer can read and write to via the Modbus protocol. The only modbus functions supported in this system are reading multiple holding registers (function code 0x03) or writing multiple holding registers (function code 0x10). Issuing invalid Modbus commands such as writing to a read-only register or exceeding the acceptable value range of a register will return an exception code in accordance with the Modbus protocol. The following link outlines the registers which the user has access to in the system...
https://docs.google.com/spreadsheets/d/11n6w8ZuzljPktblNUjErZGDjPZ7gsNEzAxKXmISzQjk/edit?usp=sharing

![image](https://github.com/user-attachments/assets/e1051145-d239-46af-90b0-d7a1edf3a766)

![image](https://github.com/user-attachments/assets/bada116d-8035-4f9f-b0c1-bb49944c0cb0)
