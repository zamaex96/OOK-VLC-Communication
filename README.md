Embedded C code which utilizes bit manipulation and GPIO pins to send and receiver information from transmitter to receiver via VLC communication.

# Summary of Transmitter Code

## Overview
This C code is designed for an AVR microcontroller  to handle serial communication, timer interrupts, and digital I/O operations. It includes functions for initializing serial ports, configuring timers, transmitting and receiving data, and controlling digital outputs on ports C and D.

## Definitions and Includes
- **`#define F_CPU 16000000`**: Defines the CPU frequency for timing calculations.
- **`#define CHECKBIT(x,y) (x & (y))`**: Macro to check a specific bit in a byte.
- **Includes**:
  - `<stdio.h>`: Standard input/output library.
  - `<avr/io.h>`: AVR-specific IO definitions.
  - `<util/delay.h>`: Provides delay functions.
  - `<avr/interrupt.h>`: AVR interrupt handling definitions.

## Global Variables and Data Arrays
- **`unsigned char data[32]`**: Array to store received data bytes.
- **`volatile long cnt`**: Volatile variable for counting timer overflows.

## Function Definitions

### `serial_init(void)`
- Initializes UART communication on UART1 and UART0.
- Sets baud rates, data format (8-bit, no parity, 1 stop bit), and enables transmitter and receiver.

### `Timer1_init(void)`
- Initializes Timer1 for generating interrupts.
- Configures Timer1 in phase and frequency correct PWM mode.
- Sets the timer period based on defined cycles and selects a prescaler (clock divider).

### `ISR(TIMER1_OVF_vect)`
- Interrupt Service Routine for Timer1 overflow interrupt.
- Increments `cnt` variable on each overflow.

### `transmit1(unsigned char data)`
- Transmits a byte of data over UART1.
- Waits until the transmit buffer is empty before sending.

### `transmit0(unsigned char data)`
- Transmits a byte of data over UART0.
- Waits until the transmit buffer is empty before sending.

### `unsigned int Rxdata(void)`
- Waits to receive a byte of data from UART0.
- Returns the received data.

### `send_code_portC(char data)`
- Sends a byte of data over Port C (digital outputs).
- Converts each bit of the data byte into a digital output state (`0xFF` or `0x00`).
- Adds a delay (`_delay_ms(2000)`) after setting each bit.

### `send_code_portD(char data)`
- Sends a byte of data over Port D, Pin 1 (digital output).
- Similar to `send_code_portC` but operates on Port D, Pin 1.
- Adds a delay (`_delay_us(1000000)`) after setting each bit.

### `send_code_portDp2(char data)`
- Sends a byte of data over Port D, Pin 2 (digital output).
- Converts each bit of the data byte into a digital output state (`0x02` or `0x00`).
- Adds a small delay (`_delay_us(50)`) after setting each bit.

### `getSensorData(void)`
- Sends a sequence of commands over UART0 to request sensor data.
- Receives 32 bytes of sensor data into the `data` array.
- Currently commented out in `main()`.

## `main()` Function
- Initializes serial communication and Timer1.
- Configures digital pins for output on Ports A, C, and D.
- Enters an infinite loop to repeatedly:
  - Send test data using `transmit0()` and `transmit1()`.
  - Optionally, call `send_code_portC()`, `send_code_portD()`, or `send_code_portDp2()` to control digital outputs.

## Notes
- The code demonstrates usage of UART for serial communication, timer interrupts for precise timing, and digital I/O operations for controlling outputs.
- Functions such as `send_code_portC()`, `send_code_portD()`, and `send_code_portDp2()` manipulate digital outputs based on the bits of the data byte passed to them.
- Timer1 is configured to generate interrupts on overflow, providing a timing mechanism for other operations.
- The code includes commented-out sections (`getSensorData()`) indicating potential functionality for sensor data acquisition, not currently active in the main loop.

# Summary of Receiver Code

## Overview
This C code is designed for an AVR microcontroller to handle serial communication, timer interrupts, and digital I/O operations. It includes functions for initializing UART communication, configuring timers for interrupt generation, and manipulating digital input/output on Port A.

## Definitions and Includes
- **`#define F_CPU 16000000`**: Defines the CPU frequency for timing calculations.
- **`#define CHECKBIT(x,y) (x & (y))`**: Macro to check a specific bit in a byte.
- **Includes**:
  - `<stdio.h>`: Standard input/output library.
  - `<avr/io.h>`: AVR-specific IO definitions.
  - `<util/delay.h>`: Provides delay functions.
  - `<avr/interrupt.h>`: AVR interrupt handling definitions.

## Global Variables
- **`volatile long cnt`**: Volatile variable for counting timer overflows used in interrupt context.

## Function Definitions

### `serial_init(void)`
- Initializes UART communication on UART1 (`UBRR1`) and UART0 (`UBRR0`).
- Sets baud rates using pre-defined values from the `baudrate` array.
- Configures UART settings for 8-bit data, no parity, and 1 stop bit.

### `transmit(unsigned char data)`
- Transmits a byte of data over UART0.
- Waits until the transmit buffer (`UDRE0`) is empty before sending.

### `unsigned char Rxdata(void)`
- Waits to receive a byte of data from UART1 (`RXC1`).
- Returns the received data from the UART receive buffer (`UDR1`).

### `Timer1_init(void)`
- Initializes Timer1 for generating overflow interrupts.
- Configures Timer1 in phase and frequency correct PWM mode (`WGM13`).
- Sets the timer period (`ICR1`) based on the defined `cycles` variable.
- Configures Timer1 to operate with no prescaling (`CS10`).

### `ISR(TIMER1_OVF_vect)`
- Interrupt Service Routine (ISR) for Timer1 overflow interrupt.
- Increments the `cnt` variable on each overflow.

### `syncHeaderPortC(void)`
- Synchronizes data transmission based on input on Port A (`PINA`).
- Waits until a specific header pattern (`0xFF80`) is detected.
- Returns 1 upon successful header synchronization.

### `shiftByte(void)`
- Reads 8 bits of data from Port A (`PINA`) and returns the byte.
- Uses a loop to shift in each bit of the byte sequentially.
- Returns the assembled byte.

## `main()` Function
- Initializes serial communication and Timer1 using `serial_init()` and `Timer1_init()`.
- Configures Port A (`DDRA`) as input (`0x00`) and enables pull-up resistors (`PORTA=0xFF`).
- Enables Timer1 overflow interrupt (`TIMSK |= 0x04`) and global interrupts (`SREG |= 0x80`).
- Enters an infinite loop to:
  - Call `shiftByte()` to read a byte from Port A and transmit it using `transmit()`.
  - Optionally, call `syncHeaderPortC()` to synchronize data transmission based on a specific header pattern.

## Notes
- The code demonstrates usage of UART for serial communication (`transmit()` and `Rxdata()`), timer interrupts (`Timer1_init()` and `ISR(TIMER1_OVF_vect)`), and digital input/output manipulation (`syncHeaderPortC()` and `shiftByte()`).
- Timer1 is configured to generate interrupts to count timer overflows (`cnt` variable).
- `main()` continuously reads data from Port A and sends it over UART0 after assembling it using `shiftByte()`.
- The code includes commented-out sections (`syncHeaderPortC()` and `transmit()` calls) indicating potential functionality not currently active in the main loop.



