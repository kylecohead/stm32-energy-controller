# Smart Prepaid Energy Controller

### Project Overview
This project was developed as a comprehensive hardware and software prototype for the Design (E) 314 module at Stellenbosch University. The system is a functional prepaid electricity meter designed to manage and monitor power consumption in rental properties, such as student accommodations or Airbnbs. The device measures real-time power metrics, switches electrical loads based on available prepaid units, logs historical data, and features an RFID-based authentication system for administrative override.

### Tech Stack and Tools
* **Languages:** C
* **Microcontroller:** STM32G0B1RE (ARM Cortex-M0+)
* **Development Environment:** STM32CubeIDE
* **Libraries:** STM32 Hardware Abstraction Layer (HAL), FatFs
* **Hardware Concepts:** PCB Prototyping, Signal Conditioning, Component Selection, Circuit Debugging

### Hardware Architecture
I was responsible for the physical design, assembly, and testing of the custom baseboard circuits to interface with the STM32 microcontroller. Key hardware implementations include:
* **Power Regulation:** Built 5V and 3.3V linear regulator circuits to step down the 9-12V power supply.
* **Analog Signal Conditioning:** Designed operational amplifier circuits (using the MCP602) to apply the necessary gain and DC offset to AC voltage and current signals so they could be safely read by the microcontroller's ADC.
* **Load Control:** Implemented a relay circuit driven by an N-Channel MOSFET to safely switch the simulated electrical load.
* **Peripherals:** Integrated a 4x3 matrix keypad, an I2C OLED display, an SPI RC522 RFID reader, and an SPI MicroSD card module.

### Core Software Implementations
While the project utilizes the standard STM32 HAL framework and open-source driver libraries for specific modules (such as the SSD1306 OLED driver), I developed the core application logic and peripheral integration. Key software features include:
* **Direct Memory Access (DMA) Sampling:** Configured the ADC to be triggered by a hardware timer at 5kHz. Used DMA to autonomously transfer the sampled voltage and current data directly to memory, freeing up the CPU for other tasks.
* **Power Metric Algorithms:** Wrote algorithms to process the raw ADC buffers to calculate RMS voltage, RMS current, phase difference (using zero-crossing detection), real power, apparent power, and reactive power.
* **State Machine User Interface:** Designed a Finite State Machine (FSM) to control the OLED display menus, handle keypad inputs, and manage system states.
* **Hardware Interrupts:** Utilized external interrupts for UART communication (running at 57600 baud) to receive and process remote commands without halting the main control loop.
* **Signal Debouncing:** Implemented software debouncing logic utilizing hardware timers to filter out mechanical noise from the keypad and emergency override button.
* **Data Logging:** Integrated the FatFs file system to log timestamped energy consumption statistics to an SD card at regular intervals to ensure data recovery after power loss.

### Testing and Validation
The system was extensively verified using oscilloscopes and digital multimeters. The signal conditioning circuits, power regulation, and software measurement algorithms were calibrated and tested to achieve an accuracy margin of within 5 percent for all voltage, current, and power calculations.
