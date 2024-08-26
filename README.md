# STM32F491RE CAN FD Simulator
# Project Overview
This project showcases the STM32F491RE microcontroller's capabilities for handling CAN FD (Controller Area Network with Flexible Data-Rate) communication. It provides a practical example of both transmitting and receiving CAN FD messages, supporting both Standard and Extended Frame formats. Additionally, the project demonstrates how to visualize CAN FD data over UART (Universal Asynchronous Receiver-Transmitter) for monitoring and debugging purposes.

### Features
#### 1.CAN FD Communication
i.Transmission and reception of both Standard and Extended Frames
ii.Flexible Data-Rate support for higher data throughput
#### 2.UART Communication

i. Real-time visualization of CAN FD data
ii. Data logging and debugging through serial communication
### Hardware Requirements
Microcontroller: STM32F491RE
CAN Transceiver: TJA1050T (or compatible)
UART-to-USB Converter: FTDI FT232R or similar
Power Supply: 3.3V or 5V, depending on your setup

### Software Requirements
STM32CubeIDE or Keil MDK: For development and debugging
STM32CubeMX: For peripheral configuration
CAN FD Tool: For interacting with CAN networks (e.g., Vector CANoe, PCAN-View)

### Getting Started
1. Clone the Repository
Start by cloning the repository to your local machine:
git clone https://github.com/MohitGupta2021/CAN-FD-Simulator.git
cd CAN-FD-Simulator
2. Open and Configure the Project
Open in STM32CubeIDE
Open the project in STM32CubeIDE or Keil MDK.

3. Configure Peripherals
Use STM32CubeMX to configure CAN and UART peripherals as described in the *.ioc file included in this repository.
Ensure CAN FD mode is enabled and configured correctly for both Standard and Extended Frame formats.
Configure UART settings according to your requirements.

4. Build and Flash the Project
Build the Project
Build the project in STM32CubeIDE or Keil MDK to generate the binary file.
Flash the Microcontroller
Connect your STM32F491RE to your computer using a debugger (e.g., ST-Link) and flash the compiled binary file to the microcontroller.

5. Connect and Test
### 1.Hardware Connections

Connect the CAN transceiver to the CAN pins of the STM32F491RE.
Connect the UART TX and RX pins to the UART-to-USB converter.
Connect the UART-to-USB converter to your computer.
### 2. Monitor CAN Communication

Use a CAN FD tool to monitor the CAN bus. Configure the tool to listen to the CAN network where the STM32F491RE is connected.

### 3. View UART Data

Open a serial terminal program (e.g., PuTTY, Tera Term) on your computer to view UART data. Ensure the terminal settings match the UART configuration used in the STM32 project.

# Result Project Images

- <img src="https://github.com/MohitGupta2021/CAN-FD-Simulator/blob/main/images/CANFD_Simulator.png" alt="Image 1" width="400"/>
# License
This project is licensed under the MIT License. See the LICENSE file for details.

# Acknowledgments
STMicroelectronics: For providing the STM32F491RE microcontroller.
Community Contributions: Thanks to contributors and community members for their feedback and support.
Feel free to adjust the content according to your specific project details and requirements. This structured approach should help users understand and utilize the STM32F491RE CAN FD and UART data flow functionalities effectively.






