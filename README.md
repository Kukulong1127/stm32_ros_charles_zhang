# Implementing Rosserial with STM32

This README provides instructions on how to implement `rosserial` for communication between a ROS (Robot Operating System) environment and an STM32 microcontroller. `rosserial` allows ROS nodes running on a computer to communicate with microcontrollers. 

The node can be published by the microcontroller to send useful information (for example, the encoder readings, or vehicle status); The microcontroller is also able to subscribe the node to receive command from the ROS.

## Prerequisites

Before you begin, ensure you have the following prerequisites:

- ROS (Noetic) installed on your computer.
- Windows & Ubuntu 20.04 (for ROS Noetic).
- STM32 micro-controller and its development environment (e.g., STM32CubeIDE is used in this example).
- STM32 board with available UART/USART ports (e.g., the example used STM32F103RB board).
- Basic knowledge of ROS and STM32 programming.

## Useful Links

- Rosserial Wiki: https://wiki.ros.org/rosserial
- Rosserial & STM32 packages: https://github.com/yoneken/rosserial_stm32
- ROS Publishers & Subscribers: https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
- Oracle VM Virtual Box: https://www.virtualbox.org/
- STM32CubeIDE: https://www.st.com/en/development-tools/stm32cubeide.html

## Rosserial Installation (Ubuntu)

This part refers to the YouTube tutorial done by 'Learn embedded systems with George' (Video Name: Interfacing STM32 boards to ROS using rosserial protocol | ROS | Learn with George), which is shown in the link below:

<a href="https://youtu.be/cq0HmKrIOt8?si=_NJrYihaPaCEl_g4" target="_blank">
    <img src="http://img.youtube.com/vi/cq0HmKrIOt8/0.jpg" 
    alt="YouTube Video" width="480" height="360" border="10" />
</a>

The video provides useful instructions to setup the rosserial on Ubuntu virtual machine, which runs on the Oracle VM Virtual Box. It is strongly recommended that to create a share folder between the Windows and the Ubuntu virtual machine, so the installed rosserial STM32 library can be accessed directly on Windows.

The following sections are summarized based on the instructions in the video.



Using the following lines to install the correct version of rosserial library for the ROS Noetic:
   
```bash
$ clear
$ sudo apt install ros-noetic-rosserial
```
    
Run the ROS using the 'roscore':

```bash
$ roscore
```

Open a new terminal, use the following lines to check the existing nodes:

```bash
$ rostopic list
$ rostopic info /rosout
$ rosnode list
```

## STM32 Code & Configuration (Windows)
After installing all the required dependencies on Ubuntu and save them into the shared folder, the following process is to configure the STM32 board's pin layout, as well as the rosserial header files.

1. **Pin Configuration (UART):**

    The communication protocol used between the STM32 micro-controller and the ROS is the UART (Universal Asynchronous Receiver/Transmitter), the features of this protocol is shown below:
    
    - __Two wires communication__: Only two wires are connected between the master and slave machine, which are the `Receiver (RX)` and `Transmitter (TX)`;

    - __Fixed Baud Rate__: there is no clock signal is required for asynchronous communication, which means certain baud rate should be chosen before starting the communication (e.g., 9600/57600/115200 etc.). The baud rate should be set as the same on both sides (ROS and STM32);

    - __Detailed Explanation__: More information of UART can be found here (from AnalogueDialogue): https://www.analog.com/en/analog-dialogue/articles/uart-a-hardware-communication-protocol.html

        ![UART_example](https://www.analog.com/-/media/images/analog-dialogue/en/volume-54/number-4/articles/uart-a-hardware-communication-protocol/335962-fig-02.svg?w=900&imgver=1)

    Choose the correct board you are using in the STM32CubeIDE board selection page. Then in the 'Pinout & Configuration' panel, the UART pins are selected as 'PA2' and 'PA3' (for 'USART_TX' and 'USART_RX') of the STM32F103RB board.

    ![pinout_1](images/stm_pinout_1.png)

    There are several settings to be done:
        
    - Set the baud rate to a designated one, in the example it is set to be 57600;
    - Set the 'Mode' to 'Asynchronous', keep 'Hardware Flow Control' disabled;
    - Enable the USART2 global interrupter under the 'NVIC Settings';
    - Add DMA requests under the 'DMA Settings' for 'USART2_TX' and 'USART2_RX' and set the priority to 'High'.

    All of these changes can be viewed in the '.ioc' files inside the repository.

    After completing all the pin configurations, click the 'Device Configuration Tool Code Generation' button to generate the code.

2. **Copy ROS files:**

    It is important to import all the necessary header files and ROS folders into the STM32 project. The following structure is recommended to organize all the files:

    ```
    STM32_project/
    │
    ├──Includes(auto-generated)
    |
    ├── Core/
    │ ├── Inc/
    | |  ├──all_ros_packages
    | |  ├──main.h (auto-generated)
    | |  ├──mainpp.h
    | |  ├──ros.h
    | |  ├──stm32f1xx_hal_conf.h (auto-generated)
    | |  ├──stm32f1xx_it.h (auto-generated)
    | |  └──STM32Hardware.h
    | |
    │ ├── Src/
    | |  ├──duration.cpp
    | |  ├──main.cpp(renamed from main.c)
    | |  ├──mainpp.cpp
    | |  ├──stm32f1xx_hal_msp.c
    | |  ├──stm32f1xx_it.c
    | |  ├──syscalls.c
    | |  ├──sysmem.c
    | |  ├──system_stm32f1xx.c    
    | |  └──time.cpp
    | |
    │ └── Startup(auto-generated)
    │
    ├── Drivers(auto-generated)
    │
    ├── STM32_project.ioc
    └── STM32_project.launch

    ``` 
    Note that when copy the ros packages into the 'STM32_Project/Core/Inc/', do not include the 'main.h','stm32f4xx_hal_it.h', and 'stm32f4xx_hal_conf.h' files because they have been generated automatically when the project is created.
    
    The 'duration.cpp' and 'time.cpp' are moved from the 'STM32_Project/Core/Inc/' to 'STM32_Project/Core/Src/'.

    Later the 'main.c' should be converted to the C++ file to be used in ROS.

    The complete example can be found in either the chatter or the LED blinking repos.


3. **main.c:**
    
    Inside the generated 'main.c' file, it contains all the configurations for the peripherals. Some modifications should be done to the code to include the 'setup()' and 'loop()' functions included in the rosserial packages.

    - 'setup()': 
    - 'loop()': 

