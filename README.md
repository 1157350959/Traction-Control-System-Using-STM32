# Traction-Control-System-Using-STM32
PID Control of 6V DC Motor

## Hardware
Microcontrollers have become ubiquitous in the world of modern electronics. Typically designed to accomplish a dedicated function with an embedded system, microcontrollers are found in a wide variety of technologies. This repository presents the design and implementation of a basic traction control system using a Cortex-M4 microcontroller (MCU), a 32-bit Arm-based processor docked in a Nucleo-64 development board (STM32F446RETx). The motor utilized in this system was a fairly simplistic brushed DC motor (3777 "TT  Motor"). The system is equipped with a Hitachi liquid-crystal display (LDC) operated in 4-bit mode, a transmissive photointerrupter to detect the rotational speed of a wheel (GP1A57HRJ00F photodiode pair), and a motor driver (L293X Quadruple Half-H). 

## Software
C programming is used to accomplish speed control of a brushed DC motor using closed-loop control feedback based on proportional-integral-derivative (PID) control theory and pulse-width modulation (PWM). Utilizing the STM32’s nested vector interrupt controller (NVIC), low-latency exception and (external) interrupt handling was shown to be a feasible and tractable solution to processing mode switching.

## Future Work
While dynamic control was achieved in the context of “mode switching”, further improvements to the posited design could be made in terms of a more realistic implementation of cruise control; future work includes noise reduction, utilization of prioritized interrupts, and more realistic physical prototyping.  

## References
[1] Mazidi, Muhammad Ali, et. al., ARM Assembly Language Programming & Architecture. MicroDigitalEd, 2016.

[2] Mazidi, Muhammad Ali, et. al., STM32 ARM Programming for Embedded Systems Using C Language with STM32 Nucleo. MicroDigitalEd, 2016.

[3] RM0390 Reference Manual: STM32F446xx advanced Arm-based 32-bit MCUs, 6th ed. STMicroelectronics., Geneva, Switzerland, 2021.
