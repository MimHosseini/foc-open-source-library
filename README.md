# foc-open-source-library

 * Driver Name: [[ FOC Basic Functions ]]
 * Created on: Jun, 2024
 * Author:     Mohammad Hosseini
 * For More Information, Tutorials, etc.
 * Visit Website: www.esfdrive.ir
 * -------------------------------------------
 *  NOTE1: This library is just a starting point for implementing the FOC method and may need some modifications and improvements
 *  NOTE2: You MUST consider using look-up tables and fixed-point arithmetic when using low stream microcontrollers
 *  this library would probably work on a MCU supporting floating-point unit and ARM CMSIS DSP library without any major problem.
 *  NOTE3: The sliding mode obserever has been implemented based on:
 *  1. Contributions to Discrete-Time Sliding Mode Observers for Permanent Magnet Synchronous Motor Drive Systems( refer to IEEE)
    2. A High-Speed Sliding-Mode Observer for the Sensorless Speed Control of a PMSM( refer to IEEE)
  * -------------------------------------------
 * Clarke & Park transforms have been tested using the SWO feature of the STM32 microcontrollers
 * You could find out how to use this library by referring to the HAL_TIM_PeriodElapsedCallback ISR
