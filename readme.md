# STM32F412-GUI-Trial

## Overview

This project is a graphical user interface (GUI) developed for the STM32F412 microcontroller using TouchGFX and STM32CubeIDE. The GUI presents four different screens that demonstrate key functionalities including external image display, ADC measurements, and PWM signal control.

The system runs on FreeRTOS, with the main tasks being:

- `defaultTask`: Handles background operations.
- `TouchGFXTask`: Manages GUI updates and interactions.
- `AdcAvgTask`: Averages periodically the measured data by the ADC trough a shadow buffer. 
## Features

- **Screen 1 - Welcome Screen**: The user is greeted with a welcome screen and can click a button to proceed.
- **Screen 2 - Image Display**: Demonstrates loading an external image ("Bubu and Dudu" as reference).
- **Screen 3 - ADC Measurement Plot**:
  - The ADC (`hadc1`) samples 128 values (`ADC_SAMPLES_NUM = 128`) via DMA.
  - The measured samples are displayed on a real-time plot.
  - ADC input channel is configured on Pin A0 (PA1).
  - Displays ADC AVG (Averaged of 128 samples Value) in Volts.
- **Screen 4 - PWM Duty Cycle Control**:
  - The user can adjust a slider to set the PWM duty cycle.
  - The PWM signal is generated using `htim5, TIM_CHANNEL_1`.
  - Output is on Pin D5 (PF5).
  - The last set duty cycle is remembered.
- **UART Debugging**:
  - `printf` is rerouted to UART2 (PA2 TX, PA3 RX) for debugging.
  - Debug messages can be observed in a terminal.

### Example Debug Output:

```
RAW ADC Value 2059 <CR>
RAW ADC Value 2056 <CR>
RAW ADC Value 2058 <CR>
RAW ADC Value 2058 <CR>
RAW ADC Value 2055 <CR>
AVG value 1.670142 <CR>
```

## Code Structure

- **GUI Logic**: Located in the `User/gui` folder.
- **Screen Logic**: Implemented in `ScreenXView.cpp` and `ScreenXPresenter.cpp` files, which handle the corresponding screen functionality.

## Requirements

- STM32F412 microcontroller
- STM32CubeIDE
- TouchGFX Designer
- FreeRTOS
- Serial terminal for debugging (UART2)

## Getting Started

1. Build and flash the firmware using STM32CubeIDE.
2. Connect a serial terminal to observe debug messages via UART2.
3. Interact with the GUI through the touchscreen.

## License

This project is provided as-is for demonstration and learning purposes.

---

For any further modifications or enhancements, feel free to contribute!



# STM32F412G_DISCO TBS

The default IDE is set to STM32CubeIDE, to change IDE open the STM32F412G_DISCO.ioc with CubeMX and select from the supported IDEs (EWARM from version 8.50.9, MDK-ARM, and STM32CubeIDE). Supports flashing of the STM32F412G_DISCO board directly from TouchGFX Designer using GCC and STM32CubeProgrammer.Flashing the board requires STM32CubeProgrammer which can be downloaded from the ST webpage. 

This TBS is configured for 240 x 240 pixels 16bpp screen resolution.  

Performance testing can be done using the GPIO pins designated with the following signals: VSYNC_FREQ  - Pin PE0, RENDER_TIME - Pin PE1, FRAME_RATE  - Pin PE3, MCU_ACTIVE  - Pin PE4
 