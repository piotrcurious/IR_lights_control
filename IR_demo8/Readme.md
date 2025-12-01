# ESP32 Advanced LED Controller with IR Remote

This project implements an 8-channel LED controller using an ESP32 microcontroller, controllable via a standard Samsung IR remote. It offers precise brightness control, adjustable PWM resolution, and a variety of dynamic effects like pulse and fade, which can be chained together across different channels. All settings are persistent and automatically saved to and restored from the ESP32's non-volatile storage.

## Features

- **8-Channel PWM Control**: Independently control up to 8 LED channels.
- **IR Remote Operation**: Full control using a common Samsung IR remote.
- **Adjustable Brightness**: Fine-grained brightness adjustment for each channel.
- **Variable PWM Resolution**: Dynamically change the PWM resolution from 1 to 13 bits for a trade-off between smoothness and responsiveness.
- **Dynamic Effects**:
  - **Pulse**: LEDs turn on and off at an adjustable frequency.
  - **Fade In/Out**: LEDs smoothly fade from off to a set brightness and back.
- **Effect Chaining**: Create sequences by linking one channel's effect to trigger another's upon completion.
- **Persistent Settings**: All brightness levels and effect configurations are saved automatically.
- **Non-Blocking Animations**: All effects and visual feedback are implemented without using `delay()`, ensuring the system remains responsive.

## Hardware Requirements

- **ESP32 Development Board**: Any ESP32-based board will work.
- **IR Receiver**: A standard IR receiver module (e.g., TSOP38238).
- **Samsung IR Remote**: The code is specifically designed for the Samsung remote protocol.
- **LEDs and Power Supply**: Appropriate LEDs, resistors, and a power supply capable of driving them.

### Pinout

| Function      | ESP32 GPIO |
|---------------|------------|
| IR Receiver   | 15         |
| LED Channel 0 | 2          |
| LED Channel 1 | 4          |
| LED Channel 2 | 16         |
| LED Channel 3 | 17         |
| LED Channel 4 | 18         |
| LED Channel 5 | 19         |
| LED Channel 6 | 21         |
| LED Channel 7 | 22         |

## Code Design and Flow

The firmware is written in C++ for the Arduino framework and is contained within a single `IR_demo8.ino` file.

### Core Components

1.  **Data Structures**: `Effect`, `FlashState`, and `SaveIndicatorState` structs hold the state for all dynamic operations, ensuring that animations and effects can be managed in a non-blocking manner.
2.  **LEDC Driver**: The ESP32's native LEDC (LED Control) driver is used for hardware-accelerated PWM generation, which allows for smooth fades and efficient control.
3.  **IR Remote Handling**: The `IRremote` library is used to decode signals from the remote. A central `processSamsungCommand` function routes commands to the appropriate handlers.
4.  **Effect Management**: A state machine within the `updateEffects` function manages the timing and state transitions for all active effects.
5.  **Persistence**: The `Preferences` library is used to save and load all settings from the ESP32's flash memory.

### Program Flow

-   **`setup()`**: Initializes serial communication, loads settings from flash, configures the LEDC driver and IR receiver, and starts any effects that were active before the last shutdown.
-   **`loop()`**: The main loop is kept short and responsive. It continuously calls updater functions for visual feedback (`updateFlash`, `updateSaveIndicator`), effect animations (`updateEffects`), and checks for new IR commands.

## Operating Instructions

Point your Samsung remote at the IR receiver and use the following buttons:

| Button         | Function                                                                                                 |
|----------------|----------------------------------------------------------------------------------------------------------|
| **Digits 0-7** | Select the active LED channel. The selected channel will flash briefly to confirm.                       |
| **VOL+ / VOL-**| Increase or decrease the brightness of the currently selected channel.                                     |
| **UP / DOWN**  | Increase or decrease the PWM resolution for all channels.                                                |
| **LEFT / RIGHT**| Adjust the frequency of the effect on the selected channel. This works even if the effect is not active. |
| **RED Key**    | Toggle the Pulse effect on or off for the selected channel.                                              |
| **GREEN Key**  | Toggle the Fade In/Out effect on or off for the selected channel.                                        |
| **CH LIST**    | Enter Chaining mode. The current channel will flash. Press a digit (0-7) to select the target channel.     |
| **SOURCE**     | Save the current brightness and effect settings for all channels to persistent storage.                  |
| **MUTE**       | Toggle the brightness of the selected channel between 0% and 50%.                                        |
