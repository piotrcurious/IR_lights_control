# ESP32 Advanced LED Controller with IR Remote

This project transforms an ESP32 into a sophisticated 8-channel LED controller, operated entirely by a standard Samsung IR remote. It provides granular control over brightness, PWM resolution, and offers dynamic effects like pulse and fade. A key feature is the ability to chain effects across channels, creating complex lighting sequences. All settings are persistent, automatically saved to and restored from the ESP32's non-volatile memory.

## Key Features

- **8-Channel PWM Control**: Independently manage up to eight LED channels.
- **IR Remote Operation**: Full functionality using a common Samsung IR remote.
- **Adjustable Brightness**: Fine-grained brightness control for each channel.
- **Variable PWM Resolution**: Dynamically adjust the PWM resolution from 1 to 13 bits, allowing a trade-off between smoothness and responsiveness.
- **Dynamic Effects**:
  - **Pulse**: LEDs cycle on and off at an adjustable frequency.
  - **Fade In/Out**: LEDs smoothly transition from off to a set brightness and back.
- **Effect Chaining**: Create intricate sequences by linking one channel's effect to trigger another upon completion.
- **Persistent Settings**: All brightness levels and effect configurations are saved to non-volatile storage.
- **Non-Blocking Architecture**: The firmware is designed to be fully responsive, with all effects and visual feedback implemented without blocking `delay()` calls.

## Hardware Requirements

- **ESP32 Development Board**: Any ESP32-based board is suitable.
- **IR Receiver**: A standard IR receiver module (e.g., TSOP38238).
- **Samsung IR Remote**: The firmware is specifically coded for the Samsung remote protocol.
- **LEDs and Power Supply**: Appropriate LEDs, current-limiting resistors, and a power supply capable of driving them.

### Pinout Configuration

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

## Code Design

The firmware is a single, well-structured Arduino sketch (`IR_demo8.ino`) that leverages the ESP32's powerful hardware features.

- **LEDC Driver**: The ESP32's native LEDC (LED Control) driver provides hardware-accelerated PWM generation, enabling smooth, efficient fades without CPU overhead.
- **IR Remote Handling**: The `IRremote` library decodes signals from the remote. A central `processSamsungCommand` function dispatches commands to the appropriate handlers.
- **Effect Management**: A non-blocking state machine in the `updateEffects` function manages the timing and state transitions for all active lighting effects.
- **Persistence**: The `Preferences` library is used to seamlessly save and load all settings from the ESP32's flash memory.

## Operating Instructions

Point a Samsung remote at the IR receiver and use the following controls:

| Button         | Function                                                                                                 |
|----------------|----------------------------------------------------------------------------------------------------------|
| **Digits 0-7** | Select the active LED channel. The selected channel will flash briefly to confirm.                       |
| **VOL+ / VOL-**| Increase or decrease the brightness of the currently selected channel.                                     |
| **UP / DOWN**  | Increase or decrease the PWM resolution for all channels.                                                |
| **LEFT / RIGHT**| Adjust the frequency of the effect on the selected channel.                                              |
| **RED Key**    | Toggle the **Pulse** effect on or off for the selected channel.                                              |
| **GREEN Key**  | Toggle the **Fade In/Out** effect on or off for the selected channel.                                        |
| **CH LIST**    | Enter **Chaining mode**. The current channel flashes to indicate it is the source. Press a digit (0-7) to select the target channel. |
| **SOURCE**     | Save the current brightness and effect settings for all channels.                                        |
| **MUTE**       | Toggle the brightness of the selected channel between 0% and 50%.                                        |
