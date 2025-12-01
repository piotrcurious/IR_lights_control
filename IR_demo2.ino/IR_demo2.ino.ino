// ESP32 LEDC PWM + IR remote (driver/ledc API — fixed initializer order)
// IR receiver on GPIO15
// Only accepts keys from SAMSUNG protocol with address 0x7
// Numeric keys 0-9 select PWM pin; VOL+/VOL- adjust brightness.
// KEY_UP/KEY_DOWN change LEDC resolution (rescale brightness automatically).

#include <Arduino.h>
#include <IRremote.h>
#include <Preferences.h>

#if defined(ARDUINO_ARCH_ESP32)
extern "C" {
  #include "driver/ledc.h"
}
#endif

enum EffectType {
  EFFECT_NONE,
  EFFECT_PULSE,
  EFFECT_FADE_IN_OUT
};

struct Effect {
  EffectType type = EFFECT_NONE;
  float frequency = 1.0; // Hz
  float dutyCycle = 50.0; // Percent
  uint32_t duration = 2000; // ms for a full fade in/out cycle
  uint32_t startTime = 0;
  bool active = false;
  uint32_t savedBrightness = 0;
  bool oneShot = false;
  uint8_t nextChannel = 0xFF; // 0xFF means no chaining
  EffectType nextEffectType = EFFECT_PULSE;
};

enum SystemState {
  STATE_NORMAL,
  STATE_AWAITING_CHAIN_CHANNEL
};

SystemState systemState = STATE_NORMAL;
uint8_t chaining_originalChannel = 0;
EffectType chaining_effectType = EFFECT_PULSE;
EffectType chaining_nextEffectType = EFFECT_PULSE;

namespace RemoteKeys {
  enum KeyCode {
    KEY_POWER = 0xE6,
    KEY_0 = 0x11, KEY_1 = 0x04, KEY_2 = 0x05, KEY_3 = 0x06, KEY_4 = 0x08,
    KEY_5 = 0x09, KEY_6 = 0x0A, KEY_7 = 0x0C, KEY_8 = 0x0D, KEY_9 = 0x0E,
    KEY_UP = 0x60, KEY_DOWN = 0x61, KEY_LEFT = 0x65, KEY_RIGHT = 0x62,
    KEY_OK = 0x68, KEY_MENU = 0x79, KEY_RED = 0x6c, KEY_GREEN = 0x14,
    KEY_YELLOW = 0x15, KEY_BLUE = 0x16, KEY_VOL_UP = 0x07, KEY_VOL_DOWN = 0x0b,
    KEY_CH_UP = 0x12, KEY_CH_DOWN = 0x10, KEY_REWIND = 0x45, KEY_PLAY = 0x47,
    KEY_PAUSE = 0x4A, KEY_FORWARD = 0x48, KEY_STOP = 0x46, KEY_SETTINGS = 0x1A,
    KEY_INFO = 0x1F, KEY_SUBTITLES = 0x25, KEY_MUTE = 0x0F, KEY_NETFLIX = 0xF3,
    KEY_PRIME_VIDEO = 0xF4, KEY_GUIDE = 0x4F, KEY_SOURCE = 0x01
  };
}

static const uint8_t IR_PIN = 15;
static const uint8_t LIGHT_SENSOR_PIN = 34;
static const uint8_t MOTION_SENSOR_PIN = 27;

float globalDimming = 1.0; // 1.0 = no dimming, 0.0 = full dimming
bool motionDetected = false;
uint32_t lastMotionTime = 0;
const uint32_t motionOverrideDuration = 10000; // 10 seconds

// Map digits 0..9 to safe GPIOs
const uint8_t pwmPins[10] = {2, 4, 16, 17, 18, 19, 21, 22, 23, 25};

const int LEDC_CHANNEL_COUNT = 10;// number of channels used
const int LEDC_FREQ = 5000; // PWM frequency (Hz)

// initial resolution (bits)
uint8_t currentResolution = 8;
const uint8_t MIN_RESOLUTION = 1;
const uint8_t MAX_RESOLUTION = 13;

// brightness stored in integer range [0 .. (2^currentResolution - 1)]
uint32_t brightness[LEDC_CHANNEL_COUNT];
Effect effects[LEDC_CHANNEL_COUNT];
int selectedIndex = 0;

// choose speed mode and timer to use
const ledc_mode_t LEDC_MODE = LEDC_HIGH_SPEED_MODE;
const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;

// Non-blocking flash state (updated for alternating flash)
struct FlashState {
  bool active = false;
  uint32_t savedValue = 0;
  uint8_t channel = 0xFF;

  // New fields for alternating flash pattern: 3 flashes (6 state changes) at 80ms interval
  uint8_t blinksRemaining = 0; // The total number of state changes (ON->OFF or OFF->ON) left.
  uint32_t nextToggleMs = 0;
  bool isBright = false; // Current state: true for full brightness, false for zero.
} flashState;

// helper: max value for resolution bits
static inline uint32_t maxForRes(uint8_t res) {
  if (res >= 32) return UINT32_MAX;
  return ((uint32_t)1 << res) - 1;
}

// scale value from oldRes -> newRes (rounded)
static inline uint32_t scaleValue(uint32_t value, uint8_t oldRes, uint8_t newRes) {
  uint32_t oldMax = maxForRes(oldRes);
  uint32_t newMax = maxForRes(newRes);
  if (oldMax == 0) return 0;
  uint64_t tmp = (uint64_t)value * (uint64_t)newMax;
  tmp = (tmp + oldMax/2) / oldMax; // rounding
  if (tmp > newMax) tmp = newMax;
  return (uint32_t)tmp;
}

// wrapper to write duty using driver API
static inline void ledcWriteDuty(uint8_t channel, uint32_t duty) {
  if (channel >= LEDC_CHANNEL_COUNT) return;
  uint32_t maxV = maxForRes(currentResolution);
  if (duty > maxV) duty = maxV;

  // Apply global dimming
  uint32_t dimmedDuty = duty * globalDimming;
  if (dimmedDuty > maxV) dimmedDuty = maxV;

  ledc_set_duty(LEDC_MODE, (ledc_channel_t)channel, dimmedDuty);
  ledc_update_duty(LEDC_MODE, (ledc_channel_t)channel);
}

void reconfigureLEDCTimer(uint8_t newRes) {
  if (newRes < MIN_RESOLUTION) newRes = MIN_RESOLUTION;
  if (newRes > MAX_RESOLUTION) newRes = MAX_RESOLUTION;
  if (newRes == currentResolution) return;

  uint8_t oldRes = currentResolution;
  uint32_t oldMax = maxForRes(oldRes);
  uint32_t newMax = maxForRes(newRes);

  Serial.printf("Reconfiguring LEDC resolution %u -> %u (oldMax=%u newMax=%u)\n",
                 oldRes, newRes, (unsigned)oldMax, (unsigned)newMax);

  // scale brightness and flash savedValue
  for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
    brightness[ch] = scaleValue(brightness[ch], oldRes, newRes);
  }
  if (flashState.active && flashState.channel < LEDC_CHANNEL_COUNT) {
    flashState.savedValue = scaleValue(flashState.savedValue, oldRes, newRes);
  }

  // reconfigure LEDC timer with new resolution (explicit assignments)
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.speed_mode = LEDC_MODE;
  ledc_timer.duty_resolution = (ledc_timer_bit_t)newRes;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.freq_hz = LEDC_FREQ;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  // reconfigure channels to use same timer and write new duties
  for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
    ledc_channel_config_t ch_conf;
    memset(&ch_conf, 0, sizeof(ch_conf));
    ch_conf.channel = (ledc_channel_t)ch;
    ch_conf.duty = 0;
    ch_conf.gpio_num = pwmPins[ch];
    ch_conf.speed_mode = LEDC_MODE;
    ch_conf.hpoint = 0;
    ch_conf.timer_sel = LEDC_TIMER;
    ledc_channel_config(&ch_conf);

    // If flash active on this ch, update duty based on its current flash state
    if (flashState.active && flashState.channel == ch) {
      if (flashState.isBright) {
        ledcWriteDuty(ch, maxForRes(newRes)); // Full brightness if currently ON
      } else {
        ledcWriteDuty(ch, 0); // Zero brightness if currently OFF
      }
    } else {
      // Not active flash, set stored brightness
      ledcWriteDuty(ch, brightness[ch]);
    }
  }

  currentResolution = newRes;
}

// Starts the non-blocking alternating flash sequence (3 cycles: ON/OFF/ON/OFF/ON/OFF)
void startFlash(uint8_t channel) {
  if (channel >= LEDC_CHANNEL_COUNT) return;

  // 1. Restore any currently active flash
  if (flashState.active && flashState.channel < LEDC_CHANNEL_COUNT) {
    ledcWriteDuty(flashState.channel, flashState.savedValue);
  }

  // 2. Initialize new flash sequence parameters
  const uint8_t totalCycles = 3;
  const uint32_t toggleInterval = 80; // ms per ON or OFF state

  flashState.channel = channel;
  flashState.savedValue = brightness[channel]; // Save original brightness
  flashState.active = true;
  flashState.blinksRemaining = totalCycles * 2; // e.g., 6 total toggles for 3 flashes

  // 3. Set the first state (Bright) and schedule the first toggle
  flashState.isBright = true;
  ledcWriteDuty(channel, maxForRes(currentResolution)); // First pulse: ON (Full Brightness)
  flashState.nextToggleMs = millis() + toggleInterval;
}

// Handles the non-blocking blinking logic
void updateFlash() {
  if (!flashState.active) return;

  // Check if it's time to toggle the LED state
  if ((int32_t)(millis() - flashState.nextToggleMs) >= 0) {
    uint8_t ch = flashState.channel;
    const uint32_t toggleInterval = 80; // Must match startFlash

    if (flashState.blinksRemaining > 0) {
      // Toggle state
      flashState.isBright = !flashState.isBright;
      flashState.blinksRemaining--;

      // Apply new brightness state
      if (flashState.isBright) {
        ledcWriteDuty(ch, maxForRes(currentResolution)); // Turn ON (Full Brightness)
      } else {
        ledcWriteDuty(ch, 0); // Turn OFF (Zero Brightness)
      }

      // Schedule next toggle
      flashState.nextToggleMs = millis() + toggleInterval;

    } else {
      // Flash sequence finished. Restore original state.
      if (ch < LEDC_CHANNEL_COUNT) {
        ledcWriteDuty(ch, flashState.savedValue);
      }
      flashState.active = false;
      flashState.isBright = false;
      Serial.printf("Flash finished on channel %u, restored to %lu\n", ch, (unsigned long)flashState.savedValue);
    }
  }
}

void setupLEDCDriver() {
  // configure timer (explicit assignments)
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.speed_mode = LEDC_MODE;
  ledc_timer.duty_resolution = (ledc_timer_bit_t)currentResolution;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.freq_hz = LEDC_FREQ;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  // configure channels (explicit assignments)
  for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
    ledc_channel_config_t ch_conf;
    memset(&ch_conf, 0, sizeof(ch_conf));
    ch_conf.channel = (ledc_channel_t)ch;
    ch_conf.duty = 0;
    ch_conf.gpio_num = pwmPins[ch];
    ch_conf.speed_mode = LEDC_MODE;
    ch_conf.hpoint = 0;
    ch_conf.timer_sel = LEDC_TIMER;
    ledc_channel_config(&ch_conf);

    brightness[ch] = 0;
    ledcWriteDuty(ch, brightness[ch]);
  }
}

void handleDigitSelection(int digit) {
  if (digit < 0 || digit > 9) return;
  selectedIndex = digit;
  Serial.printf("Selected index %d (pin %u)\n", selectedIndex, (unsigned)pwmPins[selectedIndex]);
  startFlash(selectedIndex); // Changed to call the new flash system
}

void adjustBrightnessByDelta(int32_t deltaSteps) {
  uint32_t maxV = maxForRes(currentResolution);
  uint32_t currentBrightness = brightness[selectedIndex];

  if (deltaSteps > 0 && currentBrightness >= maxV) {
    startFlash(selectedIndex);
    return;
  }

  if (deltaSteps < 0 && currentBrightness == 0) {
    startFlash(selectedIndex);
    return;
  }

  uint32_t step = maxV / 16; if (step == 0) step = 1;
  int64_t newV = (int64_t)brightness[selectedIndex] + (int64_t)deltaSteps * (int64_t)step;
  if (newV < 0) newV = 0;
  if ((uint64_t)newV > maxV) newV = maxV;
  brightness[selectedIndex] = (uint32_t)newV;
  ledcWriteDuty(selectedIndex, brightness[selectedIndex]);
  Serial.printf("Channel %d brightness -> %lu/%lu\n", selectedIndex, (unsigned long)brightness[selectedIndex], (unsigned long)maxV);
}

void startEffect(uint8_t channel, EffectType type) {
  if (channel >= LEDC_CHANNEL_COUNT) return;

  // Stop any existing effect on the channel
  stopEffect(channel);

  effects[channel].type = type;
  effects[channel].active = true;
  effects[channel].startTime = millis();
  effects[channel].savedBrightness = brightness[channel];

  Serial.printf("Starting effect %d on channel %d\n", type, channel);
}

void stopEffect(uint8_t channel) {
  if (channel >= LEDC_CHANNEL_COUNT) return;

  if (effects[channel].active) {
    effects[channel].active = false;
    ledcWriteDuty(channel, effects[channel].savedBrightness);
    Serial.printf("Stopping effect on channel %d\n", channel);
  }
}

void processSamsungCommand_Normal(uint32_t rawCmd);

void processSamsungCommand(uint32_t rawCmd) {
  switch (systemState) {
    case STATE_NORMAL:
      processSamsungCommand_Normal(rawCmd);
      break;
    case STATE_AWAITING_CHAIN_CHANNEL: {
      uint8_t cmd = rawCmd & 0xFF;
      int selectedDigit = -1;
      switch (cmd) {
        case RemoteKeys::KEY_0: selectedDigit = 0; break;
        case RemoteKeys::KEY_1: selectedDigit = 1; break;
        case RemoteKeys::KEY_2: selectedDigit = 2; break;
        case RemoteKeys::KEY_3: selectedDigit = 3; break;
        case RemoteKeys::KEY_4: selectedDigit = 4; break;
        case RemoteKeys::KEY_5: selectedDigit = 5; break;
        case RemoteKeys::KEY_6: selectedDigit = 6; break;
        case RemoteKeys::KEY_7: selectedDigit = 7; break;
        case RemoteKeys::KEY_8: selectedDigit = 8; break;
        case RemoteKeys::KEY_9: selectedDigit = 9; break;
      }
      if (selectedDigit != -1) {
        effects[chaining_originalChannel].oneShot = true;
        effects[chaining_originalChannel].nextChannel = selectedDigit;
        effects[chaining_originalChannel].nextEffectType = chaining_nextEffectType;
        startEffect(chaining_originalChannel, chaining_effectType);

        systemState = STATE_NORMAL;
        // Restore brightness on all channels
        for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
          ledcWriteDuty(i, brightness[i]);
        }
      }
      break;
    }
  }
}

void processSamsungCommand_Normal(uint32_t rawCmd) {
  uint8_t cmd = rawCmd & 0xFF;
  switch (cmd) {
    case RemoteKeys::KEY_RED:
      if (effects[selectedIndex].active && effects[selectedIndex].type == EFFECT_PULSE) {
        stopEffect(selectedIndex);
      } else {
        startEffect(selectedIndex, EFFECT_PULSE);
      }
      break;
    case RemoteKeys::KEY_GREEN:
      if (effects[selectedIndex].active && effects[selectedIndex].type == EFFECT_FADE_IN_OUT) {
        stopEffect(selectedIndex);
      } else {
        startEffect(selectedIndex, EFFECT_FADE_IN_OUT);
      }
      break;
    case RemoteKeys::KEY_BLUE:
      systemState = STATE_AWAITING_CHAIN_CHANNEL;
      chaining_originalChannel = selectedIndex;
      chaining_effectType = EFFECT_PULSE;
      chaining_nextEffectType = EFFECT_PULSE;
      Serial.println("Awaiting next channel selection...");
      break;
    case RemoteKeys::KEY_YELLOW:
      systemState = STATE_AWAITING_CHAIN_CHANNEL;
      chaining_originalChannel = selectedIndex;
      chaining_effectType = EFFECT_PULSE;
      chaining_nextEffectType = EFFECT_FADE_IN_OUT;
      Serial.println("Awaiting next channel selection...");
      break;
    case RemoteKeys::KEY_0: handleDigitSelection(0); break;
    case RemoteKeys::KEY_1: handleDigitSelection(1); break;
    case RemoteKeys::KEY_2: handleDigitSelection(2); break;
    case RemoteKeys::KEY_3: handleDigitSelection(3); break;
    case RemoteKeys::KEY_4: handleDigitSelection(4); break;
    case RemoteKeys::KEY_5: handleDigitSelection(5); break;
    case RemoteKeys::KEY_6: handleDigitSelection(6); break;
    case RemoteKeys::KEY_7: handleDigitSelection(7); break;
    case RemoteKeys::KEY_8: handleDigitSelection(8); break;
    case RemoteKeys::KEY_9: handleDigitSelection(9); break;

    case RemoteKeys::KEY_VOL_UP:
      adjustBrightnessByDelta(+1);
      break;
    case RemoteKeys::KEY_VOL_DOWN:
      adjustBrightnessByDelta(-1);
      break;
    case RemoteKeys::KEY_MUTE:
      if (brightness[selectedIndex] == 0) {
        brightness[selectedIndex] = maxForRes(currentResolution) / 2;
      } else {
        brightness[selectedIndex] = 0;
      }
      ledcWriteDuty(selectedIndex, brightness[selectedIndex]);
      Serial.printf("Mute toggle channel %d\n", selectedIndex);
      break;

    case RemoteKeys::KEY_UP:
      reconfigureLEDCTimer(currentResolution + 1);
      break;
    case RemoteKeys::KEY_DOWN:
      reconfigureLEDCTimer(currentResolution - 1);
      break;

    case RemoteKeys::KEY_SOURCE:
      saveSettings();
      break;

    default:
      Serial.printf("Samsung key (unhandled): 0x%02X\n", cmd);
      break;
  }
}

void setup() {
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(MOTION_SENSOR_PIN, INPUT);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 500) { /* tiny non-blocking wait */ }

  Serial.println();
  Serial.println("ESP32 LEDC (driver) + IR remote starting...");

  loadSettings();
  setupLEDCDriver();

  // start IR receiver on pin 15, disable built-in LED feedback
  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  Serial.printf("IR receiver started on pin %u\n", IR_PIN);

  // initial flash
  startFlash(selectedIndex); // Changed to call the new flash system
  Serial.printf("Initial resolution: %u, brightness range 0..%lu\n", currentResolution, (unsigned long)maxForRes(currentResolution));
}

void loop() {
  updateFlash(); // The core function to handle the non-blocking flash logic
  updateEffects();
  updateGlobalModifiers();
  updateChannelSelectionIndicator();

  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == SAMSUNG &&
        IrReceiver.decodedIRData.address == 0x7) {
      Serial.printf("Command 0x%02X\n", (unsigned)IrReceiver.decodedIRData.command & 0xFF);
      processSamsungCommand(IrReceiver.decodedIRData.command);
    }
    IrReceiver.resume();
  }

  // keep loop responsive
  yield();
}

uint32_t lastIndicatorToggleMs = 0;
bool indicatorLedsOn = false;
const uint32_t indicatorToggleInterval = 250; // ms

void updateChannelSelectionIndicator() {
  if (systemState != STATE_AWAITING_CHAIN_CHANNEL) {
    return;
  }

  uint32_t currentTime = millis();
  if (currentTime - lastIndicatorToggleMs >= indicatorToggleInterval) {
    lastIndicatorToggleMs = currentTime;
    indicatorLedsOn = !indicatorLedsOn;

    if (indicatorLedsOn) {
      for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
        ledcWriteDuty(ch, maxForRes(currentResolution) / 4); // Dimly lit
      }
    } else {
      for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
        ledcWriteDuty(ch, 0);
      }
    }
  }
}

void saveSettings() {
  Preferences preferences;
  preferences.begin("led_settings", false);

  preferences.putBytes("brightness", brightness, sizeof(brightness));
  preferences.putBytes("effects", effects, sizeof(effects));

  preferences.end();
  indicateSave();
}

void indicateSave() {
  // Stop all effects to restore base brightness before flashing
  for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
    stopEffect(i);
  }

  // Flash twice
  for (int i = 0; i < 2; ++i) {
    // Turn all on
    for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
      ledcWriteDuty(ch, maxForRes(currentResolution));
    }
    delay(80);
    // Turn all off
    for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
      ledcWriteDuty(ch, 0);
    }
    delay(80);
  }

  // Restore original brightness from the main brightness array
  for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
    ledcWriteDuty(i, brightness[i]);
  }
}

void loadSettings() {
  Preferences preferences;
  preferences.begin("led_settings", true);

  preferences.getBytes("brightness", brightness, sizeof(brightness));
  preferences.getBytes("effects", effects, sizeof(effects));

  // Reset transient state of effects after loading
  for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
    effects[i].active = false;
    effects[i].startTime = 0;
    effects[i].oneShot = false;
    effects[i].nextChannel = 0xFF;
  }

  preferences.end();
}

void updateGlobalModifiers() {
  // Light sensor logic
  int lightReading = analogRead(LIGHT_SENSOR_PIN);
  globalDimming = 1.0 - (float)lightReading / 4095.0; // Dim more in bright light

  // Motion sensor logic
  motionDetected = digitalRead(MOTION_SENSOR_PIN) == HIGH;
  if (motionDetected) {
    lastMotionTime = millis();
  }

  // Override dimming if motion was detected recently at night
  const float nightThreshold = 0.5;
  if (millis() - lastMotionTime < motionOverrideDuration && globalDimming > nightThreshold) {
    globalDimming = 1.0;
  }
}

void updateEffects() {
  uint32_t currentTime = millis();
  for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
    if (effects[i].active) {
      bool effectFinished = false;
      uint32_t elapsedTime = currentTime - effects[i].startTime;

      switch (effects[i].type) {
        case EFFECT_PULSE: {
          float period = 1000.0 / effects[i].frequency;
          if (effects[i].oneShot && elapsedTime >= period) {
            effectFinished = true;
          } else {
            uint32_t cycleTime = elapsedTime % (uint32_t)period;
            float duty = (effects[i].dutyCycle / 100.0) * period;
            uint32_t newBrightness = (cycleTime < duty) ? effects[i].savedBrightness : 0;
            ledcWriteDuty(i, newBrightness);
          }
          break;
        }
        case EFFECT_FADE_IN_OUT: {
          if (effects[i].oneShot && elapsedTime >= effects[i].duration) {
            effectFinished = true;
          } else {
            float progress = (float)(elapsedTime % effects[i].duration) / (float)effects[i].duration;
            float sineWave = (sin(progress * 2 * PI - PI / 2) + 1) / 2.0;
            uint32_t newBrightness = sineWave * effects[i].savedBrightness;
            ledcWriteDuty(i, newBrightness);
          }
          break;
        }
        default:
          break;
      }

      if (effectFinished) {
        uint8_t nextChannel = effects[i].nextChannel;
        EffectType nextEffectType = effects[i].nextEffectType;
        stopEffect(i);
        if (nextChannel != 0xFF) {
          startEffect(nextChannel, nextEffectType);
        }
      }
    }
  }
}
