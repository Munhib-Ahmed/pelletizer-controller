// =============================================================================
//  PELLETIZER CONTROLLER — ESP32 + AccelStepper + Beam Break Sensor
//  Target MCU : ESP32 (Arduino-ESP32 core)
//  Driver      : A4988 / TMC2209 (STEP/DIR interface)
//  Author      : Embedded Systems Engineer
//  Version     : 2.0.0
// =============================================================================
//
//  ┌─────────────────────────────────────────────────────────────────────────┐
//  │  WIRING SUMMARY                                                          │
//  │                                                                          │
//  │  ESP32 Pin        Driver / Peripheral                                    │
//  │  ─────────────    ───────────────────────────────────────────────────    │
//  │  GPIO 26  ──────► STEP  pin on A4988 / TMC2209                          │
//  │  GPIO 27  ──────► DIR   pin on A4988 / TMC2209                          │
//  │  GPIO 14  ──────► ENA   pin on driver  (LOW = enabled)                  │
//  │  3.3 V    ──────► LOGIC VCC on driver                                   │
//  │  GND      ──────► GND   on driver                                        │
//  │                                                                          │
//  │  GPIO 34  ──────── Potentiometer wiper (center pin)                     │
//  │  3.3 V    ──────── Pot high end                                          │
//  │  GND      ──────── Pot low end                                           │
//  │                                                                          │
//  │  GPIO 18  ──────── Start/Stop push-button (other leg to GND)            │
//  │                    Internal pull-up enabled; button is ACTIVE LOW.      │
//  │                                                                          │
//  │  BEAM BREAK SENSOR (e.g. Adafruit 2167 or bare IR emitter/receiver):    │
//  │  GPIO 19  ──────── Receiver/sensor output signal                        │
//  │  3.3 V    ──────── Sensor VCC (check your module; some need 5 V)        │
//  │  GND      ──────── Sensor GND                                            │
//  │                                                                          │
//  │  BEAM BREAK LOGIC:                                                       │
//  │    Most modules output LOW  when beam is BROKEN  (plastic present)      │
//  │                     HIGH when beam is CLEAR   (no plastic)              │
//  │    → Motor runs   when beam is BROKEN  (LOW)                            │
//  │    → Motor stops  when beam is CLEAR   (HIGH)                           │
//  │    If your module is inverted, flip BEAM_ACTIVE_STATE below.            │
//  │                                                                          │
//  │  SYSTEM LOGIC OVERVIEW:                                                  │
//  │    The Start/Stop button acts as a MASTER ENABLE.                        │
//  │    When master is OFF → motor always stopped regardless of beam.        │
//  │    When master is ON  → beam break controls motor automatically.        │
//  │                                                                          │
//  │  SHARED GND: ESP32, driver, sensor, PSU ground must all be common.      │
//  └─────────────────────────────────────────────────────────────────────────┘
//
// =============================================================================

#include <AccelStepper.h>

// ─────────────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS  — change these to match your wiring diagram
// ─────────────────────────────────────────────────────────────────────────────
constexpr uint8_t PIN_STEP        = 26;   // STEP signal to driver
constexpr uint8_t PIN_DIR         = 27;   // DIR  signal to driver
constexpr uint8_t PIN_ENA         = 14;   // ENA  signal to driver (active LOW)
constexpr uint8_t PIN_POT         = 34;   // Potentiometer wiper (ADC1 channel)
constexpr uint8_t PIN_BUTTON      = 18;   // Start/Stop push-button (active LOW)
constexpr uint8_t PIN_BEAM        = 19;   // Beam break sensor output

// ─────────────────────────────────────────────────────────────────────────────
//  BEAM BREAK CONFIGURATION
//
//  BEAM_ACTIVE_STATE is the pin level that means "beam is broken" (plastic
//  is present and interrupting the IR beam).
//
//  Most commercial modules (Adafruit, generic IR break-beam):
//    LOW  = beam broken  → set BEAM_ACTIVE_STATE = LOW   ← default
//    HIGH = beam clear
//
//  If your module is the opposite (HIGH = broken), change to HIGH.
// ─────────────────────────────────────────────────────────────────────────────
constexpr uint8_t BEAM_ACTIVE_STATE = LOW;   // Level when beam IS broken

// How long (ms) the beam must be continuously CLEAR before the motor stops.
// A small hold-off prevents the motor from stuttering if the plastic strand
// momentarily passes a gap (e.g., between pellets).
// Increase if your material feed is intermittent between cuts.
constexpr uint32_t BEAM_CLEAR_HOLDOFF_MS = 300;

// ─────────────────────────────────────────────────────────────────────────────
//  MOTOR / DRIVE TRAIN CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────
constexpr uint16_t FULL_STEPS_PER_REV = 200;    // 1.8° stepper = 200 steps/rev
constexpr uint8_t  MICROSTEPS         = 16;      // Match your driver MS pins
constexpr uint32_t STEPS_PER_REV      = FULL_STEPS_PER_REV * MICROSTEPS; // 3200

// ─────────────────────────────────────────────────────────────────────────────
//  SPEED RANGE  (steps per second)
//  Example @ STEPS_PER_REV = 3200:
//    320  steps/s →   6 RPM
//    6400 steps/s → 120 RPM
// ─────────────────────────────────────────────────────────────────────────────
constexpr float MIN_SPEED = 320.0f;
constexpr float MAX_SPEED = 6400.0f;

// ─────────────────────────────────────────────────────────────────────────────
//  TIMING CONSTANTS
// ─────────────────────────────────────────────────────────────────────────────
constexpr uint16_t ADC_MAX                  = 4095;
constexpr uint32_t SPEED_UPDATE_INTERVAL_MS = 200;
constexpr uint32_t DEBOUNCE_DELAY_MS        = 50;

// ─────────────────────────────────────────────────────────────────────────────
//  AccelStepper INSTANCE
// ─────────────────────────────────────────────────────────────────────────────
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// ─────────────────────────────────────────────────────────────────────────────
//  RUNTIME STATE
// ─────────────────────────────────────────────────────────────────────────────

// ── Master enable (controlled by push-button) ─────────────────────────────
bool masterEnabled = false;       // true = system armed; beam break takes over

// ── Beam break state ──────────────────────────────────────────────────────
bool     beamBroken        = false;   // Current debounced beam state
uint32_t beamClearSince    = 0;       // Timestamp beam went clear
bool     motorRunning      = false;   // Actual motor on/off state

// ── Button debounce ───────────────────────────────────────────────────────
bool     lastButtonReading = HIGH;
bool     buttonState       = HIGH;
uint32_t lastDebounceTime  = 0;

// ── Speed scheduling ─────────────────────────────────────────────────────
uint32_t lastSpeedUpdate   = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  HELPERS
// ─────────────────────────────────────────────────────────────────────────────
inline void enableMotor()
{
    digitalWrite(PIN_ENA, LOW);
    motorRunning = true;
}

inline void disableMotor()
{
    digitalWrite(PIN_ENA, HIGH);
    motorRunning = false;
}

float readTargetSpeed()
{
    uint16_t raw = analogRead(PIN_POT);
    return MIN_SPEED + (static_cast<float>(raw) / ADC_MAX) * (MAX_SPEED - MIN_SPEED);
}

// ─────────────────────────────────────────────────────────────────────────────
//  printStatus() — human-readable state dump to Serial
// ─────────────────────────────────────────────────────────────────────────────
void printStatus(const char* event)
{
    Serial.print(F("["));
    Serial.print(event);
    Serial.print(F("]  Master="));
    Serial.print(masterEnabled ? F("ON ") : F("OFF"));
    Serial.print(F("  Beam="));
    Serial.print(beamBroken   ? F("BROKEN") : F("CLEAR "));
    Serial.print(F("  Motor="));
    Serial.println(motorRunning ? F("RUNNING") : F("STOPPED"));
}

// =============================================================================
//  SETUP
// =============================================================================
void setup()
{
    Serial.begin(115200);
    Serial.println(F("\n=== Pelletizer Controller v2.0 Starting ==="));

    // ── Configure pins ────────────────────────────────────────────────────────
    pinMode(PIN_ENA,    OUTPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);   // Active LOW
    pinMode(PIN_BEAM,   INPUT_PULLUP);   // Pull-up in case module floats when
                                         // no beam; adjust if module has its own
                                         // pull-up/down resistor

    // ── Safety: start with motor disabled ────────────────────────────────────
    disableMotor();

    // ── AccelStepper ──────────────────────────────────────────────────────────
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setSpeed(readTargetSpeed());

    Serial.print(F("Steps/rev  : ")); Serial.println(STEPS_PER_REV);
    Serial.print(F("Speed range: "));
    Serial.print(MIN_SPEED); Serial.print(F(" – "));
    Serial.print(MAX_SPEED); Serial.println(F(" steps/s"));
    Serial.println(F("Press START button to arm system."));
    Serial.println(F("Motor will run automatically when beam is broken."));
}

// =============================================================================
//  LOOP
// =============================================================================
void loop()
{
    // ── 1. STEP THE MOTOR — must be called every iteration ───────────────────
    if (motorRunning)
    {
        stepper.runSpeed();
    }

    // ── 2. DEBOUNCE & DETECT START/STOP BUTTON ───────────────────────────────
    {
        bool reading = digitalRead(PIN_BUTTON);

        if (reading != lastButtonReading)
        {
            lastDebounceTime  = millis();
            lastButtonReading = reading;
        }

        if ((millis() - lastDebounceTime) >= DEBOUNCE_DELAY_MS)
        {
            if (reading != buttonState)
            {
                buttonState = reading;

                if (buttonState == LOW)   // Falling edge = button pressed
                {
                    masterEnabled = !masterEnabled;

                    if (!masterEnabled)
                    {
                        // Master turned OFF — stop motor immediately regardless
                        // of beam state
                        disableMotor();
                        printStatus("MASTER OFF");
                        Serial.println(F("System disarmed. Press button to re-arm."));
                    }
                    else
                    {
                        printStatus("MASTER ON");
                        Serial.println(F("System armed. Beam break now controls motor."));
                        // Motor will start on next loop if beam is already broken
                    }
                }
            }
        }
    }   // end button block

    // ── 3. READ BEAM BREAK SENSOR & CONTROL MOTOR AUTOMATICALLY ─────────────
    //
    //  State machine:
    //
    //    masterEnabled == false  → motor always off (handled in button block)
    //
    //    masterEnabled == true:
    //      Beam BROKEN  (plastic present) → start motor immediately
    //      Beam CLEAR   (no plastic)      → wait BEAM_CLEAR_HOLDOFF_MS,
    //                                       then stop motor
    //
    //  The hold-off prevents rapid start/stop if the strand briefly passes
    //  through a gap between pellets during cutting.
    // ─────────────────────────────────────────────────────────────────────────
    if (masterEnabled)
    {
        bool rawBeam = digitalRead(PIN_BEAM);
        bool beamIsNowBroken = (rawBeam == BEAM_ACTIVE_STATE);

        if (beamIsNowBroken)
        {
            // Beam is broken — plastic is present
            beamClearSince = 0;   // Reset the clear timer

            if (!beamBroken)
            {
                // Transition: CLEAR → BROKEN
                beamBroken = true;
                stepper.setSpeed(readTargetSpeed());   // Refresh speed from pot
                enableMotor();
                printStatus("BEAM BROKEN");
            }
        }
        else
        {
            // Beam is clear — no plastic detected
            if (beamBroken)
            {
                // Transition: BROKEN → CLEAR — start the hold-off timer
                beamBroken     = false;
                beamClearSince = millis();
                printStatus("BEAM CLEAR - holding off");
            }

            // Check if hold-off period has elapsed
            if (motorRunning && beamClearSince > 0 &&
                (millis() - beamClearSince) >= BEAM_CLEAR_HOLDOFF_MS)
            {
                disableMotor();
                beamClearSince = 0;
                printStatus("MOTOR STOPPED - beam clear timeout");
            }
        }
    }   // end beam break block

    // ── 4. UPDATE SPEED FROM POTENTIOMETER (throttled) ───────────────────────
    if ((millis() - lastSpeedUpdate) >= SPEED_UPDATE_INTERVAL_MS)
    {
        lastSpeedUpdate = millis();

        float newSpeed = readTargetSpeed();
        stepper.setSpeed(newSpeed);

        if (motorRunning)
        {
            Serial.print(F("Speed: "));
            Serial.print(newSpeed, 1);
            Serial.print(F(" steps/s  (~"));
            Serial.print((newSpeed * 60.0f) / STEPS_PER_REV, 1);
            Serial.println(F(" RPM)"));
        }
    }

}   // end loop()