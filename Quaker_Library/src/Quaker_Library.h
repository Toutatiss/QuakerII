#ifndef QUAKER_LIBRARY_H
#define QUAKER_LIBRARY_H

// Premade Libraries
#include <Wire.h>
#include <BlockNot.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include "Adafruit_ADS1X15.h"
#include "TMCStepper.h"

// Pico Functionality
#include "pico/multicore.h"

// Custom Definitions
#include <Menu.h>
#include <MenuName.h>
#include <MainMenuFunction.h>

// Address Definitions
constexpr uint8_t ENCODER_ADDRESS = 0x36;

// Pin Definitions
// Seesaw Internal Pins
constexpr uint8_t ENCODER_BUTTON = 24;
constexpr uint8_t ENCODER_NEOPIX = 6;

// Pico Pins
// Encoder
constexpr uint8_t INT_PIN = 15;

// Stepper
constexpr uint8_t EN_PIN = 22;
constexpr uint8_t STEP_PIN = 21;
constexpr uint8_t DIR_PIN = 20;

// TMCStepper
inline HardwareSerial &SERIAL_PORT = Serial1;
// #define SERIAL_PORT Serial1  // TMC2208/TMC2224 HardwareSerial port

// Interrupt Routine
void _interruptRoutine();

class Quaker
{
public:
    Quaker();

    // Core 0
    void begin();
    void runUI();

    // Core 1
    void begin1();
    void runMotor();

    // Interrupt flag setter
    volatile void setInterruptTriggered(bool state);

private:
    // Library instantiations
    // Code Functionality
    BlockNot _buttonTimer;
    BlockNot _debounceTimer;
    BlockNot _blinkTimer;
    BlockNot _stepperTimer;
    BlockNot _speedRampTimer;
    BlockNot _readADCTimer;
    BlockNot _displayUpdateTimer;

    // Peripherals
    LiquidCrystal_I2C _lcd;
    Adafruit_seesaw _encoder;
    seesaw_NeoPixel _neoPixel;
    Adafruit_ADS1015 _adc;

    // TMC Stepper Driver
    TMC2209Stepper _driver;
    const uint8_t DRIVER_ADDRESS = 0b00; // TMC2209 Driver address according to MS1 and MS2
    const float R_SENSE = 0.11f;
    const int MICROSTEPS = 2; // 0 = Fullstep
    const uint16_t RMS_CURRENT = 1200;

    // Peripheral Initialisers
    void _initI2CLCD();
    void _initEncoder();
    void _initStepper();
    void _initADC();
    void _initWeightSensor();

    // Interrupt Management
    void _initEncoderInterrupts();
    void _handleInterrupt();
    // static void _interruptRoutine();
    volatile bool _interruptTriggered = false;

    // Menus
    Menu _splashScreen;
    Menu _mainMenu;

    // Menu Trackers
    Menu _previousMenu = _splashScreen;
    Menu _currentMenu = _mainMenu;

    // Menu Display Functions
    void _displaySplashScreen();
    void _displayMainMenu();
    void _printSpeed();
    void _eraseSpeed();
    void _updatePeakPressure();

    // User Input
    bool _buttonPressed = false;
    bool _speedBlinkState = false;
    void _adjustQuakeFrequency(int8_t *encoderDelta);
    void _moveCursor(int8_t *encoderDelta);
    void _performAction();

    // Motor
    bool _motorOn = false;
    bool _needToAccelerate = false;
    bool _settingSpeed = false;
    float _pulseDuration = 150;

    uint8_t _quakeFrequency = 20;          // In Hz
    const uint8_t _maxQuakeFrequency = 20; // In Hz
    const uint8_t _minQuakeFrequency = 1;  // In Hz

    void _toggleMotor();
    void _setSpeed();
    float _calculateStepTime(float frequency);

    // Load Cell and Pressure Reading
    int _offset = 0;
    int _rawPeakPressure;
    float _peakPressure_kPa;
    void _getPressureReading();
    float _convertPressureTokPa(int rawADCValue);

    // Stepper Motor Characteristics
    const uint8_t _stepsPerRev = 200;
    const uint8_t _gearReduction = 3;
    // const float _stepAngle = 0.45; // 1.8 / quarter steps
    const float _stepAngle = 0.9; // Half steps
    // const float _stepAngle = 1.8;

    // Hardware setup characteristics
    const float _cams = 10.0;
};

#endif
