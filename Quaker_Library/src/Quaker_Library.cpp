#include "Quaker_Library.h"

// Debug Stream //
// #define DEBUG_STREAM Serial // Comment this out to remove debugging lines

#ifdef DEBUG_STREAM
#define debugPrint(...) DEBUG_STREAM.print(__VA_ARGS__)
#define debugPrintln(...) DEBUG_STREAM.println(__VA_ARGS__)
#else
#define debugPrint(...)
#define debugPrintln(...)
#endif

// Data Stream //
#define DATA_STREAM Serial // Comment this out to remove debugging lines

#ifdef DATA_STREAM
#define dataStreamPrint(...) DATA_STREAM.print(__VA_ARGS__)
#define dataStreamPrintln(...) DATA_STREAM.println(__VA_ARGS__)
#else
#define dataStreamPrint(...)
#define dataStreamPrintln(...)
#endif

// Global Pointer
Quaker *quakerInstance = nullptr; // Global pointer to hold the instance

// Interrupt Routine
void _interruptRoutine()
{
    if (quakerInstance)
    {
        quakerInstance->setInterruptTriggered(true); // Use setter method
    }
}

// Constructor
Quaker::Quaker()
    : // Timers
      _buttonTimer(50),
      _debounceTimer(200),
      _blinkTimer(500),
      _stepperTimer(150, MICROSECONDS),
      _speedRampTimer(100),
      _readADCTimer(20),
      _displayUpdateTimer(500),

      // Peripherals
      _lcd(0x27, 20, 4),
      _encoder(),
      _neoPixel(1, ENCODER_NEOPIX, NEO_GRB + NEO_KHZ800),
      _adc(),
      _driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS),

      // Menus
      _splashScreen(MenuName::SPLASH_SCREEN, 0, {0}),
      _mainMenu(MenuName::MAIN_MENU, 1, {0, 6})

{
    quakerInstance = this;

    // Set initial menu state
    _previousMenu = _splashScreen;
    _currentMenu = _mainMenu;
}

// *** SECTION: PUBLIC FUNCTIONS *** //
// Core 0
void Quaker::begin()
{
    // Initialise Premade Libraries
    debugPrintln("Initialising LCD");
    _initI2CLCD();

    debugPrintln("Initialising Encoder");
    _initEncoder();

    debugPrintln("Initialising Encoder Interrupts");
    _initEncoderInterrupts();

    debugPrintln("Initialising Stepper");
    _initStepper();

    debugPrintln("Initialising ADC");
    _initADC();

    debugPrintln("Zeroing Weight Sensor");
    _initWeightSensor();

    dataStreamPrintln("Pressure (kPa):");
}

void Quaker::runUI()
{
    // Load New Menu
    if (_currentMenu.menuName != _previousMenu.menuName)
    {
        switch (_currentMenu.menuName)
        {
        case MenuName::SPLASH_SCREEN:
            _displaySplashScreen();
            break;
        case MenuName::MAIN_MENU:
            _displayMainMenu();
            break;
        default:
            debugPrintln("Error in Load New Menu: Unknown Menu Requested");
            break;
        }
        _previousMenu = _currentMenu; // Update previous menu after processing
    }

    // Handle any user-input interrupts
    _handleInterrupt();

    // Read peak pressure
    _getPressureReading();

    // Display peak pressure
    if (_displayUpdateTimer.TRIGGERED)
    {
        _peakPressure_kPa = _convertPressureTokPa(_rawPeakPressure);
        _updatePeakPressure();
    }
}

// Core 1
void Quaker::begin1()
{
}

void Quaker::runMotor()
{
    // Variables
    static bool motorOn = false;
    // static float stepTime = 150;

    static bool pinState = false;

    // Process incoming information from Core 0
    uint32_t command;
    if (rp2040.fifo.pop_nb(&command))
    {
        motorOn = bool(command);
    }

    // Exit if the motor state variable is disabled
    if (!motorOn)
    {
        return;
    }

    if (_needToAccelerate)
    {
        static const int startingFrequency = 1;
        static int rampFrequency = startingFrequency; // Start at 1 Hz

        if (_speedRampTimer.TRIGGERED) // Periodically ramp up the speed
        {
            _stepperTimer.setDuration(_calculateStepTime(rampFrequency));
            rampFrequency++;
        }

        if (rampFrequency >= _quakeFrequency) // One we reach the set speed, reset and exit acceleration routine
        {
            _needToAccelerate = false;
            rampFrequency = startingFrequency;
        }
    }

    // Otherwise drive the motor at the given speed
    if (_stepperTimer.TRIGGERED)
    {
        pinState = !pinState;
        digitalWrite(STEP_PIN, pinState);
    }
}

// Interrupt flag setter
volatile void Quaker::setInterruptTriggered(bool value)
{
    _interruptTriggered = value;
    _debounceTimer.RESET;
}

// *** SECTION: PRIVATE FUNCTIONS *** //
// ~~~ Peripheral Initialisers ~~~ //
void Quaker::_initI2CLCD()
{
    _lcd.init();
    _lcd.backlight();
    _lcd.clear();
    debugPrintln("LCD started.");
    _displaySplashScreen();
}

void Quaker::_initEncoder()
{
    // Start up the encoder
    if (!_encoder.begin(ENCODER_ADDRESS) || !_neoPixel.begin(ENCODER_ADDRESS))
    {
        debugPrintln("ERROR in initEncoder: Couldn't find encoder on default address.");
        while (1)
            delay(10);
    }

    // Get encoder version
    debugPrintln("Encoder started.");
    uint32_t version = ((_encoder.getVersion() >> 16) & 0xFFFF);
    if (version != 4991)
    {
        debugPrint("ERROR in initEncoder: Wrong firmware loaded?");
        debugPrintln(version);
        while (1)
            delay(10);
    }
    debugPrintln("Found Product 4991.");

    // Configure Neopixel
    _neoPixel.setBrightness(20);
    _neoPixel.show();

    // Configure Encoder Button
    _encoder.pinMode(ENCODER_BUTTON, INPUT_PULLUP);

    // Enable Interrupts
    debugPrintln("Turning on encoder interrupts");
    delay(10);
    _encoder.setGPIOInterrupts((uint32_t)1 << ENCODER_BUTTON, 1);
    _encoder.enableEncoderInterrupt();
}

void Quaker::_initStepper()
{
    // Hardware Setup
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    digitalWrite(EN_PIN, LOW); // Enable driver in hardware

    // Configure TMCStepper Driver
    SERIAL_PORT.begin(115200); // Startup UART with driver
    _driver.begin();
    _driver.toff(5);                // Enables driver in software
    _driver.microsteps(MICROSTEPS); // Set step resolution
    _driver.pwm_autoscale(true);    // Needed for stealthChop

    // NOTE: Disabling this line fixed my issue with stalling too easily under load!
    // _driver.rms_current(RMS_CURRENT); // Set motor RMS current

    _pulseDuration = _calculateStepTime(_quakeFrequency);
    _stepperTimer.setDuration(_pulseDuration);
    debugPrintln(_pulseDuration);
}

void Quaker::_initADC()
{
    _adc.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    if (!_adc.begin())
    {
        debugPrintln("Error In InitADC: Failed to initialize ADS.");
        while (1)
            ;
    }
}

void Quaker::_initWeightSensor()
{
    // Take a sec to zero the sensor by determining the offset

    // Variables
    int average = 0;
    const uint8_t numReadings = 5;
    const uint8_t readingDelayMillis = 100;

    // Take a few readings
    for (int i = 0; i < numReadings; i++)
    {
        average += _adc.readADC_SingleEnded(0);
        delay(readingDelayMillis);
    }

    // Take the average
    average /= numReadings;

    // Set the offset
    _offset = average;
}

// ~~~ Interrupt Management ~~~ //
void Quaker::_initEncoderInterrupts()
{
    // Enable interrupts from the switch and encoder
    _encoder.setGPIOInterrupts((uint32_t)1 << ENCODER_BUTTON, 1);
    _encoder.enableEncoderInterrupt();

    // Pin assignment and hardware interrupt setup
    pinMode(INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), _interruptRoutine, FALLING);
}

void Quaker::_handleInterrupt()
{
    uint8_t switchReading;
    int8_t encoderDelta;

    if (_interruptTriggered)
    {
        _interruptTriggered = false; // clear interrupt flag

        // Read delta
        encoderDelta = _encoder.getEncoderDelta(); // Clears delta interrupt

        // Read button
        switchReading = _encoder.digitalRead(ENCODER_BUTTON); // Clears switch interrupt

        // Interpret delta
        if (!encoderDelta)
        { // No delta - the button was pressed (debounce)
            _buttonPressed = true;
        }
        else
        { // Delta - the switch was rotated
            debugPrintln(encoderDelta);
            if (_settingSpeed)
            {
                _adjustQuakeFrequency(&encoderDelta);
            }
            else
            {
                _moveCursor(&encoderDelta);
            }
        }
    }

    // Debounce and clear button interrupt
    if (_buttonPressed)
    {
        if (_debounceTimer.TRIGGERED)
        {
            _buttonPressed = false; // Immediately clear button press before further action
            debugPrintln("Button Pressed!");

            // Perform any needed actions
            _performAction();
        }
    }
}

// ~~~ Display Screens ~~~ //
void Quaker::_displaySplashScreen()
{
    debugPrintln("Displaying Splash Screen");

    _lcd.setCursor(3, 0);
    _lcd.print("Quaker II");
    _lcd.setCursor(3, 1);
    _lcd.print("Loading");

    for (int i = 0; i < 3; i++)
    {
        delay(250);
        _lcd.print(".");
    }

    delay(1000);
    _lcd.clear();
}

void Quaker::_displayMainMenu()
{
    debugPrintln("Displaying Main Menu");

    _lcd.clear();

    // First Row
    _lcd.setCursor(0, 0);
    _lcd.print(">RUN");
    _lcd.setCursor(7, 0);
    _lcd.print("SPD: ");
    _printSpeed();
    _lcd.print("Hz");

    // Second Row
    _lcd.setCursor(0, 1);
    _lcd.print("PRESS: xxx kPa");
}

void Quaker::_printSpeed()
{
    if (_quakeFrequency < 10)
    {
        _lcd.setCursor(13, 0);
    }
    else
    {
        _lcd.setCursor(12, 0);
    }
    _lcd.print(_quakeFrequency);
}

void Quaker::_eraseSpeed()
{
    _lcd.setCursor(12, 0);
    _lcd.print("  ");
}

void Quaker::_updatePeakPressure()
{
    static int previousPressure = -1;

    // Don't update if the pressure is the same, to reduce flickering
    if (_peakPressure_kPa == previousPressure)
    {
        return;
    }

    // Erase the old pressure
    _lcd.setCursor(7, 1);
    _lcd.print("    ");

    // Draw the new one
    _lcd.setCursor(7, 1);
    _lcd.print(_peakPressure_kPa, 0);

    // Draw the units
    _lcd.print(" kPa     ");

    // Store the previous pressure
    previousPressure = _peakPressure_kPa;
}

// ~~~ User Input ~~~ //
void Quaker::_adjustQuakeFrequency(int8_t *encoderDelta)
{
    const uint8_t newFrequency = _quakeFrequency + *encoderDelta;

    // Reset the blink timer to keep the speed shown,
    // then leave the function without doing anything if speed goes out of bounds
    if (newFrequency < _minQuakeFrequency || newFrequency > _maxQuakeFrequency)
    {
        return;
    }

    // Otherwise commit to the adjustment
    _quakeFrequency = newFrequency;
    _stepperTimer.setDuration(_calculateStepTime(_quakeFrequency));

    // Reset the blink timer to keep showing the speed
    _speedBlinkState = true;
    _blinkTimer.RESET;

    // Immediately draw the new speed to the screen
    _eraseSpeed();
    _printSpeed();
}

void Quaker::_moveCursor(int8_t *encoderDelta)
{
    uint8_t min = 0;
    uint8_t max = _currentMenu.maxPosition;
    int8_t newIndex = _currentMenu.positionIndex + *encoderDelta;

    // Leave the function without doing anything if cursor goes out of bounds
    if (newIndex < min || newIndex > max)
    {
        return;
    }

    // Erase the original cursor
    _lcd.setCursor(_currentMenu.positionArray[_currentMenu.positionIndex], 0);
    _lcd.print(" ");

    // Update posiiton and draw cursor there
    _currentMenu.positionIndex = newIndex;
    _lcd.setCursor(_currentMenu.positionArray[_currentMenu.positionIndex], 0);
    _lcd.print(">");
}

void Quaker::_performAction()
{
    if (_currentMenu.menuName == MenuName::MAIN_MENU)
    { // If in main menu
        switch (_currentMenu.positionIndex)
        {
        case (int)MainMenuFunction::TOGGLE_MOTOR:
            debugPrintln("performing action: toggle motor");
            _toggleMotor();
            break;

        case (int)MainMenuFunction::SET_SPEED:
            debugPrintln("performing action: set speed");
            _settingSpeed = !_settingSpeed; // Toggle speed setting mode
            _setSpeed();
            break;

        default:
            break;
        }
    }
}

// ~~~ Motor ~~~ //
void Quaker::_toggleMotor()
{
    // Change State
    _motorOn = !_motorOn;
    rp2040.fifo.push_nb(_motorOn);

    // Clear UI Text
    _lcd.setCursor(1, 0);
    _lcd.print("    ");
    _lcd.setCursor(1, 0);

    // Write New UI Text and call for acceleration
    if (_motorOn)
    {
        _lcd.print("STOP");
        _needToAccelerate = true;
    }
    else
    {
        _lcd.print("RUN");
    }
}

void Quaker::_setSpeed()
{
    // Lock into Set Speed until button is pressed again
    while (_settingSpeed)
    {
        // Blink the current speed
        if (_blinkTimer.TRIGGERED)
        {
            _speedBlinkState = !_speedBlinkState;
            _speedBlinkState ? _printSpeed() : _eraseSpeed();
        }

        // Check for interrupts before next loop
        _handleInterrupt();
    }
    // Ensure speed is displayed before exit
    _printSpeed();
}

float Quaker::_calculateStepTime(float frequency)
{
    return round(1e6 * ((_stepAngle / _gearReduction) * _cams) / (360 * 2 * frequency));
}

// ~~~ Load Cell and Pressure Reading ~~~ //
void Quaker::_getPressureReading()
{
    // Skip reading unless reading timer is triggered
    if (!_readADCTimer.TRIGGERED)
    {
        return;
    }

    // Variables
    static int prevReading = _offset;
    static int currReading = 0;
    static int prevDerivative = 0;
    static int currDerivative = 0;

    // Get current reading
    currReading = _adc.readADC_SingleEnded(0); // Raw ADC value
    dataStreamPrintln(_convertPressureTokPa(currReading));

    // Calculate current derivative
    currDerivative = currReading - prevReading;

    // Check for sign change (peak detected)
    if (prevDerivative > 0 && currDerivative < 0)
    {
        _rawPeakPressure = currReading;
    }

    // Shift current reading and derivative to their "previous" containers
    prevReading = currReading;
    prevDerivative = currDerivative;
}

float Quaker::_convertPressureTokPa(int rawADCValue)
{
    // debugPrintln(_rawPeakPressure);
    const float pressureKg = 0.01 * rawADCValue - (_offset * 1e-2);
    return 98.0665 * pressureKg; // Calculate pressure in kPa
}