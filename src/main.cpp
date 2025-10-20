// partially based on SalimBenBouz's mouse (https://www.thingiverse.com/thing:6102343)

#include <Arduino.h>
#include <OneButton.h>
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_TinyUSB.h>
#include <Adafruit_NeoPixel.h>
#include <RP2040_PWM.h>
#include <Tween.h>
#include <pico/sleep.h>
#include <hardware/rosc.h>

static constexpr int SPACEMOUSE_PIN_BUTTON1 = 15;
static constexpr int SPACEMOUSE_PIN_BUTTON2 = 14;
[[maybe_unused]] static constexpr int SPACEMOUSE_PIN_SDA = 26;
[[maybe_unused]] static constexpr int SPACEMOUSE_PIN_SCL = 27;
static constexpr int SPACEMOUSE_PIN_LED1 = 28;
static constexpr int SPACEMOUSE_PIN_LED2 = 29;
static constexpr int SPACEMOUSE_PIN_RGB = 13; // Changed to a unique pin for WS2812

static constexpr unsigned long LED_FADING_INACTIVITY_TIME = 1000 * 590;
static constexpr unsigned long MAX_INACTIVITY_TIME = LED_FADING_INACTIVITY_TIME + ( 1000 * 10 );

static constexpr float PWM_FREQ = 1000;

static constexpr int MAGNETOMETER_SENSITIVITY = 100;
static constexpr int MAGNETOMETER_INPUT_RANGE_t = 2.5 * MAGNETOMETER_SENSITIVITY;
static constexpr int MAGNETOMETER_INPUT_Z_RANGE = MAGNETOMETER_INPUT_RANGE_t * 8;
static constexpr int MAGNETOMETER_INPUT_RANGE = MAGNETOMETER_INPUT_Z_RANGE; // same scaling for all axes
static constexpr float MAGNETOMETER_XY_THRESHOLD = 0.4;
static constexpr float MAGNETOMETER_Z_THRESHOLD = 0.9;
static constexpr float MAGNETOMETER_Z_ORBIT_MAX_THRESHOLD = 1.7;

static constexpr int HID_POLL_INTERVAL = 4;
static constexpr int HID_REPORT_LOGICAL_RANGE = 350;
static constexpr int HID_REPORT_PHYSICAL_RANGE = 1400;
static constexpr int HID_MAX_BUTTONS = 24;

static const uint8_t desc_hid_report[] = {
    HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP ),
    HID_USAGE      ( HID_USAGE_DESKTOP_MULTI_AXIS_CONTROLLER ),
    HID_COLLECTION ( HID_COLLECTION_APPLICATION ),
        HID_COLLECTION ( HID_COLLECTION_PHYSICAL ),
            HID_REPORT_ID      ( 1 )
            HID_LOGICAL_MIN_N  ( -HID_REPORT_LOGICAL_RANGE, 2 ),
            HID_LOGICAL_MAX_N  ( HID_REPORT_LOGICAL_RANGE, 2 ),
            HID_PHYSICAL_MIN_N ( -HID_REPORT_PHYSICAL_RANGE, 2 ),
            HID_PHYSICAL_MAX_N ( HID_REPORT_PHYSICAL_RANGE, 2 ),
            HID_USAGE          ( HID_USAGE_DESKTOP_X ),
            HID_USAGE          ( HID_USAGE_DESKTOP_Y ),
            HID_USAGE          ( HID_USAGE_DESKTOP_Z ),
            HID_REPORT_SIZE    ( 16 ),
            HID_REPORT_COUNT   ( 3 ),
            HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),
        HID_COLLECTION_END,
        HID_COLLECTION ( HID_COLLECTION_PHYSICAL ),
            HID_REPORT_ID      ( 2 )
            HID_LOGICAL_MIN_N  ( -HID_REPORT_LOGICAL_RANGE, 2 ),
            HID_LOGICAL_MAX_N  ( HID_REPORT_LOGICAL_RANGE, 2 ),
            HID_PHYSICAL_MIN_N ( -HID_REPORT_PHYSICAL_RANGE, 2 ),
            HID_PHYSICAL_MAX_N ( HID_REPORT_PHYSICAL_RANGE, 2 ),
            HID_USAGE          ( HID_USAGE_DESKTOP_RX ),
            HID_USAGE          ( HID_USAGE_DESKTOP_RY ),
            HID_USAGE          ( HID_USAGE_DESKTOP_RZ ),
            HID_REPORT_SIZE    ( 16 ),
            HID_REPORT_COUNT   ( 3 ),
            HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),
        HID_COLLECTION_END,
        HID_COLLECTION ( HID_COLLECTION_PHYSICAL ),
            HID_REPORT_ID      ( 3 )
            HID_LOGICAL_MIN    ( 0 ),
            HID_LOGICAL_MAX    ( 1 ),
            HID_REPORT_SIZE    ( 1 ),
            HID_REPORT_COUNT   ( HID_MAX_BUTTONS ),
            HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON ),
            HID_USAGE_MIN      ( 1 ),
            HID_USAGE_MAX      ( HID_MAX_BUTTONS ),
            HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),
        HID_COLLECTION_END,
    HID_COLLECTION_END
};

Tlv493d mag;
SimpleKalmanFilter xFilter( 1, 1, 0.2 );
SimpleKalmanFilter yFilter( 1, 1, 0.2 );
SimpleKalmanFilter zFilter( 1, 1, 0.2 );
OneButton button1( SPACEMOUSE_PIN_BUTTON1 );
OneButton button2( SPACEMOUSE_PIN_BUTTON2 );
RP2040_PWM led1Pwm( SPACEMOUSE_PIN_LED1, PWM_FREQ, 0, false );
RP2040_PWM led2Pwm( SPACEMOUSE_PIN_LED2, PWM_FREQ, 0, false );
Adafruit_NeoPixel rgbLed(3, SPACEMOUSE_PIN_RGB, NEO_GRB + NEO_KHZ800); // 1 LED
// Adafruit_NeoPixel rgbLed(1, SPACEMOUSE_PIN_RGB, NEO_GRB + NEO_KHZ800);
Adafruit_USBD_HID usb_hid;
Tween::Timeline ledTimeline;
float nextLedLevel = 0;
uint8_t buttonsData[ HID_MAX_BUTTONS / 8 ] = {};
bool freshButtonsData = false;
float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;
float rxCurrent = 0, ryCurrent = 0, rzCurrent = 0;

bool isOrbit = false;
bool isOrbit_defaultstate = false;
bool button1Held = false;
// when switching between XY-mode and Z-mode, require values to pass through zero
// to avoid abrupt jumps. When a desired mode change is detected we enter a
// transient state (isTransitioning) and wait until sensor outputs are all
// zero (after deadzone compensation) before committing the mode change.
// Magnetometer axis mode state machine (orthogonal to isOrbit)
enum MagnetoState {
    MAG_IDLE,
    MAG_XY_MODE,
    MAG_Z_MODE
};

// current magnetometer mode and zero-cross flag
static MagnetoState magState = MAG_IDLE;
static bool magZeroCrossed = true; // true when sensor has passed through zero (all axes == 0)

enum Axis {
    x,
    y,
    z,
    rx,
    ry,
    rz
};

void magnetometerSetup();
void setLeds( float dutyCycle );
void setButton( uint8_t bit, bool on );

void setup()
{
    TinyUSBDevice.setID( 0x256f, 0xc631 );
    TinyUSBDevice.setLanguageDescriptor( 0x0409 );
    TinyUSBDevice.setManufacturerDescriptor( "3Dconnexion" );
    TinyUSBDevice.setProductDescriptor( "SpaceMouse Pro Wireless (cabled)" );

    usb_hid.setPollInterval( HID_POLL_INTERVAL );
    usb_hid.setBootProtocol( HID_ITF_PROTOCOL_NONE );
    usb_hid.setReportDescriptor( desc_hid_report, sizeof( desc_hid_report ));
    usb_hid.setStringDescriptor( "SpaceMouse Pro Wireless (cabled)" );
    usb_hid.begin();

    rgbLed.begin();
    rgbLed.show(); // Initialize LED to off

    while( !TinyUSBDevice.mounted() )
        delay( 1 );

    magnetometerSetup();
    setLeds( 100 );

    // button1 controls isOrbit and its default state:
    //   - Click: sets isOrbit to the opposite of isOrbit_defaultstate, sets button1Held to true
    //   - Double Click: toggles isOrbit_defaultstate
    //   - Idle: resets isOrbit to isOrbit_defaultstate, sets button1Held to false
    // button2 controls setButton based on button1Held:
    //   - Click: setButton(2, true) if button1Held, else setButton(0, true)
    //   - LongPressStart: setButton(3, true) if button1Held, else setButton(1, true)
    //   - Idle: resets all button bits (0, 1, 2, 3) to false


    button1.attachClick( [] {
        isOrbit = !isOrbit_defaultstate;
    } );

    button1.attachDoubleClick( [] {
        isOrbit_defaultstate = !isOrbit_defaultstate;
    } );

    button1.attachLongPressStart( [] {
        isOrbit = !isOrbit_defaultstate;
        button1Held = true;
    } );

    button1.attachLongPressStop( [] {
        isOrbit = isOrbit_defaultstate;
        button1Held = false;
    } );

    button1.attachIdle( [] {
        isOrbit = isOrbit_defaultstate;
        button1Held = false;
    } );

    button2.attachClick( [] {
        if (button1Held)
            setButton( 2, true );
        else
            setButton( 0, true );
    } );

    button2.attachLongPressStart( [] {
        if (button1Held)
            setButton( 3, true );
        else
            setButton( 1, true );
    } );

    button2.attachIdle( [] {
        setButton( 0, false );
        setButton( 1, false );
        setButton( 2, false );
        setButton( 3, false );
    } );

    button1.setClickMs( 100 );
    button1.setPressMs( 250 );
    button1.setIdleMs( 100 );
    button2.setClickMs( 100 );
    button2.setPressMs( 250 );
    button2.setIdleMs( 100 );

    ledTimeline.mode( Tween::Mode::REPEAT_TL );

    ledTimeline.add( nextLedLevel )
        .init( 100 )
        .then<Ease::SineInOut>( 20, 1000 )
        .hold( 500 )
        .then<Ease::SineInOut>( 100, 1000 )
        .hold( 500 );
}

void setRgbLed(float xCurrent, float yCurrent, float zCurrent, float rxCurrent, float ryCurrent, float rzCurrent, bool isOrbit)
{
    if (isOrbit) {
        uint8_t brightnessRX = static_cast<uint8_t>(min(255,abs(map(static_cast<long>(rxCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -100, 100)) * 2.55f / 2));
        uint8_t brightnessRY = static_cast<uint8_t>(min(255,abs(map(static_cast<long>(ryCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -100, 100)) * 2.55f / 2));
        uint8_t brightnessRZ = static_cast<uint8_t>(min(255,abs(map(static_cast<long>(rzCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_Z_RANGE, MAGNETOMETER_INPUT_Z_RANGE, -100, 100)) * 2.55f / 2));
        // uint8_t brightnessRZ = 0;

        rgbLed.setPixelColor(0, rgbLed.Color(brightnessRY + brightnessRZ, brightnessRX + brightnessRZ, brightnessRX + brightnessRY));
        rgbLed.setPixelColor(1, rgbLed.Color(brightnessRY + brightnessRZ, brightnessRX + brightnessRZ, brightnessRX + brightnessRY));
        rgbLed.setPixelColor(2, rgbLed.Color(brightnessRY + brightnessRZ, brightnessRX + brightnessRZ, brightnessRX + brightnessRY));

    } else {
        uint8_t brightnessX = static_cast<uint8_t>(min(255,abs(map(static_cast<long>(xCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -100, 100)) * 2.55f));
        uint8_t brightnessY = static_cast<uint8_t>(min(255,abs(map(static_cast<long>(yCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -100, 100)) * 2.55f));
        uint8_t brightnessZ = static_cast<uint8_t>(min(255,abs(map(static_cast<long>(zCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_Z_RANGE, MAGNETOMETER_INPUT_Z_RANGE, -100, 100)) * 2.55f));
        
        rgbLed.setPixelColor(0, rgbLed.Color(brightnessX, brightnessY, brightnessZ));
        rgbLed.setPixelColor(1, rgbLed.Color(brightnessX, brightnessY, brightnessZ));
        rgbLed.setPixelColor(2, rgbLed.Color(brightnessX, brightnessY, brightnessZ));
    }
    rgbLed.show();
}

void setRgbLedXYZ(float xCurrent, float yCurrent, float zCurrent, bool isOrbit)
{
    float dutyCycleX = abs( map( static_cast<long>( xCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -100, 100 ));
    float dutyCycleY = abs( map( static_cast<long>( yCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -100, 100 ));
    float dutyCycle = abs( map( static_cast<long>( zCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_Z_RANGE, MAGNETOMETER_INPUT_Z_RANGE, -100, 100 ));
    
    uint8_t brightnessX = static_cast<uint8_t>(dutyCycleX * 2.55f);
    uint8_t brightnessY = static_cast<uint8_t>(dutyCycleY * 2.55f);
    uint8_t brightnessZ = static_cast<uint8_t>(dutyCycle * 2.55f);
    
    if (isOrbit) {
        brightnessX  = brightnessX/2;
        brightnessY  = brightnessY/2;
        brightnessZ  = brightnessZ/2;

        // uint8_t brightnessRX = static_cast<uint8_t>(brightnessX + brightnessZ);
        // uint8_t brightnessRY = static_cast<uint8_t>(brightnessY + brightnessX);
        // uint8_t brightnessRZ = static_cast<uint8_t>(brightnessY + brightnessZ);
        uint8_t brightnessRX = static_cast<uint8_t>(brightnessX + brightnessZ);
        uint8_t brightnessRY = static_cast<uint8_t>(brightnessY + brightnessZ);
        uint8_t brightnessRZ = static_cast<uint8_t>(brightnessY + brightnessX);

        // mix the colors
        // rgbLed.setPixelColor(0, rgbLed.Color(brightnessX + brightnessZ, brightnessY + brightnessX, brightnessY + brightnessZ));
        // rgbLed.setPixelColor(1, rgbLed.Color(brightnessX + brightnessZ, brightnessY + brightnessX, brightnessY + brightnessZ));
        // rgbLed.setPixelColor(2, rgbLed.Color(brightnessX + brightnessZ, brightnessY + brightnessX, brightnessY + brightnessZ));

        rgbLed.setPixelColor(0, rgbLed.Color(brightnessRX, brightnessRY, brightnessRZ)); // Red + Green
        rgbLed.setPixelColor(1, rgbLed.Color(brightnessRX, brightnessRY, brightnessRZ)); // Red + Green
        rgbLed.setPixelColor(2, rgbLed.Color(brightnessRX, brightnessRY, brightnessRZ)); // Red + Green
    } else {
        rgbLed.setPixelColor(0, rgbLed.Color(brightnessX, brightnessY, brightnessZ)); // Red + Green
        rgbLed.setPixelColor(1, rgbLed.Color(brightnessX, brightnessY, brightnessZ)); // Red + Green
        rgbLed.setPixelColor(2, rgbLed.Color(brightnessX, brightnessY, brightnessZ)); // Red + Green
    }
    rgbLed.show();
}

void setLeds( float dutyCycle )
{
    led1Pwm.setPWM( led1Pwm.getPin(), PWM_FREQ, dutyCycle );
    led2Pwm.setPWM( led2Pwm.getPin(), PWM_FREQ, dutyCycle );
    // setRgbLed(dutyCycle); // Mirror behaviour for RGB LED
}

void setButton( uint8_t bit, bool on )
{
    if( on )
        buttonsData[0] |= 1 << bit;
    else
        buttonsData[0] &= ~( 1 << bit );

    freshButtonsData = true;
}

void magnetometerSetup()
{
    static constexpr int CALIBRATION_SAMPLES = 300;

    mag.begin( Wire1 );
    mag.setAccessMode( mag.MASTERCONTROLLEDMODE );
    mag.disableTemp();

    // crude offset calibration on first boot
    for( int i = 0; i < CALIBRATION_SAMPLES; i++ ) {

        delay( mag.getMeasurementDelay() );
        mag.updateData();

        xOffset += mag.getX();
        yOffset += mag.getY();
        zOffset += mag.getZ();

        if(( i % 20 ) == 0 )
            setLeds( 100 );
        else if(( i % 10 ) == 0 )
            setLeds( 50 );
    }

    xOffset /= CALIBRATION_SAMPLES;
    yOffset /= CALIBRATION_SAMPLES;
    zOffset /= CALIBRATION_SAMPLES;
}

void readMagnetometer()
{
    float absZ;
    float zCurrent_pre;

    mag.updateData();

    // local helper: compensate deadzone so dynamic range is preserved
    auto compensateThreshold = [](float value, float threshold, float axisMax) -> float {
        float absV = fabsf(value);
        if (absV <= threshold) return 0.0f;
        if (axisMax <= threshold) return value; // defensive fallback
        // remap [threshold .. axisMax] -> [0 .. axisMax]
        // float scaled = (absV - threshold) / (axisMax - threshold) * axisMax;
        float scaled = (absV - threshold);
        return (value < 0.0f) ? -scaled : scaled;
    };

    // estimated sensor values (same as before)
    xCurrent = xFilter.updateEstimate( mag.getX() - xOffset );
    yCurrent = yFilter.updateEstimate( mag.getY() - yOffset );
    zCurrent_pre = zFilter.updateEstimate( mag.getZ() - zOffset );

    // compute axis maxima in the same units as xCurrent/yCurrent/zCurrent
    float maxXY = MAGNETOMETER_INPUT_RANGE / static_cast<float>(MAGNETOMETER_SENSITIVITY);
    float maxZ  = MAGNETOMETER_INPUT_Z_RANGE / static_cast<float>(MAGNETOMETER_SENSITIVITY);

    // apply deadzone compensation instead of hard-clipping to zero
    xCurrent = compensateThreshold(xCurrent, MAGNETOMETER_XY_THRESHOLD, maxXY);
    yCurrent = compensateThreshold(yCurrent, MAGNETOMETER_XY_THRESHOLD, maxXY);
    zCurrent = compensateThreshold(zCurrent_pre, MAGNETOMETER_Z_THRESHOLD, maxZ);
    absZ = fabsf( zCurrent_pre );

/*
    Serial.print( xCurrent );
    Serial.print( " " );
    Serial.print( yCurrent );
    Serial.print( " " );
    Serial.print( zCurrent );
    Serial.print( "  " );
    Serial.println( isOrbit );
*/

    // -----------------------------
    // Magnetometer mode state machine
    // - MAG_IDLE: no sustained mode selected (default)
    // - MAG_XY_MODE: use x/y as translation axes
    // - MAG_Z_MODE: use z as translation axis
    // Transition rules:
    // - State transitions between XY and Z only allowed when sensor values pass through zero
    // - isOrbit remains orthogonal (separate)
    // -----------------------------

    // Determine whether current sensor reads are effectively zero
    bool xyZero = (xCurrent == 0.0f && yCurrent == 0.0f);
    bool zZero = (zCurrent == 0.0f);

    // Update zero-cross flag when all axes are zero
    if (xyZero && zZero) {
        magZeroCrossed = true;
    }

    // Decide desired mode from the current readings (priority: XY if x/y active, otherwise Z)
    MagnetoState desiredState = MAG_IDLE;
    if (!xyZero) desiredState = MAG_XY_MODE;
    else if (!zZero) desiredState = MAG_Z_MODE;

    // Only allow switching between XY and Z modes if we've seen a zero crossing
    if (magState != desiredState) {
        if (magState == MAG_IDLE) {
            // from idle, adopt the desired state
            magState = desiredState;
            magZeroCrossed = false;
        } else if (magZeroCrossed) {
            // allow transition after zero cross
            magState = desiredState;
            magZeroCrossed = false;
        } else {
            // keep previous state until zero crossing
            desiredState = magState;
        }
    }
    switch (magState) {
        case MAG_IDLE:
            // do nothing
            break;
        case MAG_XY_MODE:
            // clamp zCurrent to zero
            zCurrent = 0.0f;
            absZ = 0.0f;
            break;
        case MAG_Z_MODE:
            // zero x/y
            xCurrent = 0.0f;
            yCurrent = 0.0f;
            break;
    }

    // Route outputs according to the active mode and isOrbit
    if (isOrbit) {
        // orbit maps to rotations
        ryCurrent = xCurrent;
        rxCurrent = yCurrent;
        if (absZ) {
            rxCurrent = 0;
            ryCurrent = 0;
            rzCurrent = zCurrent;
        } else {
            rzCurrent = 0;
        }
        xCurrent = 0;
        yCurrent = 0;
        zCurrent = 0;
    } else {
        // translation mapping depends on magState
        rxCurrent = 0;
        ryCurrent = 0;
        rzCurrent = 0;

        if (magState == MAG_XY_MODE) {
            // use x/y; clamp z
            if (absZ) zCurrent = 0;
        } else if (magState == MAG_Z_MODE) {
            // use z; zero x/y
            xCurrent = 0;
            yCurrent = 0;
        } else { // MAG_IDLE
            xCurrent = 0;
            yCurrent = 0;
            zCurrent = 0;
        }
    }

    setRgbLed( xCurrent, yCurrent, zCurrent, rxCurrent, ryCurrent, rzCurrent, isOrbit );
}

void sendHidReports()
{
    static unsigned long lastActivityMillis = 0;
    bool activity = false;
    auto x = static_cast<int16_t>( map( static_cast<long>( xCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE ));
    auto y = -static_cast<int16_t>( map( static_cast<long>( yCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE ));
    auto z = -static_cast<int16_t>( map( static_cast<long>( zCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_Z_RANGE, MAGNETOMETER_INPUT_Z_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE ));
    uint8_t trans[] = {
        static_cast<uint8_t>( x & 0xFF ), static_cast<uint8_t>( x >> 8 ),
        static_cast<uint8_t>( y & 0xFF ), static_cast<uint8_t>( y >> 8 ),
        static_cast<uint8_t>( z & 0xFF ), static_cast<uint8_t>( z >> 8 )
    };
    auto rx = static_cast<int16_t>( map( static_cast<long>( rxCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE ));
    auto ry = -static_cast<int16_t>( map( static_cast<long>( ryCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE ));
    auto rz = -static_cast<int16_t>( map( static_cast<long>( rzCurrent * MAGNETOMETER_SENSITIVITY ), -MAGNETOMETER_INPUT_Z_RANGE, MAGNETOMETER_INPUT_Z_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE ));
    uint8_t rot[] = {
        static_cast<uint8_t>( rx & 0xFF ), static_cast<uint8_t>( rx >> 8 ),
        static_cast<uint8_t>( ry & 0xFF ), static_cast<uint8_t>( ry >> 8 ),
        static_cast<uint8_t>( rz & 0xFF ), static_cast<uint8_t>( rz >> 8 )
    };

    usb_hid.sendReport( 1, trans, sizeof( trans ));
    delay( HID_POLL_INTERVAL );
    usb_hid.sendReport( 2, rot, sizeof( rot ));

    if( x || y || z )
        activity = true;

    if( freshButtonsData ) {
        delay( HID_POLL_INTERVAL );
        usb_hid.sendReport( 3, buttonsData, sizeof( buttonsData ));
        freshButtonsData = false;
        activity = true;
    }

    if( activity ) {

        if( TinyUSBDevice.suspended() )
            TinyUSBDevice.remoteWakeup();

        lastActivityMillis = millis();

        if( ledTimeline.isRunning() ) {
            ledTimeline.stop();
            setLeds( 100 );
        }

    } else {
        unsigned long dt = millis() - lastActivityMillis;

        if( dt > MAX_INACTIVITY_TIME ) {
            Tween::Timeline fadeOut;

            ledTimeline.stop();

            fadeOut.add( nextLedLevel )
                .init( nextLedLevel )
                .then<Ease::SineOut>( 0, 500 );

            fadeOut.start();

            while( fadeOut.isRunning() ) {
                fadeOut.update();
                setLeds( nextLedLevel );
            }

            sleep_run_from_rosc();
            sleep_goto_dormant_until_pin( SPACEMOUSE_PIN_BUTTON1, true, false );
            rp2040.restart();

        } else if(( dt > LED_FADING_INACTIVITY_TIME ) && !ledTimeline.isRunning() )
            ledTimeline.start();
    }
}

void loop()
{
#ifdef TINYUSB_NEED_POLLING_TASK
    // Manual call tud_task since it isn't called by Core's background
    TinyUSBDevice.task();
#endif

    if( ledTimeline.isRunning() ) {
        ledTimeline.update();
        setLeds( nextLedLevel );
    }

    readMagnetometer();

    if( usb_hid.ready() ) {
        button1.tick();
        button2.tick();
        sendHidReports();
    }

    delay( mag.getMeasurementDelay() - HID_POLL_INTERVAL );
}
