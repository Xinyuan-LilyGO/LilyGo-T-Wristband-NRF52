#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <pcf8563.h>
#include <SoftSPI.h>
#include <SerialFlash.h>
#include "charge.h"
#include <Wire.h>
// Include the SparkFunLSM9DS1 library and its dependencies.
#include <SparkFunLSM9DS1.h>

#define SPI_FLASH_MOSI  16
#define SPI_FLASH_MISO  11
#define SPI_FLASH_SCLK  17
#define SPI_FLASH_CS    30

#define CHARGE_PIN      27

#define TFT_MOSI        13
#define TFT_MISO        14
#define TFT_SCLK        12
#define TFT_DC          2
#define TFT_RST         3
#define TFT_CS          4
#define TFT_BL          7

#define RTC_INT_PIN     15

#define RDYM_PIN        19
#define INT2_PIN_DRDY   22
#define INT1_PIN_THS    23
#define INTM_PIN_THS    24

#define TP_PIN_PIN      5
#define TOUCH_PW        29

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
PCF8563_Class rtc;
LSM9DS1 imu; // Create an LSM9DS1 object to use from here on.

uint8_t omm = 99;
uint32_t targetTime = 0;       // for next 1 second timeout
uint8_t hh, mm, ss ;
bool initial = 1;
uint8_t func_select = 0;
bool pressed = false;
bool charge_indication = false;
bool found_imu = false, found_rtc = false, rtcIrq = false;

uint32_t vbat_pin = PIN_VBAT;
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

float readVBAT(void)
{
    float raw;
    // Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);
    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14
    // Let the ADC settle
    delay(1);
    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbat_pin);
    // Set the ADC back to the default settings
    analogReference(AR_DEFAULT);
    analogReadResolution(10);
    // Convert the raw value to compensated mv, taking the resistor-
    // divider into account (providing the actual LIPO voltage)
    // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
    return raw * REAL_VBAT_MV_PER_LSB;
}

// configureIMU sets up our LSM9DS1 interface, sensor scales
// and sample rates.
uint16_t configureIMU()
{
    // gyro.latchInterrupt controls the latching of the
    // gyro and accelerometer interrupts (INT1 and INT2).
    // false = no latching
    imu.settings.gyro.latchInterrupt = false;

    // Set gyroscope scale to +/-245 dps:
    imu.settings.gyro.scale = 245;
    // Set gyroscope (and accel) sample rate to 14.9 Hz
    imu.settings.gyro.sampleRate = 1;
    // Set accelerometer scale to +/-2g
    imu.settings.accel.scale = 2;
    // Set magnetometer scale to +/- 4g
    imu.settings.mag.scale = 4;
    // Set magnetometer sample rate to 0.625 Hz
    imu.settings.mag.sampleRate = 0;

    // Call imu.begin() to initialize the sensor and instill
    // it with our new settings.
    return imu.begin(LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1), Wire); // set addresses and wire port
}

void configureLSM9DS1Interrupts()
{
    /////////////////////////////////////////////
    // Configure INT1 - Gyro & Accel Threshold //
    /////////////////////////////////////////////
    // For more information on setting gyro interrupt, threshold,
    // and configuring the intterup, see the datasheet.
    // We'll configure INT_GEN_CFG_G, INT_GEN_THS_??_G,
    // INT_GEN_DUR_G, and INT1_CTRL.
    // 1. Configure the gyro interrupt generator:
    //  - ZHIE_G: Z-axis high event (more can be or'd together)
    //  - false: and/or (false = OR) (not applicable)
    //  - false: latch interrupt (false = not latched)
    imu.configGyroInt(ZHIE_G, false, false);
    // 2. Configure the gyro threshold
    //   - 500: Threshold (raw value from gyro)
    //   - Z_AXIS: Z-axis threshold
    //   - 10: duration (based on ODR)
    //   - true: wait (wait duration before interrupt goes low)
    imu.configGyroThs(500, Z_AXIS, 10, true);
    // 3. Configure accelerometer interrupt generator:
    //   - XHIE_XL: x-axis high event
    //     More axis events can be or'd together
    //   - false: OR interrupts (N/A, since we only have 1)
    imu.configAccelInt(XHIE_XL, false);
    // 4. Configure accelerometer threshold:
    //   - 20: Threshold (raw value from accel)
    //     Multiply this value by 128 to get threshold value.
    //     (20 = 2600 raw accel value)
    //   - X_AXIS: Write to X-axis threshold
    //   - 10: duration (based on ODR)
    //   - false: wait (wait [duration] before interrupt goes low)
    imu.configAccelThs(20, X_AXIS, 1, false);
    // 5. Configure INT1 - assign it to gyro interrupt
    //   - XG_INT1: Says we're configuring INT1
    //   - INT1_IG_G | INT1_IG_XL: Sets interrupt source to
    //     both gyro interrupt and accel
    //   - INT_ACTIVE_LOW: Sets interrupt to active low.
    //         (Can otherwise be set to INT_ACTIVE_HIGH.)
    //   - INT_PUSH_PULL: Sets interrupt to a push-pull.
    //         (Can otherwise be set to INT_OPEN_DRAIN.)
    imu.configInt(XG_INT1, INT1_IG_G | INT_IG_XL, INT_ACTIVE_LOW, INT_PUSH_PULL);

    ////////////////////////////////////////////////
    // Configure INT2 - Gyro and Accel Data Ready //
    ////////////////////////////////////////////////
    // Configure interrupt 2 to fire whenever new accelerometer
    // or gyroscope data is available.
    // Note XG_INT2 means configuring interrupt 2.
    // INT_DRDY_XL is OR'd with INT_DRDY_G
    imu.configInt(XG_INT2, INT_DRDY_XL | INT_DRDY_G, INT_ACTIVE_LOW, INT_PUSH_PULL);

    //////////////////////////////////////
    // Configure Magnetometer Interrupt //
    //////////////////////////////////////
    // 1. Configure magnetometer interrupt:
    //   - XIEN: axis to be monitored. Can be an or'd combination
    //     of XIEN, YIEN, or ZIEN.
    //   - INT_ACTIVE_LOW: Interrupt goes low when active.
    //   - true: Latch interrupt
    imu.configMagInt(XIEN, INT_ACTIVE_LOW, true);
    // 2. Configure magnetometer threshold.
    //   There's only one threshold value for all 3 mag axes.
    //   This is the raw mag value that must be exceeded to
    //   generate an interrupt.
    imu.configMagThs(10000);

}

bool setupIMU()
{
    // Set up our Arduino pins connected to interrupts.
    // We configured all of these interrupts in the LSM9DS1
    // to be active-low.
    pinMode(INT2_PIN_DRDY, INPUT_PULLUP);
    pinMode(INT1_PIN_THS, INPUT_PULLUP);
    pinMode(INTM_PIN_THS, INPUT_PULLUP);
    // The magnetometer DRDY pin (RDY) is not configurable.
    // It is active high and always turned on.
    pinMode(RDYM_PIN, INPUT);

    Wire.begin();

    // Turn on the IMU with configureIMU() (defined above)
    // check the return status of imu.begin() to make sure
    // it's connected.
    uint16_t status = configureIMU();
    if (status == false) {
        Serial.print("Failed to connect to IMU: 0x");
        Serial.println(status, HEX);
        return false;
    }

    // After turning the IMU on, configure the interrupts:
    configureLSM9DS1Interrupts();

    return true;
}

bool setupDisplay()
{
    // OR use this initializer (uncomment) if using a 0.96" 160x80 TFT:
    tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);
    return true;
}

void setup()
{
    Serial.begin(115200);

    delay(5000);

    Serial.println("Start...");

    pinMode(TFT_BL, OUTPUT);
    pinMode(TOUCH_PW, OUTPUT);
    pinMode(TP_PIN_PIN, INPUT);

    digitalWrite(TFT_BL, HIGH);
    digitalWrite(TOUCH_PW, HIGH);

    setupDisplay();

    found_imu  = setupIMU();
    if (!found_imu) {
        Serial.println("setupIMU fail");
        tft.println("setupIMU fail");
    } else {
        Serial.println("setupIMU pass");
        tft.println("setupIMU pass");
    }

    if (!SerialFlash.begin(SPI_FLASH_CS)) {
        tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        Serial.println("setupSerialFlash fail");
        tft.println("setupSerialFlash fail");
    } else {
        Serial.println("setupSerialFlash pass");
        tft.println("setupSerialFlash pass");
    }

    Wire.beginTransmission(0x51);
    if (Wire.endTransmission() == 0) {
        rtc.begin();
        found_rtc = true;
    } else {
        tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        Serial.println("RTC deteced fail");
        tft.println("RTC deteced fail");
    }

    if (found_rtc) {
        //! RTC Interrupt Test
        pinMode(RTC_INT_PIN, INPUT_PULLUP); //need change to rtc_pin
        attachInterrupt(RTC_INT_PIN, [] {
            rtcIrq = 1;
        }, FALLING);

        rtc.disableAlarm();
        rtc.setDateTime(2019, 4, 7, 9, 5, 57);
        rtc.setAlarmByMinutes(6);
        rtc.enableAlarm();
        for (;;) {
            tft.println(rtc.formatDateTime(PCF_TIMEFORMAT_YYYY_MM_DD_H_M_S));
            if (rtcIrq) {
                rtcIrq = 0;
                detachInterrupt(RTC_INT_PIN);
                rtc.resetAlarm();
                break;
            }
            delay(1000);
        }
        tft.println("RTC Interrupt PASS");
        delay(2000);
        rtc.check();
    }

    pinMode(CHARGE_PIN, INPUT_PULLUP);
    attachInterrupt(CHARGE_PIN, [] {
        charge_indication = true;
    }, CHANGE);

    if (digitalRead(CHARGE_PIN) == LOW) {
        charge_indication = true;
    }

    tft.fillScreen(ST77XX_BLACK);
}


void RTC_Show()
{
    static int16_t x, y;
    if (targetTime < millis()) {
        RTC_Date datetime = rtc.getDateTime();
        Serial.println(millis());
        hh = datetime.hour;
        mm = datetime.minute;
        ss = datetime.second;
        targetTime = millis() + 1000;
        if (ss == 0 || initial) {
            initial = 0;
            tft.setTextSize(1);
            tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
            tft.setCursor (8, 60);
            tft.print(__DATE__); // This uses the standard ADAFruit small font
        }

        float vbat_mv = readVBAT();
        tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
        tft.setCursor (120, 60);
        tft.setTextSize(1);
        tft.print(vbat_mv / 1000.0);
        tft.print("V");
        tft.setTextSize(4);

        // Update digital time
        uint8_t xpos = 19;
        uint8_t ypos = 10;

        if (omm != mm) {
            tft.setTextColor(0x39C4, ST77XX_BLACK);
            tft.setTextSize(4);
            tft.setCursor(xpos, ypos);
            tft.println("88:88");
            tft.setCursor(xpos, ypos);
            tft.setTextColor(0xFBE0, ST77XX_BLACK); // Orange
            omm = mm;
            if (hh < 10) {
                tft.print('0');
            }
            tft.print(hh);
            x = tft.getCursorX();
            y = tft.getCursorY();
            tft.print(':');
            if (mm < 10) {
                tft.print('0');
            }
            tft.print(mm);
        }
        if (ss % 2) {
            tft.setTextColor(0x39C4, ST77XX_BLACK);
            tft.setCursor(x, y);
            tft.print(':');
            tft.setTextColor(0xFBE0, ST77XX_BLACK);
        } else {
            tft.setTextColor(0xFBE0, ST77XX_BLACK);
            tft.setCursor(x, y);
            tft.print(':');
        }
    }
}

void IMU_Show()
{
    // Update the sensor values whenever new data is available
    if ( imu.gyroAvailable() ) {
        // To read from the gyroscope,  first call the
        // readGyro() function. When it exits, it'll update the
        // gx, gy, and gz variables with the most current data.
        imu.readGyro();
    }
    if ( imu.accelAvailable() ) {
        // To read from the accelerometer, first call the
        // readAccel() function. When it exits, it'll update the
        // ax, ay, and az variables with the most current data.
        imu.readAccel();
    }
    if ( imu.magAvailable() ) {
        // To read from the magnetometer, first call the
        // readMag() function. When it exits, it'll update the
        // mx, my, and mz variables with the most current data.
        imu.readMag();
    }
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.print("ACC x:"); tft.print( imu.calcAccel(imu.ax)); tft.print(" y:"); tft.print(imu.calcAccel(imu.ay)); tft.print(" z:"); tft.println(imu.calcAccel(imu.az));
    tft.print("GYR x:"); tft.print( imu.calcGyro(imu.gx)); tft.print(" y:"); tft.print(imu.calcGyro(imu.gy)); tft.print(" z:"); tft.println(imu.calcGyro(imu.gz));
    tft.print("MAG x:"); tft.print( imu.calcMag(imu.mx)); tft.print(" y:"); tft.print(imu.calcMag(imu.my)); tft.print(" z:"); tft.println(imu.calcMag(imu.mz));
    delay(200);
}


void loop()
{
    if (charge_indication) {
        charge_indication = false;
        if (digitalRead(CHARGE_PIN) == LOW) {
            tft.drawRGBBitmap(100, 55, charge, 16, 16);
        } else {
            tft.fillRect(100, 55, 16, 16, ST77XX_BLACK);
        }
    }

    if (digitalRead(TP_PIN_PIN) == HIGH) {
        if (!pressed) {
            if (digitalRead(TFT_BL)) {
                func_select = func_select + 1 > 2 ? 0 : func_select + 1;
            } else {
                func_select = 0;
                digitalWrite(TFT_BL, HIGH);
                imu.sleepGyro(false);
                tft.sendCommand(ST77XX_SLPOUT);
            }
            tft.fillScreen(ST77XX_BLACK);
            initial = 1;
            targetTime = millis() + 1000;
            omm = 99;
            pressed = true;
        }
    } else {
        pressed = false;
    }

    switch (func_select) {
    case 0:
        RTC_Show();
        break;
    case 1:
        IMU_Show();
        break;
    case 2:
        if (digitalRead(TFT_BL)) {
            imu.sleepGyro(true);
            tft.sendCommand(ST77XX_SLPIN);
            digitalWrite(TFT_BL, LOW);
        }
        break;
    default:
        break;
    }
}


