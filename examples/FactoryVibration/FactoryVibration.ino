#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <pcf8563.h>
#include <SoftSPI.h>
#include <SerialFlash.h>
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include <Adafruit_DRV2605.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include "nrf_rtc.h"

#include "charge.h"

// If you need to turn on DRV2605, uncomment this line
// #define ENABLE_DRV2605


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


#define MOTOR_PIN       28



Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
PCF8563_Class rtc;
MPU9250_DMP imu;

#ifdef ENABLE_DRV2605
Adafruit_DRV2605 drv;
#endif

uint8_t omm = 99;
uint8_t odate = 100;
uint32_t targetTime = 0;       // for next 1 second timeout
uint8_t date_now, year_now, month_now, hh, mm, ss;
bool initial = 1;
uint8_t func_select = 0;
bool pressed = false;
bool charge_indication = false;
bool found_imu = false, found_rtc = false, rtcIrq = false;
bool sleep_enable = false;
volatile bool event = false;

uint32_t vbat_pin = PIN_VBAT;
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


void setSleepEnabled(bool enabled);

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

bool setupIMU()
{

    // Call imu.begin() to verify communication with and
    // initialize the MPU-9250 to it's default values.
    // Most functions return an error code - INV_SUCCESS (0)
    // indicates the IMU was present and successfully set up
    if (imu.begin() != INV_SUCCESS) {
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
        return false;
    }


    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Enable all sensors:
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(2000); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(2); // Set accel to +/-2g
    // Note: the MPU-9250's magnetometer FSR is set at
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    imu.setLPF(5); // Set LPF corner frequency to 5Hz

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(10); // Set sample rate to 10Hz

    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    imu.setCompassSampleRate(10); // Set mag rate to 10Hz
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

    Serial.println("Start...");

    pinMode(TFT_BL, OUTPUT);
    pinMode(TOUCH_PW, OUTPUT);
    pinMode(TP_PIN_PIN, INPUT);

    digitalWrite(TFT_BL, HIGH);

    digitalWrite(TOUCH_PW, HIGH);

    setupDisplay();


    Wire.begin();

#ifdef ENABLE_DRV2605
    Wire.beginTransmission(0x5A);
    if (Wire.endTransmission() != 0) {
        Serial.println("No detected drv2605 !!!");
        tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        tft.println("No detected drv2605 !!!");
        delay(5000);
        NVIC_SystemReset();
    }

    drv.begin();

    drv.selectLibrary(1);

    // I2C trigger by sending 'go' command
    // default, internal trigger when sending GO command
    drv.setMode(DRV2605_MODE_INTTRIG);
    tft.println("DRV2605 Started!!!");
#else
    /*If DRV2605 is not defined, the normal vibration module will be turned on by default*/
    pinMode(MOTOR_PIN, OUTPUT);
#endif



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
        date_now = datetime.day;
        month_now = datetime.month;
        year_now = datetime.year;
        hh = datetime.hour;
        mm = datetime.minute;
        ss = datetime.second;
        targetTime = millis() + 1000;
        /*
        if (ss == 0 || initial) {
            initial = 0;
            tft.setTextSize(1);
            tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
            tft.setCursor (8, 60);
            tft.print(__DATE__); // This uses the standard ADAFruit small font
        }*/
        if (odate != date_now) {
            tft.setTextSize(1);
            tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
            tft.setCursor (8, 60);
            tft.printf("%d.%d", month_now, date_now);
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
    // dataReady() checks to see if new accel/gyro data
    // is available. It will return a boolean true or false
    // (New magnetometer data cannot be checked, as the library
    //  runs that sensor in single-conversion mode.)
    if ( imu.dataReady() ) {
        // Call update() to update the imu objects sensor data.
        // You can specify which sensors to update by combining
        // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
        // UPDATE_TEMPERATURE.
        // (The update function defaults to accel, gyro, compass,
        //  so you don't have to specify these values.)
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

        tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextSize(1);
        tft.setCursor(0, 0);
        tft.print("ACC x:"); tft.print( imu.calcAccel(imu.ax)); tft.print(" y:"); tft.print(imu.calcAccel(imu.ay)); tft.print(" z:"); tft.println(imu.calcAccel(imu.az));
        tft.print("GYR x:"); tft.print( imu.calcGyro(imu.gx)); tft.print(" y:"); tft.print(imu.calcGyro(imu.gy)); tft.print(" z:"); tft.println(imu.calcGyro(imu.gz));
        tft.print("MAG x:"); tft.print( imu.calcMag(imu.mx)); tft.print(" y:"); tft.print(imu.calcMag(imu.my)); tft.print(" z:"); tft.println(imu.calcMag(imu.mz));
    }
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

#ifdef ENABLE_DRV2605
                drv.setWaveform(0, 54);  // play effect
                drv.setWaveform(1, 0);       // end waveform
                // play the effect!
                drv.go();
#else
                digitalWrite(MOTOR_PIN, HIGH);
                delay(100);
                digitalWrite(MOTOR_PIN, LOW);
#endif
                func_select = func_select + 1 > 2 ? 0 : func_select + 1;
            } else {
                Serial.println("Trun on");
                func_select = 0;
                digitalWrite(TFT_BL, HIGH);
                // imu.sleepGyro(false);
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
            // imu.sleepGyro(true);
            tft.sendCommand(ST77XX_SLPIN);
            digitalWrite(TFT_BL, LOW);
            setSleepEnabled(true);
            SerialFlash.sleep();

            sleep_enable = true;

            sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

            attachInterrupt(TP_PIN_PIN, []() {
                if (sleep_enable) {
                    sleep_enable = false;
                }
            }, RISING);

            NRF_UARTE0->ENABLE = 0; //disable UART
            NRF_SAADC ->ENABLE = 0; //disable ADC
            NRF_PWM0  ->ENABLE = 0; //disable all pwm instance
            NRF_PWM1  ->ENABLE = 0;
            NRF_PWM2  ->ENABLE = 0;
            NRF_TWIM1 ->ENABLE = 0; //disable TWI Master
            NRF_TWIS1 ->ENABLE = 0; //disable TWI Slave
            NRF_SPI0 -> ENABLE = 0; //disable SPI
            NRF_SPI1 -> ENABLE = 0; //disable SPI
            NRF_SPI2 -> ENABLE = 0; //disable SPI

            while (sleep_enable) {
                __WFE();
                __WFI();
            }

            detachInterrupt(TP_PIN_PIN);

            NRF_UARTE0->ENABLE =    UARTE_ENABLE_ENABLE_Enabled;                               //disable UART
            NRF_SPIM2->ENABLE =     (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);;      //disable SPI
            NRF_TWIM0->ENABLE  =    (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);;  //disable TWI Master

            tft.sendCommand(ST77XX_SLPOUT);
            tft.sendCommand(ST77XX_DISPON);
            digitalWrite(TFT_BL, HIGH);
            setSleepEnabled(false);
            SerialFlash.wakeup();
            func_select = 0;
        }
        break;
    default:
        break;
    }
}

#define MPU9250_ADDRESS     0x69
#define PWR1_SLEEP_BIT      6
#define MPU9250_PWR_MGMT_1  0x6B

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

void writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t b = readByte(devAddr, regAddr);
    printf("rb:%x\n", b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    printf("wb:%x\n", b);
    writeByte(devAddr, regAddr, b);
}

void setSleepEnabled(bool enabled)
{
    writeBit(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, PWR1_SLEEP_BIT, enabled);
}
