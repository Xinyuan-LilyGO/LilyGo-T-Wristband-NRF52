<h1 align = "center">🌟LilyGo T-Wristband-NRF52🌟</h1>

## **English | [中文](docs/docs_cn.md)**

The default example is **Arduino**, which also supports [**nRF5-SDK**](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download)

## How to use
### Using Arduino IDE
1. Download and install the CH340 driver 
   - [CH340 Drivers](http://www.wch-ic.com/search?q=ch340&t=downloads)
1. Open Arduino, open preferences-> add https://www.adafruit.com/package_adafruit_index.json to the board installation manager address list
2. Open the board installation manager, wait for the index update to complete, select 'Adafruit nRF52 by Adafruit' and click install
3. After the installation is complete, select 'Adafruit Feather nRF52832' in the board list
4. Copy all the folders in the lib directory to `"C:\User\<YourName>\Documents\Arduino\libraries"`
5. Open the sketch => tool => port, select the port of the connected board, and click upload

### Use PlatformIO , just open it, and it will automatically download Adafruit_nRF52_Arduino for the first use

## Precautions:
1. Need to use the files in the **lib** directory, it includes:
- `Adafruit ST7735 and ST7789 Library` (the source file has been changed, using the main branch code will result in abnormal display => main change BGR -> RGB, change offset coordinates)
- `Adafruit GFX Library` (unmodified, you can use the main branch)
- `PCF8563_Library` (unmodified, you can use the main branch)
- `SerialFlash` (modified => SoftSPI header files are added, compatible with SoftSPI)
- `SoftSPI` (no separate branch)
- `SparkFun_MPU-9250-DMP_Arduino_Library` (unmodified, you can use the main branch)

2. Use [Adafruit_nRF52_Arduino](https://github.com/adafruit/Adafruit_nRF52_Arduino) by default, so [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader) has been burned in the factory, if you use **nRF5-SDK** Programming the board will lose the original bootloader

3. If you need to use **nRF5-SDK** for programming, please click the link to download [**nRF5-SDK**](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download)

4. **NFC Function** is not supported in **Adafruit_nRF52_Arduino**, please use [**nRF5-SDK**](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download) for programming