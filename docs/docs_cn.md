
## **[English](../README.MD) | 中文**

默认使用示例为**Arduino**,同时也支持[**nRF5-SDK**](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download)

## 如何使用
### 使用**Arduino IDE**
1. 打开Arduino,打开首选项 -> 添加https://www.adafruit.com/package_adafruit_index.json 到 板安装管理器地址列表
2. 打开板子安装管理器中,等待索引更新完成,选择'Adafruit nRF52 by Adafruit'点击安装
3. 安装完成后,在板子列表中选择'Adafruit Feather nRF52832'
4. 将lib目录中的所有文件夹拷贝到`"C:\User\<YourName>\Documents\Arduino\libraries"`中
5. 打开草图 => 工具 => 端口 ,选择已连接板子的端口,然后点击上传

### 使用**PlatformIO**，直接打开即可,在初次使用会自动下载**Adafruit_nRF52_Arduino**

## 注意事项:
1. 需要使用**lib**目录中的文件,它包括:
   - `Adafruit ST7735 and ST7789 Library` (已经对源文件进行更改,使用主分支代码将会导致显示不正常 => 主要更改BGR -> RGB , 更改偏移坐标)
   - `Adafruit GFX Library`   (未修改,可以使用主分支)
   - `PCF8563_Library`        (未修改,可以使用主分支)
   - `SerialFlash`            (已修改 => 主要添加SoftSPI头文件,兼容SoftSPI)
   - `SoftSPI `               (单独无分支)
   - `SparkFun LSM9DS1 IMU`   (未修改,可以使用主分支)

2. 默认使用[Adafruit_nRF52_Arduino](https://github.com/adafruit/Adafruit_nRF52_Arduino),所以出厂已经烧录[Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader),如果使用**nRF5-SDK**对板子编程 将会丢失原先Bootloader

3. 如果需要使用**nRF5-SDK**进行编程,请点击链接下载[**nRF5-SDK**](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download)

4. **Adafruit_nRF52_Arduino**中不支持**NFC**功能,请用[**nRF5-SDK**](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download)进行编程