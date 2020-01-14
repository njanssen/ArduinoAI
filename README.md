# arduino-nano-33-ble

Bluetooth service implementation for the [Arduino Nano 33 BLE](https://www.arduino.cc/en/Guide/NANO33BLE) and 
[Arduino Nano 33 BLE Sense](https://www.arduino.cc/en/Guide/NANO33BLESense).

## What does this sketch do?

Supported inertial measurement unit (IMU) sensors found on both the Nano 33 BLE and Nano 33 BLE Sense:

-   Accelerometer (LSM9DS1)
-   Gyroscope (LSM9DS1)
-   Magnetometer (LSM9DS1)

Supported sensors found on the Nano 33 BLE Sense only:

-   Digital microphone (MP34DT05)
-   Temperature (HTS221)
-   Relative humidity (HTS221)
-   Pressure (LPS22HB)
-	Gesture sensor (APDS9960)
-	Ambient light (APDS9960)
-	Color (APDS9960)
-	Proximity (APDS9960)

The library provides the following filters and algorithms related to the IMU sensors:

-   Arduino's official [Madgwick AHRS sensor fusion algorithm implementation](https://github.com/arduino-libraries/MadgwickAHRS) for orientation (heading, pitch, and roll)

## Extras

-   This project is a fork of the official [Arduino and AI](https://github.com/arduino/ArduinoAI) repository.
-   See the [License](LICENSE) file for license rights and limitations (GPL).
-   Pull Requests are welcome!
