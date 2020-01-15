# arduino-nano-33-ble

Bluetooth service implementation for the [Arduino Nano 33 BLE](https://www.arduino.cc/en/Guide/NANO33BLE) and 
[Nano 33 BLE Sense](https://www.arduino.cc/en/Guide/NANO33BLESense).

## What does this sketch do?

This sketch implements a Bluetooth Low Energy (BLE) service which makes it easy to listen to data from one or more sensors on a Arduino Nano 33 BLE and Nano 33 BLE Sense. The sketch utilizes the on-board sensors and Bluetooth Low Energy connectivity.

See the [BLE Specification](BLE_specification.md) for service implementations details (services and characteristics).

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

The library uses the following filters and algorithms related to the IMU sensors:

-   Arduino's [Madgwick AHRS sensor fusion algorithm implementation](https://github.com/arduino-libraries/MadgwickAHRS) for orientation (heading, pitch, and roll)

## Libraries

The following libraries are compatible with the Bluetooth service implementation provided by this sketch:

- Node.js interface (npm module): [@vliegwerk/arduino-nano-33-ble](https://www.npmjs.com/package/@vliegwerk/arduino-nano-33-ble) 


## Extras

-   The Arduino sketch on this project is a fork of the [Arduino and AI](https://github.com/arduino/ArduinoAI) repository by Arduino.
-   See the [License](LICENSE) file for license rights and limitations (GPL).
-   Pull Requests are welcome!
