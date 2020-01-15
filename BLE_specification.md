# Nano 33 BLE Service

BLE specification for the Bluetooth LE service implementation for the [Arduino Nano 33 BLE](https://www.arduino.cc/en/Guide/NANO33BLE) and
[Nano 33 BLE Sense](https://www.arduino.cc/en/Guide/NANO33BLESense)..

## Generic Access Profile (GAP)

### Advertisement

-   Local name:
    -   Nano 33 BLE microcontrollers: Nano33BLE-<last 4 characters of BT address>
    -   Nano 33 BLE Sense microcontrollers: Nano33BLESense-<last 4 characters of BT address>
-   Service UUID: e905de3e-0000-44de-92c4-bb6e04fb0212

## Generic Attribute Profile (GATT)

### Services

-   UUID: e905de3e-0000-44de-92c4-bb6e04fb0212

### Characteristics

Version Characteristic

-   UUID: e905de3e-1001-44de-92c4-bb6e04fb0212
-   Properties: read
-   Value size: 4 bytes
-   Data format: 32-bit unsigned integer (little endian)
-   Description: Version of firmware

Ambient Light Characteristic (BLE Sense only)

-   UUID: e905de3e-2001-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 2 bytes
-   Data format: 16-bit unsigned integer (little endian)
-   Description: Ambient light level, 0 => dark

Color Characteristic (BLE Sense only)

-   UUID: e905de3e-2002-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 6 bytes
-   Data format: array of 16-bit unsigned integers (little endian), RGB
-   Description: RGB color sensor values

Proximity Characteristic (BLE Sense only)

-   UUID: e905de3e-2003-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 1 byte
-   Data format: 8-bit
-   Description: Proximity sensor value, 0 => close, 255 far

Gesture Characteristic (BLE Sense only)

-   UUID: e905de3e-2004-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 1 byte
-   Data format: 8-bit
-   Description: Gesture detected, NONE = -1, UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3

Acceleration Characteristic

-   UUID: e905de3e-3001-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 12 bytes
-   Data format: Array of 3 x 32-bit IEEE floats (little endian)
-   Description: X, Y, Z acceleration values in G's

Gyroscope Characteristic

-   UUID: e905de3e-3002-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 12 bytes
-   Data format: Array of 3 x 32-bit IEEE floats (little endian)
-   Description: X, Y, Z gyroscope values in degrees per second

Magnetic Field Characteristic

-   UUID: e905de3e-3003-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 12 bytes
-   Data format: Array of 3 x 32-bit IEEE floats (little endian)
-   Description: X, Y, Z magnetic fields values in uT

Orientation Characteristic

-   UUID: e905de3e-3004-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 12 bytes
-   Data format: Array of 3 x 32-bit IEEE floats (little endian)
-   Description: heading, pitch, roll values in radians

Pressure Characteristic (BLE Sense only)

-   UUID: e905de3e-4001-44de-92c4-bb6e04fb0212
-   Properties: read
-   Value size: 4 bytes
-   Data format: 32-bit IEEE floats (little endian)
-   Description: Pressure sensor value in kPA

Temperature Characteristic (BLE Sense only)

-   UUID: e905de3e-4002-44de-92c4-bb6e04fb0212
-   Properties: read
-   Value size: 4 bytes
-   Data format: 32-bit IEEE floats (little endian)
-   Description: Temperature sensor value Celcius

Humidity Characteristic (BLE Sense only)

-   UUID: e905de3e-4003-44de-92c4-bb6e04fb0212
-   Properties: read
-   Value size: 4 bytes
-   Data format: 32-bit IEEE floats (little endian)
-   Description: Humidity sensor value %

Microphone Value Characteristic (BLE Sense only)

-   UUID: e905de3e-5001-44de-92c4-bb6e04fb0212
-   Properties: notify
-   Value size: 2 bytes
-   Data format: 16-bit signed (little endian)
-   Description: Mic level (RMS)

RGB LED Characteristic (BLE Sense only)

-   UUID: e905de3e-6001-44de-92c4-bb6e04fb0212
-   Properties: write
-   Value size: 3 bytes
-   Data format: Array of unsigned 8-bits (little endian)
-   Description: RGB led value, 0 => off, 255 => on
