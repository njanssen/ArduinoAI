/*
   Bluetooth Service implementation for Arduino Nano 33 BLE and Arduino Nano 33 BLE Sense

   Copyright (C) 2019 Arduino SA [original work]
   Copyright (C) 2020 Niels Janssen // VLIEGWERK (https://www.vliegwerk.com)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <arm_math.h>

#include <Arduino_APDS9960.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
#include <PDM.h>

#include <ArduinoBLE.h>

#define ARDUINO_NANO_33_BLE_SERVICE_UUID(val) ("6fbe1da7-" val "-44de-92c4-bb6e04fb0212")

const int VERSION = 0x00000001;
const int IMU_SENSOR_RATE = 119

BLEService                     service                       (ARDUINO_NANO_33_BLE_SERVICE_UUID("0000"));
BLEUnsignedIntCharacteristic   versionCharacteristic         (ARDUINO_NANO_33_BLE_SERVICE_UUID("1001"), BLERead);
BLEUnsignedShortCharacteristic ambientLightCharacteristic    (ARDUINO_NANO_33_BLE_SERVICE_UUID("2001"), BLENotify); // 16-bit
BLECharacteristic              colorCharacteristic           (ARDUINO_NANO_33_BLE_SERVICE_UUID("2002"), BLENotify, 3 * sizeof(unsigned short)); // Array of 16-bit, RGB
BLEUnsignedCharCharacteristic  proximityCharacteristic       (ARDUINO_NANO_33_BLE_SERVICE_UUID("2003"), BLENotify); // Byte, 0 - 255 => close to far
BLEByteCharacteristic          gestureCharacteristic         (ARDUINO_NANO_33_BLE_SERVICE_UUID("2004"), BLENotify); // NONE = -1, UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3
BLECharacteristic              accelerationCharacteristic    (ARDUINO_NANO_33_BLE_SERVICE_UUID("3001"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, G
BLECharacteristic              gyroscopeCharacteristic       (ARDUINO_NANO_33_BLE_SERVICE_UUID("3002"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, dps
BLECharacteristic              magneticFieldCharacteristic   (ARDUINO_NANO_33_BLE_SERVICE_UUID("3003"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, uT
BLECharacteristic              orientationCharacteristic     (ARDUINO_NANO_33_BLE_SERVICE_UUID("3004"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, rad

BLEFloatCharacteristic         pressureCharacteristic        (ARDUINO_NANO_33_BLE_SERVICE_UUID("4001"), BLERead); // Float, kPa
BLEFloatCharacteristic         temperatureCharacteristic     (ARDUINO_NANO_33_BLE_SERVICE_UUID("4002"), BLERead); // Float, Celcius
BLEFloatCharacteristic         humidityCharacteristic        (ARDUINO_NANO_33_BLE_SERVICE_UUID("4003"), BLERead); // Float, Percentage
BLECharacteristic              microphoneLevelCharacteristic (ARDUINO_NANO_33_BLE_SERVICE_UUID("5001"), BLENotify, 32); // Int, RMS of audio input
BLECharacteristic              rgbLedCharacteristic          (ARDUINO_NANO_33_BLE_SERVICE_UUID("6001"), BLERead | BLEWrite, 3 * sizeof(byte)); // Array of 3 bytes, RGB

// String to calculate the local and device name
String name;

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];

arm_rfft_instance_q15 FFT;

// number of samples read
volatile int samplesRead;

// initialize a MadgwickAHRS filter:
Madgwick filter;

unsigned long micros_per_reading, micros_previous;

void setup() {
  Serial.begin(9600);

  //while (!Serial);
  Serial.println("Started");


  if (!APDS.begin()) {
    Serial.println("Failled to initialized APDS!");

    while (1);
  }

  if (!HTS.begin()) {
    Serial.println("Failled to initialized HTS!");

    while (1);
  }

  if (!BARO.begin()) {
    Serial.println("Failled to initialized BARO!");

    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failled to initialized IMU!");

    while (1);
  }

  // configure the data receive callback
  PDM.onReceive(onPDMdata);

  // initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("Failled to initialized BLE!");

    while (1);
  }

  String address = BLE.address();

  Serial.print("address = ");
  Serial.println(address);

  address.toUpperCase();

  name = "BLESense-";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  Serial.print("name = ");
  Serial.println(name);


  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(ambientLightCharacteristic);
  service.addCharacteristic(colorCharacteristic);
  service.addCharacteristic(proximityCharacteristic);
  service.addCharacteristic(gestureCharacteristic);
  service.addCharacteristic(accelerationCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(magneticFieldCharacteristic);
  service.addCharacteristic(orientationCharacteristic);

  service.addCharacteristic(pressureCharacteristic);
  service.addCharacteristic(temperatureCharacteristic);
  service.addCharacteristic(humidityCharacteristic);
  service.addCharacteristic(microphoneLevelCharacteristic);
  service.addCharacteristic(rgbLedCharacteristic);

  versionCharacteristic.setValue(VERSION);
  pressureCharacteristic.setEventHandler(BLERead, onPressureCharacteristicRead);
  temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
  humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
  rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);

  BLE.addService(service);

  BLE.advertise();

  // start the filter to run at the sample rate:
  filter.begin(IMU_SENSOR_RATE);

  micros_per_reading = 1000000/IMU_SENSOR_RATE;
  micros_previous = micros(); 
}

void loop() {
  while (BLE.connected()) {

    // Most recent IMU readings for 
    float acceleration[3];
    float dps[3];
    float magneticField[3];

    if ((ambientLightCharacteristic.subscribed() || colorCharacteristic.subscribed()) && APDS.colorAvailable()) {
      int r, g, b, ambientLight;

      APDS.readColor(r, g, b, ambientLight);

      ambientLightCharacteristic.writeValue(ambientLight);

      unsigned short colors[3] = { r, g, b };

      colorCharacteristic.writeValue(colors, sizeof(colors));
    }

    if (proximityCharacteristic.subscribed() && APDS.proximityAvailable()) {
      int proximity = APDS.readProximity();

      proximityCharacteristic.writeValue(proximity);
    }

    if (gestureCharacteristic.subscribed() && APDS.gestureAvailable()) {
      int gesture = APDS.readGesture();

      gestureCharacteristic.writeValue(gesture);
    }

    if ((orientationCharacteristic.subscribed() || accelerationCharacteristic.subscribed()) && IMU.accelerationAvailable()) {
      float x, y, z;

      IMU.readAcceleration(x, y, z);

      acceleration[0] = x;
      acceleration[1] = y;
      acceleration[2] = z;

      if (accelerationCharacteristic.subscribed()) accelerationCharacteristic.writeValue(acceleration, sizeof(acceleration));
    }

    if ((orientationCharacteristic.subscribed() || gyroscopeCharacteristic.subscribed()) && IMU.gyroscopeAvailable()) {
      float x, y, z;

      IMU.readGyroscope(x, y, z);

      dps[0] = x;
      dps[1] = y;
      dps[2] = z;

      if (gyroscopeCharacteristic.subscribed()) gyroscopeCharacteristic.writeValue(dps, sizeof(dps));
    }

    if ((orientationCharacteristic.subscribed() || magneticFieldCharacteristic.subscribed()) && IMU.magneticFieldAvailable()) {
      float x, y, z;

      IMU.readMagneticField(x, y, z);

      magneticField[0] = x;
      magneticField[1] = y;
      magneticField[2] = z;

      if (magneticFieldCharacteristic.subscribed()) magneticFieldCharacteristic.writeValue(magneticField, sizeof(magneticField));
    }

    if (orientationCharacteristic.subscribed() && (micros() - micros_previous >= micros_per_reading)) {
      float heading, pitch, roll;

      // update the filter, which computes orientation
      filter.update(
        dps[0], dps[1], dps[2],
        acceleration[0], acceleration[1], acceleration[2],
        magneticField[0], magneticField[1], magneticField[2]
      );

      heading = filter.getYawRadians();
      pitch = filter.getPitchRadians();
      roll = filter.getRollRadians();

      float orientation[3] = { heading, pitch, roll };

      orientationCharacteristic.writeValue(orientation, sizeof(orientation));

      micros_previous = micros_previous + micros_per_reading;
    }


    if (microphoneLevelCharacteristic.subscribed() && samplesRead) {
      short micLevel;
      // arm_rms_q15 (sampleBuffer, samplesRead, &micLevel);

      static arm_rfft_instance_q15 fft_instance;
      static q15_t fftoutput[256 * 2]; //has to be twice FFT size
      static byte spectrum[32];
      arm_rfft_init_q15(&fft_instance, 256/*bin count*/, 0/*forward FFT*/, 1/*output bit order is normal*/);
      arm_rfft_q15(&fft_instance, (q15_t*)sampleBuffer, fftoutput);
      arm_abs_q15(fftoutput, fftoutput, 256);

      float temp = 0;
      for (int i = 1; i < 256; i++) {
        temp = temp + fftoutput[i];
        if ((i & 3) == 2) {
          if (temp > 1023) {
            temp = 1023;
          };
          spectrum[i >> 3] = (byte)(temp / 2);
          temp = 0;
        }
      }
      microphoneLevelCharacteristic.writeValue((byte *) &spectrum, 32);
      samplesRead = 0;
    }
  }
}

void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float pressure = BARO.readPressure();

  pressureCharacteristic.writeValue(pressure);
}

void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float temperature = HTS.readTemperature() - 5;

  temperatureCharacteristic.writeValue(temperature);
}

void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float humidity = HTS.readHumidity();

  humidityCharacteristic.writeValue(humidity);
}

void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
  byte r = rgbLedCharacteristic[0];
  byte g = rgbLedCharacteristic[1];
  byte b = rgbLedCharacteristic[2];

  setLedPinValue(LEDR, r);
  setLedPinValue(LEDG, g);
  setLedPinValue(LEDB, b);
}

void setLedPinValue(int pin, int value) {
  // RGB LED's are pulled up, so the PWM needs to be inverted

  if (value == 0) {
    // special hack to clear LED
    analogWrite(pin, 256);
  } else {
    analogWrite(pin, 255 - value);
  }
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
