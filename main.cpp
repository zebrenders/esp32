/*Complete project details at https://RandomNerdTutorials.com/esp32-web-bluetooth/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <DFRobot_LIS2DW12.h>

// When using I2C communication, use the following program to construct an object by DFRobot_LIS2DW12_I2C
/*!
 * @brief Constructor
 * @param pWire I2c controller
 * @param addr  I2C address(0x18/0x19)
 */
// DFRobot_LIS2DW12_I2C acce(&Wire,0x18);
DFRobot_LIS2DW12_I2C acce;

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 3

RTC_DATA_ATTR int bootCount = 0;

#define LIS2DW12_CS D5

BLEServer *pServer = NULL;
BLECharacteristic *pSensorCharacteristic = NULL;
BLECharacteristic *pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define ledPin D10 // Use the appropriate GPIO pin for your setup

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

void set_tap(float threshold)
{
  acce.setTapThresholdOnX(threshold);
  acce.setTapThresholdOnY(threshold);
  acce.setTapThresholdOnZ(threshold);
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pLedCharacteristic)
  {

    std::string ledvalue = pLedCharacteristic->getValue();
    String value = String(ledvalue.c_str());

    Serial.print("Characteristic event, written: ");
    Serial.println(static_cast<int>(value[0])); // Print the integer value

    int receivedValue = static_cast<int>(value[0]);
    if (receivedValue == 1)
    {
      digitalWrite(ledPin, HIGH);
    }
    else if (receivedValue == 0)
    {
      digitalWrite(ledPin, LOW);
    }
  }
};

void setup()
{
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  while (!acce.begin())
  {
    Serial.println("Communication failed, check the connection and I2C address setting when using I2C communication.");
    delay(1000);
  }
  Serial.print("chip id : ");
  Serial.println(acce.getID(), HEX);
  // Software reset
  acce.softReset();
  acce.setRange(DFRobot_LIS2DW12::e2_g);
  acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
  acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  acce.setWakeUpDur(0.5);
  acce.setWakeUpThreshold(0.1);
  acce.setPowerMode(DFRobot_LIS2DW12::eContLowPwrLowNoise1_12bit);
  acce.setActMode(DFRobot_LIS2DW12::eDetectAct);
  acce.setInt1Event(DFRobot_LIS2DW12::eWakeUp);
  acce.setDataRate(DFRobot_LIS2DW12::eRate_200hz);
  acce.setTapDur(6);
  acce.enableTapDetectionOnZ(true);
  acce.enableTapDetectionOnY(true);
  acce.enableTapDetectionOnX(true);
  set_tap(0.2);
  acce.setTapMode(DFRobot_LIS2DW12::eBothSingleDouble);
  delay(100);

  // Create the BLE Device
  BLEDevice::init("ESP32_zeb");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
      SENSOR_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
      LED_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop()
{
  // notify changed value
  if (deviceConnected)
  {
    value = acce.getTapDirection();
    pSensorCharacteristic->setValue(String(value).c_str());
    pSensorCharacteristic->notify();

    Serial.print("New value notified: ");
    Serial.println(value);
    delay(3000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    Serial.println("Device disconnected.");
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}