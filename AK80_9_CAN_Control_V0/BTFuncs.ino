#include<ArduinoBLE.h>

void setupBLE() {
  if (BLE.begin()) {
    pinMode(BLUE, OUTPUT);
    //Advertised name and service
    BLE.setLocalName("TMOTOR");
    BLE.setDeviceName("TMOTOR");
    BLE.setAdvertisedService(TMotorService);
    //Add chars to service
    TMotorService.addCharacteristic(enableChar);
    TMotorService.addCharacteristic(setpointChar);
    TMotorService.addCharacteristic(modeChar);
    TMotorService.addCharacteristic(motorChar);
    //Add service to BLE
    BLE.addService(TMotorService);

    //Add callbacks
    BLE.setEventHandler(BLEConnected, onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);
    enableChar.setEventHandler(BLEWritten, enableWritten);
    setpointChar.setEventHandler(BLEWritten, setpointWritten);
    modeChar.setEventHandler(BLEWritten, modeWritten);
    motorChar.setEventHandler(BLEWritten, motorWritten);
    BLE.advertise();
  }
}

//==============BLE Callbacks=================

void onBLEConnected(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  BLE.stopAdvertise();
  digitalWrite(BLUE, LOW);
}

void onBLEDisconnected(BLEDevice central) {
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  BLE.advertise();
  digitalWrite(BLUE, HIGH);
}

void enableWritten(BLEDevice central, BLECharacteristic characteristic) {
  bt_enable = enableChar.value();
  Serial.print("Enable written to: ");
  Serial.println(bt_enable);
  if (bt_enable) {
    Serial.println("Enable");
  } else {
    Serial.println("Disable");
  }
}

void setpointWritten(BLEDevice central, BLECharacteristic characteristic) {
  bt_setpoint = setpointChar.value();
  Serial.print("Setpoint written to: ");
  Serial.println(bt_setpoint);
}

void modeWritten(BLEDevice central, BLECharacteristic characteristic) {
  bt_mode = modeChar.value();
  Serial.print("Mode written to: ");
  Serial.println(bt_mode);
}

void motorWritten(BLEDevice central, BLECharacteristic characteristic) {
  bt_motor = motorChar.value();
  Serial.print("Motor written to: ");
  Serial.println(bt_motor);
}
