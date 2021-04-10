#include<ArduinoBLE.h>
#include"BTFuncs.h"
#include"Globals.h"

//Complete Bluetooth test
const char* TMotorServiceUUID = "12341234-1212-EFDE-1523-785FEABCD120";
const char* enableCharUUID    = "12341234-1212-EFDE-1523-785FEABCD121";
const char* setpointCharUUID  = "12341234-1212-EFDE-1523-785FEABCD122";
const char* modeCharUUID      = "12341234-1212-EFDE-1523-785FEABCD123";
const char* motorCharUUID     = "12341234-1212-EFDE-1523-785FEABCD124";

BLEService TMotorService(TMotorServiceUUID);
BLEByteCharacteristic enableChar(enableCharUUID,        BLERead | BLEWrite | BLEBroadcast); 
BLEFloatCharacteristic setpointChar(setpointCharUUID,  BLERead | BLEWrite | BLEBroadcast);
BLEIntCharacteristic modeChar(modeCharUUID,             BLERead | BLEWrite | BLEBroadcast);
BLEIntCharacteristic motorChar(motorCharUUID,           BLERead | BLEWrite | BLEBroadcast);


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
    digitalWrite(GREEN,LOW);
  }
  else {
    digitalWrite(RED,LOW);
  }
}

//==============BLE Callbacks=================

void onBLEConnected(BLEDevice central) {
  //Serial.print("Connected event, central: ");
  //Serial.println(central.address());
  BLE.stopAdvertise();
  digitalWrite(GREEN,HIGH);
  digitalWrite(RED,HIGH);
  digitalWrite(BLUE, LOW);
}

void onBLEDisconnected(BLEDevice central) {
  //Serial.print("Disconnected event, central: ");
  //Serial.println(central.address());
  BLE.advertise();
  digitalWrite(BLUE, HIGH);
}

void enableWritten(BLEDevice central, BLECharacteristic characteristic) {
  //bt_enable = enableChar.value();
  //Serial.print("Enable written to: ");
  //Serial.println(bt_enable);
  /*
  if (bt_motor == 0) {
    if (bt_enable) {
      //Serial.println("Left Enable");
      left_motor.enableMotor(true);
    } else {
      //Serial.println("Left disable");
      left_motor.disableMotor();
    }
  } else if (bt_motor == 1) {
    if (bt_enable) {
      //Serial.println("Right Enable");
      //right_motor.enableMotor(true);
    } else {
      //Serial.println("Right Disable");
      //right_motor.disableMotor();
    }
  }
  */
}

void setpointWritten(BLEDevice central, BLECharacteristic characteristic) {
  //bt_setpoint = setpointChar.value();
  //Serial.print("Setpoint written to: ");
  //Serial.println(bt_setpoint);
  /*
  if (bt_motor == 0) {
    left_motor.setpoint(bt_setpoint);
  } else if (bt_motor == 1) {
    //Serial.println("Right motor setpoint changed");
    //right_motor.setpoint(bt_setpoint);
  }
  */
}

void modeWritten(BLEDevice central, BLECharacteristic characteristic) {
  //bt_mode = modeChar.value();
  //Serial.print("Mode written to: ");
  //Serial.println(bt_mode);
  /*
  if (bt_motor == 0) {
    left_motor.changeMode(bt_mode);
  } else if (bt_motor == 1) {
    //Serial.println("Right motor mode changed");
    //right_motor.changeMode(bt_mode);
  }
  */
}

void motorWritten(BLEDevice central, BLECharacteristic characteristic) {
  //bt_motor = motorChar.value();
  //Serial.print("Motor written to: ");
  //Serial.println(bt_motor);
  //Update new motor with global variables (not setpoint)
  /*
  if (bt_motor == 0) {
    left_motor.changeMode(bt_mode);
    if (bt_enable) {
      left_motor.enableMotor(true);
    } else {
      left_motor.disableMotor();
    }
  } else if (bt_motor == 1) {
    //right_motor.changeMode(bt_mode);
    if (bt_enable) {
      //right_motor.enableMotor(true);
    } else {
      //right_motor.disableMotor();
    }
  }
  */
}
