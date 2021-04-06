#ifndef BT_FUNCS_H
#define BT_FUNCS_H

#include<ArduinoBLE.h>


void setupBLE();
void onBLEConnected(BLEDevice central);
void onBLEDisconnected(BLEDevice central);
void enableWritten(BLEDevice central, BLECharacteristic characteristic);
void setpointWritten(BLEDevice central, BLECharacteristic characteristic);
void modeWritten(BLEDevice central, BLECharacteristic characteristic);
void motorWritten(BLEDevice central, BLECharacteristic characteristic);


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

#define RED 22
#define GREEN 23
#define BLUE 24


#endif
