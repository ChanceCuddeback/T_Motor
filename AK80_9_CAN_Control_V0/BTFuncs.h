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


#endif
