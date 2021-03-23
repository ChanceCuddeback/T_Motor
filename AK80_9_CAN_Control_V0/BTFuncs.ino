//Complete Bluetooth test
const char* TMotorServiceUUID = "12341234-1212-EFDE-1523-785FEABCD120";
const char* enableCharUUID    = "12341234-1212-EFDE-1523-785FEABCD121";
const char* setpointCharUUID  = "12341234-1212-EFDE-1523-785FEABCD122";
const char* modeCharUUID      = "12341234-1212-EFDE-1523-785FEABCD123";
const char* motorCharUUID     = "12341234-1212-EFDE-1523-785FEABCD124";

BLEService TMotorService(TMotorServiceUUID);
BLEByteCharacteristic enableChar(enableCharUUID,        BLERead | BLEWrite | BLEBroadcast); 
BLEDoubleCharacteristic setpointChar(setpointCharUUID,  BLERead | BLEWrite | BLEBroadcast);
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
  byte enable = enableChar.value();
  Serial.print("Enable written to: ");
  Serial.println(enable);
  if (enable) {
    Serial.println("Enable");
  } else {
    Serial.println("Disable");
  }
}
void setpointWritten(BLEDevice central, BLECharacteristic characteristic) {
  double setpoint = setpointChar.value();
  Serial.print("Setpoint written to: ");
  Serial.println(setpoint);
}
void modeWritten(BLEDevice central, BLECharacteristic characteristic) {
  int mode = modeChar.value();
  Serial.print("Mode written to: ");
  Serial.println(mode);
}
void motorWritten(BLEDevice central, BLECharacteristic characteristic) {
  int motor = motorChar.value();
  Serial.print("Motor written to: ");
  Serial.println(motor);
}
