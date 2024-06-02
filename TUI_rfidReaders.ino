#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPI.h>
#include <MFRC522.h>
#include <string>
#include <iostream>

#define RST_PIN         32          // Configurable, see typical pin layout above
#define SS_1_PIN        14          // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 2
#define SS_2_PIN        12          // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 1
#define SS_3_PIN        13 

#define NR_OF_READERS   3

byte ssPins[] = {SS_1_PIN, SS_2_PIN, SS_3_PIN};

MFRC522 mfrc522[NR_OF_READERS];   // Create MFRC522 instance.

String state[NR_OF_READERS];
String lastState[NR_OF_READERS];

//MFRC522::MIFARE_Key key; 

// new NUID 
byte nuidPICC[4];

// https://www.uuidgenerator.net/

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

BLECharacteristic cardCharacteristic("b481757e-6e97-4ea4-862b-5a651dc7db82", BLECharacteristic::PROPERTY_NOTIFY|BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor cardDescriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic positionCharacteristic("6ea4171c-3f56-443e-a654-83f2c6f867d7", BLECharacteristic::PROPERTY_NOTIFY|BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor positionDescriptor(BLEUUID((uint16_t)0x2902));

bool deviceConnected = false;

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);

  SPI.begin(); // Init SPI bus

  Serial.println("Starting BLE");
  Serial.println();

  // for (byte i = 0; i < 6; i++) {
  //   key.keyByte[i] = 0xFF;
  // }

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], MFRC522::UNUSED_PIN); // Init each MFRC522 card
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
    mfrc522[reader].PCD_DumpVersionToSerial();
  }

  BLEDevice::init("ESP32server");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pService ->addCharacteristic(&cardCharacteristic);
  cardDescriptor.setValue("Card ID");
  cardCharacteristic.addDescriptor(new BLE2902());

  pService ->addCharacteristic(&positionCharacteristic);
  positionDescriptor.setValue("Position");
  positionCharacteristic.addDescriptor(new BLE2902());  

  pServer->setCallbacks(new MyServerCallbacks());
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined");
}

void loop() {
  for (int reader = 0; reader < NR_OF_READERS; reader++) {

  if(!mfrc522[reader].PICC_IsNewCardPresent()) {
    cardCharacteristic.setValue(new byte[0], 0);

  } else if (mfrc522[reader].PICC_ReadCardSerial()) {
      Serial.print(F("Reader "));
      Serial.print(reader);
      positionCharacteristic.setValue(reader);

      MFRC522::PICC_Type piccType = mfrc522[reader].PICC_GetType(mfrc522[reader].uid.sak);
      
      Serial.print(F(": Card UID:"));
      printDec(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.println();

      state[reader] = "";

      //WRITE TO STATE
      for (byte i = 0; i < mfrc522[reader].uid.size; i++) {
        state[reader] += String(mfrc522[reader].uid.uidByte[i], DEC);
        if (i < mfrc522[reader].uid.size - 1) {
          state[reader] += " ";
        }
      }

      cardCharacteristic.setValue(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.println();

      mfrc522[reader].PICC_HaltA();
      mfrc522[reader].PCD_StopCrypto1();

        Serial.println("State array:");
        for (int reader = 0; reader < NR_OF_READERS; reader++) {
        Serial.print("Reader ");
        Serial.print(reader);
        Serial.print(": ");
        Serial.println(state[reader]);
    }
    }
    //pollPres(reader);
    
  }

}

void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(' ');
    Serial.print(buffer[i], DEC);
  }
}

void pollPres(int reader){
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  mfrc522[reader].PCD_WriteRegister(mfrc522[reader].TxModeReg, 0x00);
  mfrc522[reader].PCD_WriteRegister(mfrc522[reader].RxModeReg, 0x00);
  mfrc522[reader].PCD_WriteRegister(mfrc522[reader].ModWidthReg, 0x26);

   
   if(mfrc522[reader].PICC_WakeupA(bufferATQA, &bufferSize)){
     if (mfrc522[reader].uid.size != 0) {
      Serial.print(reader);
      Serial.print(": ");
      printDec(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.print(" LEFT"); // WRITE TO CHAR
      Serial.println();
     }
   }
   else{
      // DO NOTHING
   }
   mfrc522[reader].PICC_HaltA();   
}
