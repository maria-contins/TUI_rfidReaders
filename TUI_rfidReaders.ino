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

  //rfid.PCD_Init(); // Init MFRC522 
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
    //positionCharacteristic.setValue(reader);
    cardCharacteristic.setValue(new byte[0], 0);

  } else if (mfrc522[reader].PICC_ReadCardSerial()) {
      //Serial.print(mfrc522[reader].PICC_ReadCardSerial());
      Serial.print(F("Reader "));
      Serial.print(reader);
      positionCharacteristic.setValue(reader);
      //positionCharacteristic.notify();

      // Show some details of the PICC (that is: the tag/card)
      //Serial.print(F("PICC type: "));
      MFRC522::PICC_Type piccType = mfrc522[reader].PICC_GetType(mfrc522[reader].uid.sak);
      
      Serial.print(F(": Card UID:"));
      printDec(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.println();
      
      //Serial.println(mfrc522[reader].PICC_GetTypeName(piccType));

      cardCharacteristic.setValue(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      //cardCharacteristic.notify(); 
      Serial.println();

      //Serial.println(mfrc522[reader].PICC_ReadCardSerial());

      // Halt PICC
      mfrc522[reader].PICC_HaltA();
      // Stop encryption on PCD
      mfrc522[reader].PCD_StopCrypto1();
    }



    //pollPres(reader);
    
  }
  delay(30);
}


/**
 * Helper routine to dump a byte array as hex values to Serial. 
 */
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
 * Helper routine to dump a byte array as dec values to Serial.
 */
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
    //  Serial.print(reader);
    //  Serial.print(": ");
    //  printDec(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
    //  Serial.print(" Still here"); // works the otehr way around?
    //  Serial.println();



    //   // A halted card has responded to WUPA therefore our card is still there

    //   mfrc522[reader].PICC_HaltA();
   }
   else{
      // No response to WUPA our card must have left
      Serial.print(reader);
     Serial.print(": ");
     printDec(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
     Serial.print(" left");
     Serial.println();
   }
}
