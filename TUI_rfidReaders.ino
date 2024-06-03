#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPI.h>
#include <MFRC522.h>
#include <esp_now.h>
#include <WiFi.h>

#define RST_PIN         32
#define SS_1_PIN        14
#define SS_2_PIN        12
#define SS_3_PIN        13 

#define NR_OF_READERS   3

// -----------------------ESP-NOW---------------------------------------------
typedef struct struct_message {
  int id;
  String module[NR_OF_READERS];
}struct_message;

// Create a struct_message called myData
struct_message myData;
// Create a structure to hold the readings from each board
struct_message board1;
// Create an array with all the structures
struct_message boardsStruct[1] = {board1};
// -----------------------ESP-NOW---------------------------------------------

byte ssPins[] = {SS_1_PIN, SS_2_PIN, SS_3_PIN};

MFRC522 mfrc522[NR_OF_READERS];

String state[NR_OF_READERS];
String lastState[NR_OF_READERS];

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define STATE_CHAR_UUID "b481757e-6e97-4ea4-862b-5a651dc7db82"

BLECharacteristic stateCharacteristic(STATE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY|BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor stateDescriptor(BLEUUID((uint16_t)0x2902));

bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};


// -----------------------ESP-NOW---------------------------------------------
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  for (int i = 0; i < NR_OF_READERS; i++) {
    boardsStruct[myData.id-1].module[i] = myData.module[i];
  }
  Serial.printf("module value: %d \n", boardsStruct[myData.id-1].module);
  Serial.println();
}
// -----------------------ESP-NOW---------------------------------------------

void setup() {

  
  Serial.begin(115200);

  // -----------------------ESP-NOW---------------------------------------------
  WiFi.mode(WIFI_STA);
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    //return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  // -----------------------ESP-NOW---------------------------------------------

  SPI.begin();

  Serial.println("Starting BLE");
  Serial.println();

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], MFRC522::UNUSED_PIN);
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
    mfrc522[reader].PCD_DumpVersionToSerial();
  }

  BLEDevice::init("ESP32server");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pService ->addCharacteristic(&stateCharacteristic);
  stateDescriptor.setValue("State Array");
  stateCharacteristic.addDescriptor(new BLE2902());  

  pServer->setCallbacks(new MyServerCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined");
}

void loop() {
  // Acess the variables for each board

  for (int reader = 0; reader < NR_OF_READERS; reader++) {
    if(!mfrc522[reader].PICC_IsNewCardPresent()) {
      // If no card is present
    } else if (mfrc522[reader].PICC_ReadCardSerial()) {
      Serial.print(F("Reader "));
      Serial.print(reader);

      MFRC522::PICC_Type piccType = mfrc522[reader].PICC_GetType(mfrc522[reader].uid.sak);
      
      Serial.print(F(": Card UID:"));
      printDec(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.println();

      // Check if the UID is already present in the state array
      bool uidPresent = false;
      String currentUID = "";
      for (int i = 0; i < NR_OF_READERS; i++) {
        if (state[i] == "") continue; // Skip empty states
        if (state[i] == currentUID) {
          uidPresent = true;
          break;
        }
      }

      if (!uidPresent) {
        // If UID is not present in any other reader's state, update the state for this reader
        state[reader] = "";
        for (byte i = 0; i < mfrc522[reader].uid.size; i++) {
          state[reader] += String(mfrc522[reader].uid.uidByte[i], DEC);
          if (i < mfrc522[reader].uid.size - 1) {
            state[reader] += " ";
          }
        }
      }

      // Set the stateCharacteristic value to the serialized state array
      String serializedState = serializeStateArray();
      stateCharacteristic.setValue(serializedState.c_str());

      mfrc522[reader].PICC_HaltA();
      mfrc522[reader].PCD_StopCrypto1();

      // Print out the state array
      printStateArray();
    }
  }
}

String serializeStateArray() {
  String serializedState = "";
  for (int i = 0; i < NR_OF_READERS; i++) {
    serializedState += state[i];
    if (i < NR_OF_READERS - 1) {
      serializedState += ",";
    }
  }
  return serializedState;
}

void printStateArray() {
  Serial.println("State array:");
  for (int reader = 0; reader < NR_OF_READERS; reader++) {
    Serial.print("Reader ");
    Serial.print(reader);
    Serial.print(": ");
    Serial.println(state[reader]);
  }
}

void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(' ');
    Serial.print(buffer[i], DEC);
  }
}
