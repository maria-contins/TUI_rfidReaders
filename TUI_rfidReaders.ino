#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPI.h>
#include <MFRC522.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define NUMFLAKES     10
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
#define XPOS   0 // Indexes into the 'icons' array in function below
#define YPOS   1
#define DELTAY 2
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };
  
#define SS_1_PIN        12
#define SS_2_PIN        13
#define SS_3_PIN        14 
#define RST_PIN         22         // Configurable, see typical pin layout above
#define BUTTON_PIN      32
#define NR_OF_READERS   3
#define MODULE_SIZE     50  // Max size for the module char array
#define ID              0
#define BLE_NAME        "ESP32server" 

enum MessageType {ELECTION, DATA};
MessageType messageType;

// LEADER ELECTION

// Leader
bool leader = false;
bool knowsleader = false;
int id = 0;
unsigned long lastMsgTime = 0;

// MAC
uint8_t broadcastAddress[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t leaderAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// Create peer interface
esp_now_peer_info_t peerInfo;

typedef struct leader_Message {
  uint8_t msgType;
  int type;
  int id;
} leader_Message;

leader_Message myleader_Message;

// MODULE STATE

int buttonState;            // Current reading from the input pin
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;  // Last time the output pin was toggled
unsigned long debounceDelay = 20;
bool readCard = false;

int nr_modules = 3; // TODO: change dynamically

typedef struct struct_message {
  uint8_t msgType;
  int id;
  int nr_readers;
  char module[MODULE_SIZE];
} struct_message;

struct_message myData;

byte ssPins[] = {SS_1_PIN, SS_2_PIN, SS_3_PIN};
MFRC522 mfrc522[NR_OF_READERS];

String state[NR_OF_READERS];
String globalState[NR_OF_READERS * 3];  // Assuming 3 modules for now
String lastState[NR_OF_READERS];

// BLE CONFIG

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define STATE_CHAR_UUID "b481757e-6e97-4ea4-862b-5a651dc7db82"

BLECharacteristic stateCharacteristic(STATE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor stateDescriptor(BLEUUID((uint16_t)0x2902));

bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};


void printStructMessage(const struct_message& msg) {
  Serial.print("ID: ");
  Serial.println(msg.id);

  Serial.print("Number of Readers: ");
  Serial.println(msg.nr_readers);

  Serial.print("Modules: ");
  Serial.println(msg.module);
}

void printMAC(const uint8_t *mac_addr) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);  
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.print("RECEIVED MESSAGE: ");
  uint8_t type = incomingData[0];
  switch (type) {
  case DATA :  
    readData(mac_addr, incomingData, len);
    break;
  case ELECTION :
    checkLeader(mac_addr, incomingData, len);
    break;
  }
}

void readData(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    printMAC(mac_addr);
    memcpy(&myData, incomingData, sizeof(myData));
    printStructMessage(myData);

    // Directly use myData.module without re-serializing the ID
    stateCharacteristic.setValue(myData.module);
}

void checkLeader(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  leader_Message incomingMsg;
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));

  Serial.print("Received message of type: ");
  Serial.println(incomingMsg.type);

  if (incomingMsg.type == 3) { // there is a leader
    memcpy(leaderAddress, mac_addr, 6);
    char macStr[18];
    Serial.print("Packet received from: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.println(macStr);
    // Register peer
    memcpy(peerInfo.peer_addr, leaderAddress, 6);
    peerInfo.channel = 1;  
    peerInfo.encrypt = false;
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    Serial.print("Leader elected with ID: ");
    Serial.println(incomingMsg.id);
    leader = false; // If we receive an elected message, we are not the leader
    knowsleader = true;
  }
  else if (incomingMsg.type == 2) { // checking
    if (leader) {
      Serial.println("Received check_leader message, sending elected message.");
      leader_Message electedMsg;
      electedMsg.msgType = ELECTION;
      electedMsg.type = 3;
      electedMsg.id = id;
      esp_now_send(broadcastAddress, (uint8_t *) &electedMsg, sizeof(electedMsg));
    }
  }
  else if (incomingMsg.type == 1 && leader) {
    Serial.print("Received data message from ID: ");
    Serial.println(incomingMsg.id);
  }  
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void espNowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.channel(1);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized.");
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  leaderElection();

  delay(500);
}

void leaderElection() {
  Serial.println("Starting leader election.");
  leader_Message checkLeaderMsg;
  checkLeaderMsg.msgType = ELECTION;
  checkLeaderMsg.type = 2;
  esp_now_send(broadcastAddress, (uint8_t *) &checkLeaderMsg, sizeof(checkLeaderMsg));

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    // Waiting for elected message
    delay(100);
  }

  if (!knowsleader) {
    Serial.println("No leader found, assuming leadership.");
    leader = true;
    leader_Message electedMsg;
    electedMsg.msgType = ELECTION;
    electedMsg.type = 3;
    electedMsg.id = id;
    esp_now_send(broadcastAddress, (uint8_t *) &electedMsg, sizeof(electedMsg));
    knowsleader = true;
  } else {
    Serial.println("Leader found.");
  }

  if(!leader) {
    esp_now_unregister_recv_cb();
  }
}

void bleInit() {
    Serial.println("Starting BLE");

    BLEDevice::init(BLE_NAME);
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);

    pService->addCharacteristic(&stateCharacteristic);
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

void setupLCDs() {
   if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Ready :)");
  display.display();

  delay(2000);
  display.clearDisplay();

  testanimate(logo_bmp, LOGO_WIDTH, LOGO_HEIGHT);
}

void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for(f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for(;;) { // Loop forever...
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for(f=0; f< NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for(f=0; f< NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initialize SPI
    SPI.begin();

    // Initialize RFID readers
    for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
        mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN);
        delay(30);
        Serial.print(F("Reader "));
        Serial.print(reader);
        Serial.print(F(": "));
        mfrc522[reader].PCD_DumpVersionToSerial();
    }

    // Initialize ESP-NOW
    espNowInit();

    // Initialize BLE if this ESP32 is the leader
    if (leader) {
        bleInit();
    }

    setupLCDs();
}


void serializeStateArray(char *serializedState) {
    String stateString = "";
    stateString += ID;
    stateString += ",";
    for (int i = 0; i < NR_OF_READERS; i++) {
        stateString += state[i];
        if (i < NR_OF_READERS - 1) {
            stateString += ",";
        }
    }
    stateString += (buttonState == HIGH) ? ",1" : ",0";
    stateString.toCharArray(serializedState, MODULE_SIZE);
}

void serializeMessage(char *serializedState, const struct_message& myData) {
    String stateString = "";
    // Only serialize module, since module already includes the necessary information.
    stateString += myData.module;
    stateString.toCharArray(serializedState, MODULE_SIZE);
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

void pollPres(int reader) {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);

  mfrc522[reader].PCD_WriteRegister(mfrc522[reader].TxModeReg, 0x00);
  mfrc522[reader].PCD_WriteRegister(mfrc522[reader].RxModeReg, 0x00);
  mfrc522[reader].PCD_WriteRegister(mfrc522[reader].ModWidthReg, 0x26);

  if (mfrc522[reader].PICC_WakeupA(bufferATQA, &bufferSize)) {
    if (mfrc522[reader].uid.size != 0) {
      if (state[reader] != "") {
        state[reader] = "";

        char serializedState[MODULE_SIZE];
        serializeStateArray(serializedState);
        printStateArray();
        sendData();

        
      }
    }
  }
  mfrc522[reader].PICC_HaltA();
}

void sendData() {        
  char serializedState[MODULE_SIZE];
  serializeStateArray(serializedState);
        
  if (leader) {
    stateCharacteristic.setValue(serializedState);
  } else {       
    myData.msgType = DATA;   
    myData.id = ID;
    myData.nr_readers = NR_OF_READERS;
    strncpy(myData.module, serializedState, MODULE_SIZE);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
  }
}

void loop() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        Serial.println("Button press.");
        sendData();
      }
    }
  }

  lastButtonState = reading;

  for (int reader = 0; reader < NR_OF_READERS; reader++) {
    if (!mfrc522[reader].PICC_IsNewCardPresent()) {
      pollPres(reader);
    } else if (mfrc522[reader].PICC_ReadCardSerial()) {
      state[reader] = "";
      for (byte i = 0; i < mfrc522[reader].uid.size; i++) {
        state[reader] += String(mfrc522[reader].uid.uidByte[i], DEC);
        if (i < mfrc522[reader].uid.size - 1) {
          state[reader] += " ";
        }
      }
      
      printStateArray();      
      sendData();
      
      mfrc522[reader].PICC_HaltA();
      mfrc522[reader].PCD_StopCrypto1();

      
    }
  }
}
