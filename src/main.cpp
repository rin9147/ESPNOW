#include <M5Core2.h>
#include <WiFi.h>
#include <esp_now.h>
#include <CircularBuffer.h>

CircularBuffer<int, 10> esp_status_buffer;
uint16_t esp_status_counter;

//  ESP-NOW
esp_err_t result;

esp_now_peer_info_t peerInfo;

unsigned int Slave_waiting = 0;

float angleData_;
int velocityData_;
int torqueData_;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t Address[] = {0xAC, 0x0B, 0xFB, 0x6F, 0x40, 0x70}; //stamp
//uint8_t Address[] = {0x8C, 0xAA, 0xB5, 0x81, 0x72, 0x7C}; //core2

typedef struct struct_message_to_Slave
{
  unsigned int Command;
  unsigned int torque_patternData;
} struct_message_to_Slave;
struct_message_to_Slave toSlaveData;

typedef struct struct_message_from_Slave
{
  float angleData;
  int velocityData;
  int torqueData;
} struct_message_from_Slave;
struct_message_from_Slave fromSlaveData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

//PROTOTYPE
void init_esp();

void setup()
{
  M5.begin(true, true, false, false, kMBusModeOutput);
  M5.Lcd.setTextSize(2);

  init_esp();
  esp_status_buffer.push(10);

  delay(1000);
  M5.Lcd.clear();
}

void loop()
{
  M5.Lcd.setCursor(0, 0);

  toSlaveData.torque_patternData = 2;
  toSlaveData.Command = 3000;
  
  M5.Lcd.println(toSlaveData.torque_patternData);
  M5.Lcd.println(toSlaveData.Command);

  M5.Lcd.println(fromSlaveData.angleData);
  M5.Lcd.println(fromSlaveData.velocityData);
  M5.Lcd.println(fromSlaveData.torqueData);

  esp_now_send(Address, (uint8_t *)&toSlaveData, sizeof(toSlaveData));
  delay(500);
  M5.Lcd.clear();
}

// ESPNOW initialization
//------------------------------------------------------------------//
void init_esp()
{
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() == ESP_OK)
  {
    M5.Lcd.println("Success for init ESP NOW");
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    M5.Lcd.println("Failed to add peer1");
  }
  else
  {
    M5.Lcd.println("Success to add peer1");
  }
}

// callback when data is sent
//------------------------------------------------------------------//
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  /*
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  M5.Lcd.printf("Last Packet Sent to: ");
  M5.Lcd.println(macStr);
  M5.Lcd.printf("Last Packet Send Status:\t");
  M5.Lcd.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  M5.Lcd.println();
  */

  if (status != ESP_NOW_SEND_SUCCESS)
  {
    esp_status_buffer.push(0);
  }
  else
  {
    esp_status_buffer.push(1);
  }
  esp_status_counter = 0;
  using index_t = decltype(esp_status_buffer)::index_t;
  for (index_t i = 0; i < esp_status_buffer.size(); i++)
  {
    esp_status_counter += esp_status_buffer[i];
  }
  if (esp_status_counter <= 5)
  {
    //SEND PATTERN OF EMG STOP
    //pattern = 90...;
    M5.Lcd.println("ESPNOW LOST COMM");
    init_esp();
  }
}

// callback function that will be executed when data is received
//------------------------------------------------------------------//
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&fromSlaveData, incomingData, sizeof(fromSlaveData));
  /*
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  M5.Lcd.println(macStr);
  */
}
