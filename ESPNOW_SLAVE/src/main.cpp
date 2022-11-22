#include <M5Atom.h>
#include <mcp_can.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <CircularBuffer.h>

CircularBuffer<int, 10> esp_status_buffer;
uint16_t esp_status_counter;

// CAN
long unsigned int canId;
unsigned char len = 0;
unsigned char data[8];

#define VSPI_CLOCK_PIN 18
#define VSPI_MOSI_PIN 26
#define VSPI_MISO_PIN 36
#define CAN0_INT 21
MCP_CAN CAN0(19);

int cnt = 0;
int TYPE = 0; // Standard Format
int DLC = 8;

byte sndStat;

byte angle_H;
byte angle_L;
byte velocity_H;
byte velocity_L;
byte torque_H;
byte torque_L;

int angle_buff;
float angle;
int velocity_buff;
int velocity;
int torque_buff;
int torque;

unsigned int torque_pattern;

// ESP-NOW
esp_err_t result;

esp_now_peer_info_t peerInfo;

unsigned int Master_waiting = 0;

uint8_t Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // YOUR ESP MACADDRESS

typedef struct struct_message_to_Master
{
  float angleData;
  int velocityData;
  int torqueData;
} struct_message_to_Master;
struct_message_to_Master toMasterData;

typedef struct struct_message_from_Master
{
  unsigned int Command;
  unsigned int torque_patternData;
} struct_message_from_Master;
struct_message_from_Master fromMasterData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

//------------------------------------------------------------------//
//PROTOTYPE
void init_can();
void canRecieve(void);
void torqueControl(int trq);
void init_esp();

void setup()
{
  M5.begin(true, false, true);
  
  init_can();
  init_esp();

  esp_status_buffer.push(10);

  M5.dis.drawpix(0, 0x000000);
  delay(1000);
}

void loop()
{
  canRecieve();
  result = esp_now_send(Address, (uint8_t *)&toMasterData, sizeof(toMasterData));
}

//CAN initialization
void init_can()
{
  SPIClass mffVSPI = SPIClass(VSPI);
  mffVSPI.begin(VSPI_CLOCK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN);

  Serial.printf("CAN INIT... ");
  if(CAN0.begin(mffVSPI, MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK)
  {
    Serial.println("OK");
    M5.dis.drawpix(0, 0x0000ff);
    delay(50);
    M5.dis.drawpix(0, 0x000000);
    delay(50);
    M5.dis.drawpix(0, 0x0000ff);
    delay(50);
    M5.dis.drawpix(0, 0x000000);
    delay(50);
  }
  else
  {
    Serial.println("FAIL");
    M5.dis.drawpix(0, 0x00ff00);
    delay(50);
    M5.dis.drawpix(0, 0x000000);
    delay(50);
    M5.dis.drawpix(0, 0x00ff00);
    delay(50);
    M5.dis.drawpix(0, 0x000000);
    delay(50);
  }
  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);
}

void canRecieve(void)
{
  canId = 201;
  byte sndStat = CAN0.readMsgBuf(&canId, &len, data);
  if (sndStat == CAN_OK)
  {
    M5.dis.drawpix(0, 0x0000ff);
  }
  else
  {
    M5.dis.drawpix(0, 0x00ff00);
  }

  for (int i = 0; i < 8; i++)
  {
    switch (i)
    {
    case 0:
      angle_H = (char)data[i];
      break;
    case 1:
      angle_L = (char)data[i];
      break;
    case 2:
      velocity_H = (char)data[i];
      break;
    case 3:
      velocity_L = (char)data[i];
      break;
    case 4:
      torque_H = (char)data[i];
      break;
    case 5:
      torque_L = (char)data[i];
      break;
    case 6:
      break;
    case 7:
      break;
    }
  }

  angle_buff = ((angle_H << 8 & 0xFF00) | (angle_L & 0xFF));
  angle = (float)angle_buff / 8191 * 360;
  velocity_buff = ((velocity_H << 8 & 0xFF00) | (velocity_L & 0xFF));
  if (velocity_buff >= 32768)
  {
    velocity = velocity_buff - 65535;
  }
  else
  {
    velocity = velocity_buff;
  }
  torque_buff = ((torque_H << 8 & 0xFF00) | (torque_L & 0xFF));
  if (torque_buff >= 32768)
  {
    torque = torque_buff - 65535;
  }
  else
  {
    torque = torque_buff;
  }
  toMasterData.angleData = angle;
  toMasterData.velocityData = velocity;
  toMasterData.torqueData = torque;
}

// Torque Control
void torqueControl(int trq)
{
  canId = 200;
  if (trq > 3000)
    trq = 3000;
  if (trq < -3000)
    trq = -3000;

  switch (fromMasterData.torque_patternData)
  {
  case 0:
    data[0] = 0 >> 8 & 0xFF;
    data[1] = 0 & 0xFF;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    sndStat = CAN0.sendMsgBuf(0x200, TYPE, DLC, data);
    break;

  case 1:
    // if( millis() - trq_cnt > 10000 ) torque_pattern = 0;
    data[0] = -3000 >> 8 & 0xFF;
    data[1] = -3000 & 0xFF;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    sndStat = CAN0.sendMsgBuf(0x200, TYPE, DLC, data);
    break;

  case 2:
    data[0] = trq >> 8 & 0xFF;
    data[1] = trq & 0xFF;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    sndStat = CAN0.sendMsgBuf(0x200, TYPE, DLC, data);
    break;
  }
  if (sndStat == CAN_OK)
  {
    M5.dis.drawpix(0, 0x0000ff);
  }
  else
  {
    M5.dis.drawpix(0, 0x00ff00);
  }
}

void init_esp()
{
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() == ESP_OK)
  {
    Serial.printf("\n\n Success for init ESP NOW \n\n");
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("\n Failed to add peer1\n");
    M5.dis.drawpix(0, 0x00ff00);
    delay(50);
    M5.dis.drawpix(0, 0x000000);
    delay(50);
    M5.dis.drawpix(0, 0x00ff00);
    delay(50);
    M5.dis.drawpix(0, 0x000000);
    delay(50);
  }
  else
  {
    Serial.println("\n Success to add peer1\n");
    M5.dis.drawpix(0, 0xb1ff00);
    delay(50);
    M5.dis.drawpix(0, 0x000000);
    delay(50);
    M5.dis.drawpix(0, 0xb1ff00);
    delay(50);
    M5.dis.drawpix(0, 0x000000);
    delay(50);
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
  M5.Lcd.print("Last Packet Sent to: ");
  M5.Lcd.println(macStr);
  M5.Lcd.print("Last Packet Send Status:\t");
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
    M5.dis.drawpix(0, 0xf2a21e);
    torqueControl(-2000);  //EMG STOP
    init_esp();
  }
}

// callback function that will be executed when data is received
//------------------------------------------------------------------//
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&fromMasterData, incomingData, sizeof(fromMasterData));
  /*
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  M5.Lcd.println(macStr);
  */
  torqueControl(fromMasterData.Command);
}
