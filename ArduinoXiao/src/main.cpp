#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Update.h>
#include <aWOT.h>
//#include <WiFiServer.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <ESP32-TWAI-CAN.hpp>
#include <HardwareSerial.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <LittleFS.h>
// FreeRTOS Includes
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//byte mac[] = {
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
//};

void readFileFS(const char* filename); 

// Xiao SPI PIN remapping SCK, MISO, MOSI, SS
#define SCK_PIN    D8
#define MISO_PIN   D9
#define MOSI_PIN   D10
#define SS_PIN     D0

static const int spiClk = 1000000; 

static uint8_t mac[6];

IPAddress WindowsIP(192, 168, 1, 100);
IPAddress localIP(192, 168, 1, 101);
IPAddress remoteIP = IPAddress();
static const char* hostname= "akbarge";

static uint16_t localPort = 8888;      // local port to listen on
static uint16_t remotePort;
static uint16_t WindowsPort = 9999;


// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char replyBuffer[512] = "ACK";        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP udpRx;
EthernetUDP udpTx;

typedef enum {
  disable,
  enable,
  connected,
  disconnected,
  error
} Status_t;

struct netStatus {
  Status_t enWiFiSTA = disable;
  Status_t enWiFiAP  = disable;
  Status_t enEth     = disable;
} netsta;

/***** FreeRTOS util defines *****/
#define DELAY1 250
#define DELAY2 500

#define USING_RTOS_TASKS    0
#define USING_RTOS_MUTEX    0

#if USING_RTOS_TASKS
QueueHandle_t queue;
#define QUEUE_MAX_ITEMS     10      // Max items that the queue can hold  

// Define an enumerated type used to identify the source of the data.
typedef enum {
  device1 = 1,
  device2,
  device3,
  device4
} DataSource_t;

// Define the structure type that will be passed on the queue. 
typedef struct {
  int32_t value;
  const char* msg;
  DataSource_t dataSource;
  DataSource_t dataDestine;
} Data_t;

// Declare two variables of type Data_t that will be passed on the queue.
static Data_t xStructsToSend[ 2 ] = {
  {100, "TASK1", device1, device2}, 
  {200, "TASK2", device1, device2} 
};

// Task prototypes
void writerTask1(void *arg); 
void writerTask2(void *arg); 
void readerTask(void *arg);
#endif

#if USING_RTOS_MUTEX
xSemaphoreHandle sLock;
#define SERIAL_LOCK()    do {} while (xSemaphoreTake(sLock, portMAX_DELAY) != pdPASS)
#define SERIAL_UNLOCK()  xSemaphoreGive(sLock)
#else
#define SERIAL_LOCK()
#define SERIAL_UNLOCK()
#endif

/******* Hardware Defines *******/ 

// Serial RS-232
#define RX_PIN D7
#define TX_PIN D6
#define BAUD 115200

/****** Function prototypes ******/


/****** Global variables ******/

// Webserver
const char* host = "AKBarge";
const char* ssidAP = "AKBarge";
const char* passwordAP = "";

//WiFiServer server(80);
Application WiFi_WebApp;

WebServer server(80);  // Create a web server on port 80 (HTTP)

char expectHeader[20] {};
bool shouldRestart = false;

// Hardware
HardwareSerial SerialRS232(0);               // UART (RS-232)
// I2C
// CAN

/****** Files System (move) ******/
#define SAV_MAX_SIZE_FS  (1048576)           // 1MB = 1048576, max 1500000.
const char *fLogin  = "/server/login.html";
const char *fIndex = "/server/index.html";
const char *fUpdate = "/server/update.html";
const char *fUpload = "/server/upload.html";
const char *fSave = "/save/stw_3cs.hex";
bool fAppend = false;

/****** Real Time Application Tasks ******/
#if USING_RTOS_TASKS
// Inbox for system's sensor messages. React to inputs and changes
// to the system state as communicated in to the qeueue.
void readerTask(void *pvParameters ) {
    // Receive sensor data
    Data_t sensorx;

    // Sets itself to Block state when no adata in the queue.
    while(1){
     if (xQueueReceive(queue, &sensorx, (TickType_t)0))
     { 
        
        Serial.println("Source:" + String(sensorx.dataSource)
                    + " Dest:"   + String(sensorx.dataDestine) 
                    + " Msg: "   + String(sensorx.msg)
                    + " Value: " + String(sensorx.value));
        
        // take action or update data structure based on message source/destin.        
     }
    }
}

// Task 1: Read data from a sensor and update system
void writerTask1(void *pvParameters ) {
  // Collect data from sensor 1
  Data_t message;
  message.value = 1111;
  message.msg = "TASK1";
  message.dataSource = device1;
  message.dataDestine = device2;

  while (1) {
    xQueueSend(queue, &message, (TickType_t)0);
    vTaskDelay(1000/ portTICK_RATE_MS);    
  }
}

// Task 2: Read data from a sensor and update system
void writerTask2(void *pvParameters ) {
  // Collect data from sensor 2
  Data_t message;
  message.value = 2222;
  message.msg = "TASK2";
  message.dataSource = device2;
  message.dataDestine = device1;
  
  while (1) {
    xQueueSend(queue, &message, (TickType_t)0);
    vTaskDelay(1000/ portTICK_RATE_MS);
  }
}

// Task 2: Read data from a sensor and update system
void writerTask3(void *pvParameters ) {
  // Collect data from sensor 2
  Data_t message;
  message.value = 3333;
  message.msg = "TASK3";
  message.dataSource = device2;
  message.dataDestine = device1;
  
  while (1) {
    xQueueSend(queue, &message, (TickType_t)0);
    vTaskDelay(1000/ portTICK_RATE_MS);
  }
}

// Add more tasks here
#endif

/****** loop tasks ******/

void heartBeatTask(void *pvParameters ) 
{
  while (1) {
    // LED
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void initWifiAP()
{
    WiFi.softAP(ssidAP, passwordAP);
    Serial.print("WiFi AP IP address: ");
    Serial.println(WiFi.softAPIP());

    if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
         delay(1000);
    }
  }
  Serial.println("mDNS responder started");    
}


void hServerLogin() 
{
  server.sendHeader("Connection", "close");  
  // inline read file
  File file = LittleFS.open(fLogin, "r");
  if (!file) {
    Serial.println("could not open file for reading");
  } else {
    while (file.available()) {      
      server.send(200, "text/html", file.readString());         
    }
    file.close();
  }
}

void hServerIndex() 
{
  server.sendHeader("Connection", "close");
  // inline read file
  File file = LittleFS.open(fIndex, "r");
  if (!file) {
    Serial.println("could not open file for reading");
  } else {
    while (file.available()) {      
      server.send(200, "text/html", file.readString());         
    }
    file.close();
  }
}

void hServerUpdate() 
{
  server.sendHeader("Connection", "close");
   // inline read file
  File file = LittleFS.open(fUpdate, "r");
  if (!file) {
    Serial.println("could not open file for reading");
  } else {
    while (file.available()) {      
      server.send(200, "text/html", file.readString());         
    }
    file.close();
  }
}

void hServerUpdatedEnd() 
{
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}

void hServerUpdatedStart() 
{
  HTTPUpload& upload = server.upload();
  Serial.println("Updating..");
 
  if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/      
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
      }
  } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
  }
}


void hServerUpload() 
{
  server.sendHeader("Connection", "close");
   // inline read file
  File file = LittleFS.open(fUpload, "r");
  if (!file) {
    Serial.println("could not open file for reading");
  } else {
    while (file.available()) {      
      server.send(200, "text/html", file.readString());         
    }
    file.close();
  }
}

void hServerUploadEnd() 
{
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  // Verify the file was written
  readFileFS(fSave);
  //writeFileUdp(fSave);  // This may be CAN instead
}

// Attempt to delete the file
bool deleteFileFS(const char* filePath) 
{
  if (LittleFS.exists(filePath)) {    
    if (LittleFS.remove(filePath)) {
      Serial.println("File deleted successfully");
      return true;
    } else {
      Serial.println("Failed to delete the file.");
    }
  } else {
    Serial.println("File does not exist.");
  }
  return false;
}

// upload a file in to a flash file that is a multiple of 1436 bytes buffer 
void hServerUploadStart() 
{
  HTTPUpload& upload = server.upload();
  static size_t accStream = 0;
  static size_t fStreamSize = 0;

  // delete old file and clear upload buffer
  if (!fAppend) {
      deleteFileFS(fSave); 
      fStreamSize = 0;   
      accStream = 0;
      fAppend = true;      
  }

  File file = LittleFS.open(fSave, "a");
  
  if (!file) {
      Serial.println("could not open file for writting");
      return;
  }
  
  if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Uploading: %s\n", upload.filename.c_str());
  } else if (upload.status == UPLOAD_FILE_WRITE) {
      // The actual usefull data in the file will be upload.totalSize
      int bufferSize = sizeof(upload.buf);
      accStream += bufferSize;

      // Dont try to write beyond available file size.
      if (accStream > SAV_MAX_SIZE_FS) {
          Serial.println("Upload file size " + String(accStream) + " larger than flash space " + String(SAV_MAX_SIZE_FS));          
          // should we delete the partial file?
          file.close();     
          upload.status = UPLOAD_FILE_ABORTED;           
          return;
      } else {
          // May have to sanitize the last buffer upload.buf
          fStreamSize += file.write((const uint8_t*)upload.buf, bufferSize);
          memset(upload.buf, '\0', sizeof(upload.buf));          
      }

      Serial.println("Upload buffer written to file, file size: " + String(fStreamSize));

  } else if (upload.status == UPLOAD_FILE_END) {
      Serial.printf("Upload of %u bytes completed \n", fStreamSize);
      fAppend = false;
      fStreamSize = 0;   
      accStream = 0;           
      file.close();      
  }
}


void initWebServer()
{
  server.on("/", HTTP_GET, hServerLogin);
  server.on("/index", HTTP_GET, hServerIndex);  // There is no /index until there is a portal 
  
  // OTA Flash Update
  server.on("/update", HTTP_GET, hServerUpdate);
  server.on("/flash", HTTP_POST, hServerUpdatedEnd, hServerUpdatedStart);

  // Upload a file and save to file-system
  server.on("/upload", HTTP_GET, hServerUpload);
  server.on("/save", HTTP_POST, hServerUploadEnd, hServerUploadStart);
  server.begin();
}


void initSPI()
{
  // Manual pin reasignment for Xiao - clk, miso, mosi, ss
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
// Set Slave Select mode.
  pinMode(SPI.pinSS(),OUTPUT);

  /*
  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS); 
  Serial.print("SPI SS: GPIO");
  Serial.println(SPI.pinSS());  
  */
}

void spiCommand(byte data) 
{
  //use it as you would the regular arduino SPI API
  SPI.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI.pinSS(), LOW);   //pull SS slow to prep other end for transfer
  SPI.transfer(data);
  digitalWrite(SPI.pinSS(), HIGH);  //pull ss high to signify end of data transfer
  SPI.endTransaction();
}

void initEth() 
{
  esp_efuse_mac_get_default(mac);
  Ethernet.init(SPI.pinSS());
  Ethernet.begin(mac, localIP);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet device not found. Can't run without hardware.");
      netsta.enEth = error;
      return;
  } else {
      netsta.enEth = enable;
  }
  if (Ethernet.linkStatus() == LinkON) {
      netsta.enEth = connected;
      Serial.print("Ethernet Started on Gateway:");  
      Serial.print(Ethernet.gatewayIP());
      Serial.print("and IP:");
      Serial.println(Ethernet.localIP());
  } else {
      Serial.println("Ethernet cable is not connected.");
      netsta.enEth = disconnected;
  }

  // start UDP
  udpRx.begin(localPort);
  udpTx.begin(remotePort);
  Serial.print("UDP service on:");  
  Serial.println(udpRx.localPort());

  // Give device a hostname so webpage can be easier to access
  if (!MDNS.begin(hostname)) {                          
      Serial.println("Error starting mDNS \n");
  } else {
      Serial.println( "Access " + String(hostname) + ".local/ or the IP address into a browser to access portal. \n");
  }
}


void readUdp() 
{
  // if there's data available, read a packet
  int packetSize = udpRx.parsePacket();       
      
  if ((netsta.enEth == connected) && packetSize) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");

      // Identify remote IP
      remoteIP = udpRx.remoteIP();
      for (int i=0; i < 4; i++) {
           Serial.print(remoteIP[i], DEC);
           if (i < 3) Serial.print(".");
      }

      // Identify remote port
      Serial.print(", port ");
      remotePort = udpRx.remotePort();
      Serial.println(remotePort);

      // read the packet into packetBuffer
      udpRx.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      Serial.print("Contents: ");
      Serial.println(packetBuffer);
      
      // Place packet in Mailbox?
  } 
}

void writeUdp(IPAddress remoteIP, uint16_t remotePort, const char* tBuffer) 
{
      // send a reply to the IP address and port that sent us the packet we received
      Serial.print("Sending UDP msg to: ");
      Serial.print(remoteIP);
      Serial.print(" Port: ");
      Serial.println(remotePort);
      
      if (!udpTx.beginPacket(remoteIP, remotePort)) {
          Serial.println("Remote IP/Port error.");
      } 

      udpTx.write(tBuffer);

      if (!udpTx.endPacket()) {
          Serial.println("UDP packet sent error.");
      }
}


// The file size may be larger than the data in it.
void readFileFS(const char* filename) 
{
   // inline read file
  File file = LittleFS.open(filename, "r");
  int size = file.size();
  //char buffer[dataLimit];
  size_t bytesRead = 0;

  Serial.println("Reading flashed file:" + String(filename) + " of size:" + String(size));

  if (!file) {
    Serial.println("could not open file for reading");
  } else {      
      while (file.available()) {             
        Serial.printf("%X ", file.read());         // Process file here CAN/UDP    
        bytesRead++;          
        //if (bytesRead > SAV_MAX_SIZE_FS) break;        
        //bytesRead = file.readBytes(buffer, sizeof(buffer)-1);             
        //Serial.println(file.readString());
        //for (int i=0; i<(bytesRead-1); i++) {
        //    Serial.printf("%X", buffer[i]); 
        //    if (!(i % 31)) Serial.println();           
        //}        
        // yield();
      }
    Serial.println("Amount of data in file:" + String(bytesRead));    
    file.close();
  }
}


void mountFS() 
{
    if(!LittleFS.begin()){
        Serial.println("An Error has occurred while mounting SPIFFS");
    } else {
        Serial.println("LittleFS file system mounted");
    }                
}



// void initFS() {
//   File file = LittleFS.open(fLogin, "r");
//   if (!file) {
//     Serial.println("could not open file for reading");
//   } else {
//     while (file.available()) {
//       char buf[32];
//       size_t bytesRead = file.readBytes(buf, sizeof(buf) - 1);
//       buf[bytesRead] = '\0';
//       Serial.print(buf);
//       yield();
//     }
//     file.close();
//   }
//   LittleFS.end();
// }

/****** setup tasks ******/

void setup() 
{
  Serial.begin(460800);
  delay(1000);
  Serial.println("Setup started.");
  
  mountFS();
  
  
  // On Board LED heatbeat
  pinMode(LED_BUILTIN, OUTPUT);
  //initSPI();
  
  // Set UART for RS-232 interface
  // Configure MySerial0 on pins TX=D6 and RX=D7 
  // SerialRS232.onReceive(); TODO: set onRecive callback so no need to add in the polling loop)
  SerialRS232.begin(BAUD, SERIAL_8N1,RX_PIN,TX_PIN);
  
  // Connect to WiFi network
  initWifiAP();
  initWebServer();
  //initEth();
  readFileFS(fSave);


  

#if USING_RTOS_MUTEX
  sLock = xSemaphoreCreateMutex();
  if(sLock == NULL) {
    while(1) {
      Serial.println("\n------------\nSerial Lock Failure\n------------\n");
      delay(10000);
    }
  }
#endif

#if USING_RTOS_TASKS
  // Create queue for various senders
  queue = xQueueCreate(QUEUE_MAX_ITEMS, sizeof(Data_t)); 

  if (queue == 0) printf("Failed to create queue.\n");
  
  // Task Function, Task Name, Stack Size, Task Param, Priority, task handle  
  xTaskCreate(readerTask, "Mailbox" , 2048, NULL, 2, NULL);
  xTaskCreate(writerTask1,"Writer 1", 2048, NULL, 1, NULL);
  xTaskCreate(writerTask2,"Writer 2", 2048, NULL, 1, NULL);
  xTaskCreate(writerTask3,"Writer 3", 2048, NULL, 1, NULL);
  
  // Create more tasks here.
  
#endif

  xTaskCreate(heartBeatTask,"LED", 2048, NULL, 1, NULL);
}

/****** loop tasks ******/
void loop() {

  server.handleClient();
  //readUdp();
  //delay(2000);
  //writeUdp(WindowsIP, WindowsPort, replyBuffer);
  //delay(2000);
  //spiCommand(0xBE);

  
  if (shouldRestart) {
      delay(1000);
      ESP.restart();
  }

}
