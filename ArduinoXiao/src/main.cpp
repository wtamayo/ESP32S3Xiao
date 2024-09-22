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
char replyBuffer[] = "ACK";        // a string to send back

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

WebServer wserver(80);  // Create a web server on port 80 (HTTP)

char expectHeader[20] {};
bool shouldRestart = false;

// Hardware
HardwareSerial SerialRS232(0);     // UART (RS-232)
// I2C
// CAN

/****** Files System (move) ******/

const char *fLogin  = "/server/login.html";
const char *fUpload = "/server/upload.html";
const char *fController = "/stw/samplestw.hex";



String style =
"<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
"input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
"#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
"#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
"form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
".btn{background:#3498db;color:#fff;cursor:pointer}</style>";

/*
String loginIndex = 
"<form name=loginForm>"
"<h1>Couer Barge Login</h1>"
"<input name=userid placeholder='User ID'> "
"<input name=pwd placeholder=Password type=Password> "
"<input type=submit onclick=check(this.form) class=btn value=Login></form>"
"<script>"
"function check(form) {"
"if(form.userid.value=='AKBarge' && form.pwd.value=='admin')"
"{window.open('/serverIndex')}"
"else"
"{alert('Error Password or Username')}"
"}"
"</script>" + style;

String serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
"<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
"<label id='file-input' for='file'>   Choose file...</label>"
"<input type='submit' class=btn value='Update'>"
"<br><br>"
"<div id='prg'></div>"
"<br><div id='prgbar'><div id='bar'></div></div><br></form>"
"<script>"
"function sub(obj){"
"var fileName = obj.value.split('\\\\');"
"document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
"};"
"$('form').submit(function(e){"
"e.preventDefault();"
"var form = $('#upload_form')[0];"
"var data = new FormData(form);"
"$.ajax({"
"url: '/update',"
"type: 'POST',"
"data: data,"
"contentType: false,"
"processData:false,"
"xhr: function() {"
"var xhr = new window.XMLHttpRequest();"
"xhr.upload.addEventListener('progress', function(evt) {"
"if (evt.lengthComputable) {"
"var per = evt.loaded / evt.total;"
"$('#prg').html('progress: ' + Math.round(per*100) + '%');"
"$('#bar').css('width',Math.round(per*100) + '%');"
"}"
"}, false);"
"return xhr;"
"},"
"success:function(d, s) {"
"console.log('success!') "
"},"
"error: function (a, b, c) {"
"}"
"});"
"});"
"</script>" + style;
*/

String uploadIndex = 
"<body>"
//"<h1 align='center' color='white'> Upload fimrware update file (.bin) </h1>"
"<form class='form' id='myForm'>"
"<h1>Upload .bin Update File</h1>"
"<input type='file' id='inpFile'><br>"
"<button type='submit'>Upload File</button>"
"</form>"

"<script>"
"const myForm = document.getElementById('myForm');"
"const inpFile = document.getElementById('inpFile');"

"myForm.addEventListener('submit', e => {"
"e.preventDefault();"

"const endpoint = '/update';"
"const formData = new FormData();"
"console.log(inpFile.files);"

"formData.append('inpFile', inpFile.files[0]);"

"fetch(endpoint, {"
"method: 'post',"
"body: formData"
"}).then((response) => {"
"if (!response.ok) {"
"return alert('File upload failed');"
"} else {"
"alert('Firmware update completed');"
"location.reload();"
"}"
"});"
"});"
"</script>"
"</body>" + style;



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

void heartBeatTask(void *pvParameters ) {

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


void hServerLogin() {
  wserver.sendHeader("Connection", "close");  
  // inline read file
  File file = LittleFS.open(fLogin, "r");
  if (!file) {
    Serial.println("could not open file for reading");
  } else {
    while (file.available()) {      
      wserver.send(200, "text/html", file.readString());         
    }
    file.close();
  }
}

void hServerIndex() {
  wserver.sendHeader("Connection", "close");
  wserver.send(200, "text/html", fUpload);
}

void hServerUpdate() {
  wserver.sendHeader("Connection", "close");
  wserver.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}

void hServerUpload() {
  HTTPUpload& upload = wserver.upload();
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

void initWebServer()
{
  wserver.on("/", HTTP_GET, hServerLogin); 
  wserver.on("/index", HTTP_GET, hServerIndex);
  wserver.on("/update", HTTP_POST, hServerUpdate, hServerUpload);
  wserver.begin();
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

void mountFS() 
{
    if(!LittleFS.begin()){
        Serial.println("An Error has occurred while mounting SPIFFS");
    } else {
        Serial.println("LittleFS file system mounted");
    }                
}


void initFS() {

  File file = LittleFS.open(fLogin, "r");

  if (!file) {
    Serial.println("could not open file for reading");
  } else {
    while (file.available()) {
      char buf[32];
      size_t bytesRead = file.readBytes(buf, sizeof(buf) - 1);
      buf[bytesRead] = '\0';
      Serial.print(buf);
      yield();
    }

    file.close();
  }

  LittleFS.end();

}

/****** setup tasks ******/

void setup() 
{
  Serial.begin(460800);
  delay(5000);
  Serial.println("Setup started.");
  
  mountFS();
  
  // On Board LED heatbeat
  pinMode(LED_BUILTIN, OUTPUT);
  initSPI();
  
  // Set UART for RS-232 interface
  // Configure MySerial0 on pins TX=D6 and RX=D7 
  // SerialRS232.onReceive(); TODO: set onRecive callback so no need to add in the polling loop)
  SerialRS232.begin(BAUD, SERIAL_8N1,RX_PIN,TX_PIN);
  
  // Connect to WiFi network
  initWifiAP();
  initWebServer();
  initEth();


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

  /*
  WiFiClient client = server.available();
    
  if (client.connected()) {
      WiFi_WebApp.process(&client);
      client.stop();
  }
  */

  wserver.handleClient();
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
