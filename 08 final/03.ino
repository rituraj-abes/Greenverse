//AirBuddi1
//SPM1

#include "bsec.h"
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "FreeRTOS.h"
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "esp_mac.h"
#include <Preferences.h>

#define AWS_IOT_PUBLISH_TOPIC   "SPM1"
String mac = "";
int connectedFlag = 0;

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

//BME 688
void checkIaqSensorStatus(void);
void errLeds(void);
Bsec iaqSensor;
String output;
int LED_BUILTIN = 2;

//HPMA
long lastMsg = 0;
char msg[50];
bool HPMAstatus = false;
int zedLevel = 1;
int PM25;
int PM10;
int oldP;
HardwareSerial HPMA115S0(1);
#define RXD2 16
#define TXD2 17
bool start_autosend(void);
bool receive_measurement(void);
//RGB
#define LED_PIN 13
#define LED_COUNT 16
int mynumb;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//LED matrix parameter
int pixelInterval = 50;
uint16_t pixelCurrent = 0;
uint16_t pixelNumber = LED_COUNT;
//LCD
unsigned char a[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X20, 0X00, 0X00  //Serial output prefix
};
unsigned char b[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X21, 0X00, 0X00  //Serial output prefix
};
unsigned char c[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X22, 0X00, 0X00  //Serial output prefix
};
unsigned char d[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X23, 0X00, 0X00  //Serial output prefix
};
unsigned char e[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X24, 0X00, 0X00  //Serial output prefix
};
unsigned char f[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X25, 0X00, 0X00  //Serial output prefix
};
unsigned char k[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X26, 0X00, 0X00  //Serial output prefix
};
unsigned char l[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X27, 0X00, 0X00  //Serial output prefix
};
unsigned char m[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X28, 0X00, 0X00  //Serial output prefix
};
float pressure;
float temperature;
float Humidity;
float Gasresistance;
float IAQ;
float Co2;
float Vocs;

void bmeTask(void *pvParameters);
void hpmaTask(void *pvParameters);
void ledTask(void *pvParameters);
void displayTask(void *pvParameters);
void WifiManagerTask(void *pvParameters);
void AWSTask(void *pvParameters);

// Task handles
TaskHandle_t bmeTaskHandle;
TaskHandle_t hpmaTaskHandle;
TaskHandle_t ledTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t WifiManagerTaskHandle;
TaskHandle_t AWSTaskHandle;

void connectAWS()
{
  Serial.println("Connecting to Wi-Fi");
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME)){
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }
  
  Serial.println("AWS IoT Connected!");
}

//BME 688
void checkIaqSensorStatus(void)
{
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}


bool start_autosend(void) {
  // Start auto send
  byte start_autosend[] = { 0x68, 0x01, 0x40, 0x57 };
  HPMA115S0.write(start_autosend, sizeof(start_autosend));
  HPMA115S0.flush();
  delay(500);
  //Then we wait for the response
  while (HPMA115S0.available() < 2);
  byte read1 = HPMA115S0.read();
  byte read2 = HPMA115S0.read();
  // Test the response
  if ((read1 == 0xA5) && (read2 == 0xA5)) {
    // ACK
    return true;
  } 
  else if ((read1 == 0x96) && (read2 == 0x96)) {
    // NACK
    return false;
  } 
  else {
    return false;
  }
}

bool receive_measurement(void) {
  while (HPMA115S0.available() < 32);
  byte HEAD0 = HPMA115S0.read();
  byte HEAD1 = HPMA115S0.read();
  while (HEAD0 != 0x42) {
    if (HEAD1 == 0x42) {
      HEAD0 = HEAD1;
      HEAD1 = HPMA115S0.read();
    } 
    else {
      HEAD0 = HPMA115S0.read();
      HEAD1 = HPMA115S0.read();
    }
  }
  if (HEAD0 == 0x42 && HEAD1 == 0x4D) {
    byte LENH = HPMA115S0.read();
    byte LENL = HPMA115S0.read();
    byte Data0H = HPMA115S0.read();
    byte Data0L = HPMA115S0.read();
    byte Data1H = HPMA115S0.read();
    byte Data1L = HPMA115S0.read();
    byte Data2H = HPMA115S0.read();
    byte Data2L = HPMA115S0.read();
    byte Data3H = HPMA115S0.read();
    byte Data3L = HPMA115S0.read();
    byte Data4H = HPMA115S0.read();
    byte Data4L = HPMA115S0.read();
    byte Data5H = HPMA115S0.read();
    byte Data5L = HPMA115S0.read();
    byte Data6H = HPMA115S0.read();
    byte Data6L = HPMA115S0.read();
    byte Data7H = HPMA115S0.read();
    byte Data7L = HPMA115S0.read();
    byte Data8H = HPMA115S0.read();
    byte Data8L = HPMA115S0.read();
    byte Data9H = HPMA115S0.read();
    byte Data9L = HPMA115S0.read();
    byte Data10H = HPMA115S0.read();
    byte Data10L = HPMA115S0.read();
    byte Data11H = HPMA115S0.read();
    byte Data11L = HPMA115S0.read();
    byte Data12H = HPMA115S0.read();
    byte Data12L = HPMA115S0.read();
    byte CheckSumH = HPMA115S0.read();
    byte CheckSumL = HPMA115S0.read();
    if (((HEAD0 + HEAD1 + LENH + LENL + Data0H + Data0L + Data1H + Data1L + Data2H + Data2L + Data3H + Data3L + Data4H + Data4L + Data5H + Data5L + Data6H + Data6L + Data7H + Data7L + Data8H + Data8L + Data9H + Data9L + Data10H + Data10L + Data11H + Data11L + Data12H + Data12L) % 256) != CheckSumL) {
      Serial.println("Checksum fail");
      return false;
    }
    PM25 = (Data1H * 256) + Data1L;
    PM10 = (Data2H * 256) + Data2L;
    return true;
  }
}

void colorWipe(uint32_t color, int wait) {
  if (pixelInterval != wait)
    pixelInterval = wait;                    //  Update delay time
  strip.setPixelColor(pixelCurrent, color);  //  Set pixel's color (in RAM)
  strip.show();                              //  Update strip to match
  pixelCurrent++;                            //  Advance current pixel
  if (pixelCurrent >= pixelNumber)           //  Loop the pattern from the first LED
    pixelCurrent = 0;
}

String getDefaultMacAddress() {
  unsigned char mac_base[6] = {0};
  if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
    char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    mac = buffer;
  }
  return mac;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
     Serial.println(getDefaultMacAddress());
 
  // Create tasks
  xTaskCreate(displayTask, "Display Task", 1024, NULL, 1, &displayTaskHandle);
  xTaskCreate(bmeTask, "BME Task", 2048, NULL, 1, &bmeTaskHandle);
  xTaskCreate(hpmaTask, "HPMA Task", 2048, NULL, 1, &hpmaTaskHandle);
  xTaskCreate(ledTask, "LED Task", 4096, NULL, 1, &ledTaskHandle);
  xTaskCreate(WifiManagerTask, "WifiManager Task", 4096, NULL, 1, &WifiManagerTaskHandle);
  xTaskCreate(AWSTask, "AWS Task", 10000, NULL, 1, &AWSTaskHandle);
}

void publishMessage()
{
  StaticJsonDocument<200> doc;
    doc["id"] = mac;  
    doc["IAQ"] = IAQ;
    doc["Humidity"] = Humidity;
    doc["PM 2.5"] = PM25;
    doc["PM 10"] = PM10;
    doc["Temperature"] = temperature;
    doc["Pressure"] = pressure;
    doc["C02 Equivalent"] = Co2;
    doc["VOC's"]= Vocs;
    doc["Gas Resistance"] = Gasresistance;
   
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
 
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}
 
void loop(){
  
}

// Task functions
void bmeTask(void *pvParameters) {
    pinMode(LED_BUILTIN, OUTPUT);
    iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);
    output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkIaqSensorStatus();
    bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };
  iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
  // Print the header
  output = "Timestamp [ms], IAQ, IAQ accuracy, Static IAQ, CO2 equivalent, breath VOC equivalent, raw temp[°C], pressure [hPa], raw relative humidity [%], gas [Ohm], Stab Status, run in status, comp temp[°C], comp humidity [%], gas percentage";
//  Serial.println(output);
  
  while (1) {
    unsigned long time_trigger = millis();
    if (iaqSensor.run()) { // If new data is available
      digitalWrite(LED_BUILTIN, LOW);
      output = String(time_trigger);
      pressure=iaqSensor.pressure;
      temperature=iaqSensor.temperature;
      Humidity=iaqSensor.humidity;
      Gasresistance=iaqSensor.gasResistance;
      IAQ=iaqSensor.iaq;
      Co2=iaqSensor.co2Equivalent;
      Vocs=iaqSensor.breathVocEquivalent;
      output = String(time_trigger);
      output += ", " + String(iaqSensor.iaq);
      output += ", " + String(iaqSensor.iaqAccuracy);
      output += ", " + String(iaqSensor.staticIaq);
      output += ", " + String(iaqSensor.co2Equivalent);
      output += ", " + String(iaqSensor.breathVocEquivalent);
      output += ", " + String(iaqSensor.rawTemperature);
      output += ", " + String(iaqSensor.pressure);
      output += ", " + String(iaqSensor.rawHumidity);
      output += ", " + String(iaqSensor.gasResistance);
      output += ", " + String(iaqSensor.stabStatus);
      output += ", " + String(iaqSensor.runInStatus);
      output += ", " + String(iaqSensor.temperature);
      output += ", " + String(iaqSensor.humidity);
      output += ", " + String(iaqSensor.gasPercentage);
      Serial.println(output);
      digitalWrite(LED_BUILTIN, HIGH);
    } 
    else {
      checkIaqSensorStatus();
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void hpmaTask(void *pvParameters) {
  HPMA115S0.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!HPMA115S0);
  start_autosend();
   
  while (1) {

    HPMAstatus = receive_measurement();
    if (!HPMAstatus) {
      
      Serial.println("Cannot receive data from HPMA115S0!");
      return;
    }
    snprintf (msg, 16, "%D", PM25);
    
    snprintf (msg, 16, "%D", PM10);
    if(PM10 != 0){
  
    Serial.println("PM 2.5:\t" + String(PM25) + " ug/m3");
    Serial.println("PM 10:\t" + String(PM10) + " ug/m3");
    
    } 
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void ledTask(void *pvParameters) {
  while (1) {
    int lm = 0;  // LED Matrix
    mynumb = PM25;  // Led Matrix indication
  if (mynumb > 500) {

    for (lm = 0; lm < 16; lm++) {
      colorWipe(strip.Color(255, 0, 0), 50);  // red
    }
  }

  else if (100 < mynumb && mynumb < 500) {
    for (lm = 0; lm < 16; lm++) {
      colorWipe(strip.Color(0, 0, 255), 50);  // Blue
    }
  }
  else if (mynumb < 100) {
    for (lm = 0; lm < 16; lm++) {
      colorWipe(strip.Color(0, 255, 0), 50);  // Green
    }
  }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void displayTask(void *pvParameters) {
  while (1) {
    long g;
    long h;
    long i;
    long j;

    g = Gasresistance;
    i = g >> 8;
    j = g >> 16;
    Serial.write(a, 7);
    Serial.write(j);
    Serial.write(i & 0x0000FF);
    Serial.write(g & 0x0000FF);

    g = pressure;
    i = g >> 8;
    j = g >> 16;
    Serial.write(b, 7);
    Serial.write(j);
    Serial.write(i & 0x0000FF);
    Serial.write(g & 0x0000FF);

    g = temperature;
    i = g >> 8;
    j = g >> 16;
    Serial.write(c, 7);
    Serial.write(j);
    Serial.write(i & 0x0000FF);
    Serial.write(g & 0x0000FF);

    g = Humidity;
    i = g >> 8;
    j = g >> 16;
    Serial.write(d, 7);
    Serial.write(j);
    Serial.write(i & 0x0000FF);
    Serial.write(g & 0x0000FF);

    g = IAQ;
    i = g >> 8;
    j = g >> 16;
    Serial.write(e, 7);
    Serial.write(j);
    Serial.write(i & 0x0000FF);
    Serial.write(g & 0x0000FF);

    g = Co2;
    i = g >> 8;
    j = g >> 16;
    Serial.write(f, 7);
    Serial.write(j);
    Serial.write(i & 0x0000FF);
    Serial.write(g & 0x0000FF);

    float vocs = Vocs;
    int16_t vocs_int = (int16_t)(vocs * 100); // Assuming 2 decimal places precision
    i = vocs_int >> 8;
    j = vocs_int >> 16;
    Serial.write(k, 7);
    Serial.write(j);
    Serial.write(i & 0x00FF);
    Serial.write(vocs_int & 0x00FF);

    g = PM25;
    i = g >> 8;
    j = g >> 16;
    Serial.write(l, 7);
    Serial.write(j);
    Serial.write(i & 0x0000FF);
    Serial.write(g & 0x0000FF);

    g = PM10;
    i = g >> 8;
    j = g >> 16;
    Serial.write(m, 7);
    Serial.write(j);
    Serial.write(i & 0x0000FF);
    Serial.write(g & 0x0000FF);
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

//wifi task
void WifiManagerTask(void *pvParameters){
    WiFi.mode(WIFI_STA);
    WiFiManager wm;

    bool connected = false;
    const int reconnectAttempts = 10;
    int attemptCount = 0;
    const int reconnectInterval = 120000;

    // Attempt to connect to previously saved WiFi network
    WiFi.begin();
    while (1) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi not connected, attempting to reconnect...");

            while (WiFi.status() != WL_CONNECTED && attemptCount < reconnectAttempts) {
                Serial.print("Attempt ");
                Serial.println(attemptCount + 1);
                WiFi.begin(); // Attempt to reconnect to the last known network
                attemptCount++;
                vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
            }

            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("Failed to reconnect to WiFi. Starting configuration portal...");
                
                // Start WiFiManager AP for reconfiguration
                wm.startConfigPortal("AutoConnectAP" , "password");
                connected = false;
                
                while (!connected) {
                    Serial.println("Checking for previously saved WiFi network...");
                    WiFi.begin();
                    attemptCount = 0;
                    while (WiFi.status() != WL_CONNECTED && attemptCount < reconnectAttempts) {
                        Serial.print("Attempt ");
                        Serial.println(attemptCount + 1);
                        attemptCount++;
                        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
                    }

                    if (WiFi.status() == WL_CONNECTED) {
                        Serial.println("Reconnected to previously saved WiFi network.");
                        connected = true;
                        wm.stopConfigPortal(); // Stop the configuration portal
                        connectedFlag = 1;
                    } else {
                        Serial.println("Still not connected to previously saved WiFi network.");
                        vTaskDelay(reconnectInterval / portTICK_PERIOD_MS); // Delay for 2 minutes before trying again
                    }
                }
            } 
           
            else {
                Serial.println("Reconnected to WiFi network.");
                attemptCount = 0; // Reset attempt count after successful reconnection
                connectedFlag = 1;
            }
        } 
        
        else {
            // WiFi is connected
            Serial.println("WiFi connected.");
            connectedFlag = 1;
        }

        // Wait before checking the connection status again
//        vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay for 10 seconds
        vTaskDelay(reconnectInterval / portTICK_PERIOD_MS);
    }
}


void AWSTask(void *pvParameters) {
  
  while(!connectedFlag){
     vTaskDelay(1000/portTICK_PERIOD_MS);
  }
  connectAWS();
  while(1){
    if(client.connected()){
          if (isnan(pressure) || isnan(temperature) || isnan(Humidity) ||isnan(Gasresistance) ||isnan(IAQ) ||isnan(Co2) ||isnan(Vocs) )  // Check if any reads failed and exit early (to try again).
          {
            Serial.println(F("Failed to read from sensor!"));
            return;
          }
          publishMessage();
          client.loop();   
    }
    else{
      connectAWS();
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
