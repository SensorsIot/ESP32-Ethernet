#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ETH.h>

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER 12

int request = 0;
char *payloadString;

#define inTopic "Ethernet/command"
#define outTopic "Ethernet/status"

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

int led1 = 32;
int led2 = 33;
unsigned long entry;


int LED = 02;

// #define ETH_CLK_MODE    ETH_CLOCK_GPIO0_OUT          // Version with PSRAM
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT            // Version with not PSRAM

// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN   -1

// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE        ETH_PHY_LAN8720

// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR        0

// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN     23

// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN    18

#define NRST            5

// CHANGE THESE SETTINGS FOR YOUR APPLICATION
const char* mqttServerIp = "192.168.0.203"; // IP address of the MQTT broker
const short mqttServerPort = 1883; // IP port of the MQTT broker
const char* mqttClientName = "ESP32-POE";
const char* mqttUsername = "admin";
const char* mqttPassword = "admin";

// Initializations of network clients
WiFiClient espClient;
PubSubClient mqttClient(espClient);
static bool eth_connected = false;
uint64_t chipid;

//ESP32 Specific Ethernet
//Yes, I know it says wifi.  Trust me.
void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    payload[length] = '\0';
  }
  Serial.println(payloadString);
  Serial.println(request);
  Serial.println(topic);


  /*  // This snippet sets all the setpoints at once for the whole arduino
    if (strstr(topic, "setall") != NULL) {

    }
  */
  if (strstr(topic, "command") != NULL) {
    payloadString = (char *) payload;
    Serial.println("The request is...");
    Serial.println(payloadString);
    if (strstr(payloadString, "1on") != NULL) {
      Serial.println("Turning led 1 on");
      digitalWrite(led1, HIGH);
    }
    if (strstr(payloadString, "1off") != NULL) {
      Serial.println("Turning led 1 off");
      digitalWrite(led1, LOW);
    }
    if (strstr(payloadString, "2on") != NULL) {
      Serial.println("Turning led 2 on");
      digitalWrite(led2, HIGH);
    }
    if (strstr(payloadString, "2off") != NULL) {
      Serial.println("Turning led 2 off");
      digitalWrite(led2, LOW);
    }

  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(mqttClientName)) {
    //if (mqttClient.connect(mqttClientName, mqttUsername, mqttPassword)) { // if credentials is nedded
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish("random/test", "hello world");
      // ... and resubscribe
      mqttClient.subscribe("leds/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    mqttClient.publish(outTopic,  "connected to MQTT");
    mqttClient.subscribe(inTopic);
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);

  mqttClient.setServer(mqttServerIp, mqttServerPort);
  mqttClient.setCallback(callback);

  chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32)); //print High 2 bytes
  Serial.printf("%08X\n", (uint32_t)chipid); //print Low 4bytes.

  // Init sensor
  Wire.begin (14, 15);  // init I2C on the respective pins
  unsigned status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  WiFi.onEvent(WiFiEvent);
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
}

void loop() {
  // check if ethernet is connected
  if (eth_connected) {
    // now take care of MQTT client...
    if (!mqttClient.connected()) {
      reconnect();
    } else {
      mqttClient.loop();

    }
  }
  if (millis() > entry + 5000) {
    float temp = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    float pressure = bme.readPressure() / 100.0F;
    Serial.print(pressure);
    Serial.println(" hPa");

    float  alti = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("Approx. Altitude = ");
    Serial.print(alti);
    Serial.println(" m");

    float hum = bme.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(hum);
    Serial.println(" %");

    Serial.println();
    char message_buff[50];
    String payloadStr = "T " + String(temp, 1) + " H " + String(hum, 1) + " P " + pressure + " A " + String(alti, 1);
    payloadStr.toCharArray(message_buff, payloadStr.length() + 1);

    mqttClient.publish (outTopic, message_buff);
    entry = millis();
  }
}
