/*

Home Assistant MQTT Integration using ESP32 board by MS

- wifi connection to local LAN only (no gateway needed)
- log in with authentication to MQTT Broker

- Subscribes to two mqtt topics to control two relays
- LED reacts to mqtt topic update 
- Buttons press updates mqtt topic to publish 
- AHT20 temp and humidity sensor pubish to seperate mqtt topics 


*/

#include "WiFi.h"
#include <PubSubClient.h>
#include <Adafruit_AHTX0.h>
 
Adafruit_AHTX0 aht;
WiFiClient espClient;

const char* ssid = "your wifi name";
const char* password =  "wifi password";
const char* mqtt_server = "mqtt broker ip adress";

PubSubClient client(espClient);

// relay 1 pinout
int relay1 = 12;
int relay1LED = 15;
String relay1_switch;

//relay 2 pinout
int relay2 = 13;
int relay2LED = 16;
String relay2_switch;

String strTopic;
String strPayload;

// BUTTONs setup
/* 
BUTTON 1 
*/
#define RELAY_BUTTON_1 14         // declare PIN
int button1_state;       // the current state of button
int last_button1_state;  // the previous state of button
int button1Count = 0;

/* 
BUTTON 2 
*/
#define RELAY_BUTTON_2 17        // declare PIN
int button2_state;       // the current state of button
int last_button2_state;  // the previous state of button
int button2Count = 0;

/// need checking   //////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  strTopic = String((char*)topic);
  // add topic and set checker
  
  
  if(strTopic == "home/kitchen/vhomeMqtt/relay1") {
    // relay1
    relay1_switch = String((char*)payload);
    if(relay1_switch == "ON")
      {
        Serial.println("MQTT: Relay1 - ON");
        digitalWrite(relay1, LOW);
        digitalWrite(relay1LED, HIGH);
        /*
        # toggle if needed
        digitalWrite(relay1, HIGH);
        delay(1000);
        digitalWrite(relay1, LOW);
        */
      }
    if(relay1_switch == "OFF")
      {
        Serial.println("MQTT: Relay1 - OFF");
        digitalWrite(relay1, HIGH);
        digitalWrite(relay1LED, LOW);
      }
    } 
   
  else if(strTopic == "home/kitchen/vhomeMqtt/relay2"){
  // RELAY 2 
    relay2_switch = String((char*)payload);
    if(relay2_switch == "ON") {
        Serial.println("relay2 - ON");
        digitalWrite(relay2, LOW);
        digitalWrite(relay2LED, HIGH);
        /*
        # toggle if needed
        digitalWrite(relay1, HIGH);
        delay(1000);
        digitalWrite(relay1, LOW);
        */
    }
    if(relay2_switch == "OFF") {
        Serial.println("relay2 - OFF");
        digitalWrite(relay2, HIGH);
        digitalWrite(relay2LED, LOW);
    }
  }
}
 
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32_kitchen1", "create user in HA", "users password")) {
      Serial.println("connected");
      // Once connected, subscribe to topics
      client.subscribe("home/kitchen/vhomeMqtt/relay1");
      client.subscribe("home/kitchen/vhomeMqtt/relay2");
      client.subscribe("home/kitchen/vhomeMqtt/temp1");


    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // serial connection:
  Serial.begin(115200);
  // connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  //set up mqtt
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // configure pins
  pinMode(relay1, OUTPUT);
  pinMode(relay1LED, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay2LED, OUTPUT);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay1LED, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay2LED, HIGH);
  
  // button 1 
  pinMode(RELAY_BUTTON_1, INPUT_PULLUP);
  button1_state = digitalRead(RELAY_BUTTON_1);
  
  // button 2
  pinMode(RELAY_BUTTON_2, INPUT_PULLUP);
  button2_state = digitalRead(RELAY_BUTTON_2);
  
  // AHT sensor setup
  if (! aht.begin()) 
  {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
  aht.getTemperatureSensor();
}

void button1check() {
  /*
  BUTTON 1 functionality with publish request
  */
  last_button1_state = button1_state;      // save the last state
  button1_state = digitalRead(RELAY_BUTTON_1); // read new state

  if (last_button1_state == HIGH && button1_state == LOW && button1Count == 0) {
    Serial.println("Published: Relay1 ON");
    client.publish("home/kitchen/vhomeMqtt/relay1", "ON");
    button1Count = 1;
    delay(50);
  } else if(last_button1_state == HIGH && button1_state == LOW && button1Count == 1) {
    Serial.println("Published: Relay1 OFF");
    client.publish("home/kitchen/vhomeMqtt/relay1", "OFF");
    button1Count = 0;
    delay(50);
  }
}

void button2check() {
  /*
  BUTTON 2 functionality with publish request
  */
  last_button2_state = button2_state;      // save the last state
  button2_state = digitalRead(RELAY_BUTTON_2); // read new state

  if (last_button2_state == HIGH && button2_state == LOW && button2Count == 0) {
    Serial.println("Published: Relay2 ON");
    client.publish("home/kitchen/vhomeMqtt/relay2", "ON");
    button2Count = 1;
    delay(50);
  } else if(last_button2_state == HIGH && button2_state == LOW && button2Count == 1) {
    Serial.println("Published: Relay2 OFF");
    client.publish("home/kitchen/vhomeMqtt/relay2", "OFF");
    button2Count = 0;
    delay(50);
  }
}

void ahtSensorCheck() {
// AHT sensor 
  sensors_event_t humidity, temp;
  boolean rc;
  char msg_out[20];
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  // temp sensor - check and send mqtt publish
  if(temp.temperature > 3 && temp.temperature < 50) {
    Serial.println(temp.temperature);
    // convert float to string -- publish wont work otherwise
    float tempString = temp.temperature - 18;
    dtostrf(tempString,2,2,msg_out);
    rc = client.publish("home/kitchen/vhomeMqtt/temp1", msg_out);
  }
  // humidity sensor - check and send mqtt publish
  if(humidity.relative_humidity > 2) {
    Serial.println(humidity.relative_humidity);
    float humString = humidity.relative_humidity + 10;
    dtostrf(humString,2,2,msg_out);
    rc = client.publish("home/kitchen/vhomeMqtt/hum1", msg_out);
  }
 }

// START PROGRAM LOOP ///
void loop() {
  if (!client.connected()) {
    reconnect();
  } 
  // physical
  button1check();
  button2check();
  ahtSensorCheck();
  // MQTT 
  client.loop();
  // give some time for sensor readout .2sec
  delay(200);
}
