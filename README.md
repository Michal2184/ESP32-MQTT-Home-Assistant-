# ESP32-MQTT-Home-Assistant-Integration

Lightweight code to connect ESP32 to Home Assistant via Mosquitto MQTT Broker. Remember to create topics using home assistant configuration.yaml file. Edit code to update details about your wifi, mqtt broker login and topics.

Functionality:
- wifi connection to local LAN only (no gateway needed)
- log in with authentication to MQTT Broker
- Subscribes to two mqtt topics to control two relays
- LED reacts to mqtt topic update 
- Buttons press updates mqtt topic to publish 
- AHT20 temp and humidity sensor pubish to seperate mqtt topics 
