#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "krishna";
const char* password = "krishna7";

const char* mqtt_server = "thingsboard.cloud";
const int mqtt_port = 1883;
const char* access_token = "5s00u25xfieh77mkh9q5";

WiFiClient espClient;
PubSubClient client(espClient);

void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Connecting to ThingsBoard...");
    if (client.connect("STM32_DATA", access_token, NULL)) {
      Serial.println("Connected to ThingsBoard!");
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);   
  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

if (Serial.available()) {
  String data = Serial.readStringUntil('\n');
  data.trim();

  if (data.startsWith("CAN,")) {
    data.remove(0, 4);

    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);

    int distance = data.substring(0, c1).toInt();
    int temperature = data.substring(c1 + 1, c2).toInt();
    int gas = data.substring(c2 + 1).toInt();

    String payload = "{";
    payload += "\"distance\":" + String(distance) + ",";
    payload += "\"temperature\":" + String(temperature) + ",";
    payload += "\"gas\":" + String(gas);
    payload += "}";

    client.publish("v1/devices/me/telemetry", payload.c_str());
  }
  else if (data.startsWith("GPS,")) {
    data.remove(0, 4);

    String gpsPayload = "{";
    gpsPayload += "\"gps_raw\":\"" + data + "\"";
    gpsPayload += "}";

    client.publish("v1/devices/me/telemetry", gpsPayload.c_str());
  }
}



}
