/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "arduino_secrets.h"
#include <ArduinoJson.h>


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const char broker[] = "mqtt.cetools.org";
int        port     = 1884;
const char topic[]  = "UCL/90TCR/SCD-41";

String clientID;

//set interval for sending messages (milliseconds)
const long interval = 30000;
unsigned long previousMillis = 0;



void setup() {
    //Set up serial debug
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    // attempt to connect to Wifi network:
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(SECRET_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(SECRET_SSID, SECRET_PASS);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("");
   
    mqttClient.setServer(broker, port);
    mqttClient.setCallback(callback);
    clientID = WiFi.macAddress();
 
}


void loop() {
  
    if (!mqttClient.connected()) {
        reconnect();
    }
   
    mqttClient.loop(); 
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();


  StaticJsonDocument<100> doc;
  deserializeJson(doc, payload);
  int co2 = doc["co2"];
  int temperature = doc["temperature"];
  int humidity = doc["humidity"];

  Serial.print("co2=");
  Serial.println(co2);

  Serial.print("temperature=");
  Serial.println(temperature);

  Serial.print("humidity=");
  Serial.println(humidity);
  Serial.println("");
}


void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (  mqttClient.connect(clientID.c_str(), MQTT_NAME, MQTT_PASS)  ) {
      Serial.println("connected");
      mqttClient.subscribe(topic);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
