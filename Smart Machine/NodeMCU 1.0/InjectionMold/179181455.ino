//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Project : Smart Machine //
// Name    : Mr.Grisada Chooprasert //
// Version : 6.00 Build 20161009 //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Arduino.h"
// #include <Arduino.h>
#include <AuthClient.h>
#include <ESP8266WiFi.h>
#include <MQTTClient.h>
#include <MicroGear.h>
#include <PubSubClient.h>
#include <SHA1.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <debug.h>

SoftwareSerial Serial4(13, 15); // RX, TX D7 D8

const char *ssid = "NsetService";
const char *password = "nset1234";

/*
  --------------------------------------------------------------------------------------------------------------------------------------------------
  NO.| SHOP_NAME    | MACHINE_NAME         | KEY             | SECRET
  | ALIAS    | MAC ADDRESS       | TOPIC
  --------------------------------------------------------------------------------------------------------------------------------------------------
  51 | Stator Mold  | Injection Mold No.16 | h47PFuRbg4uQscL |
  Xjf12T7poF2izRzjEcDPPt3Y1 | SM_IM16 | 5C:CF:7F:18:99:A4 | SMx_IM16/Monitor0
  /Monitor1 /Monitor2
  52 | Stator Mold  | Injection Mold No.17 | kN74LdJmvpvfLcT |
  0DmIqNNY2VIzLafkZ4SPIy4td | SM_IM17 | 5C:CF:7F:85:E9:C4 | SMx_IM17/Monitor0
  /Monitor1 /Monitor2
  53 | Stator Mold  | Injection Mold No.18 | g1KYx8uZCdRySGQ |
  JKgDtSqlOodi9TQPKh1SBmYJT | SM_IM18 | 5C:CF:7F:19:06:33 | SMx_IM18/Monitor0
  /Monitor1 /Monitor2
  54 | Stator Mold  | Injection Mold No.19 | qGwgch5JKNF4Nup |
  t7T5DXGx1CleHbaZUua64T5z1 | SM_IM19 | 5C:CF:7F:17:9F:1A | SMx_IM19/Monitor0
  /Monitor1 /Monitor2
  55 | Stator Mold  | Injection Mold No.20 | 3QY57LbRiOT86j0 |
  JxgaEIF3YOrjej2nodncY6YFB | SM_IM20 | 5C:CF:7F:87:1B:BD | SMx_IM20/Monitor0
  /Monitor1 /Monitor2
*/

#define APPID "SmartMachine"
#define KEY "qGwgch5JKNF4Nup"
#define SECRET "t7T5DXGx1CleHbaZUua64T5z1"
#define ALIAS "IM020"

WiFiClient client;
AuthClient *authclient;

IPAddress local_ip = {192, 168, 2, 100};
IPAddress gateway = {192, 168, 2, 1};
IPAddress subnet = {255, 255, 255, 0};

unsigned long currentTime;
unsigned long previousTime = 0;
const long CycleTime = 3000;

int timer = 0;

MicroGear microgear(client);

void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //  ;
  //}
  Serial4.begin(9600);
  Serial.println("Starting");

  microgear.on(MESSAGE,
               onMsghandler); // Call onMsghandler() when new message arraives
  // microgear.on(PRESENT, onFoundgear);   // Call onFoundgear()  when new gear
  // appear
  // microgear.on(ABSENT, onLostgear);     // Call onLostgear()   when some gear
  // goes offline
  microgear.on(
      CONNECTED,
      onConnected); // Call onConnected()  when NETPIE connection is established

  // microgear.resetToken();             // Reset token
  microgear.init(KEY, SECRET,
                 ALIAS); // Initial with KEY, SECRET and also set the ALIAS here
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (microgear.connected()) {
      microgear.loop();
      Serial_Loop();
    } else {
      Microgear_Connect();
    }
  } else {
    WIFI_Connect();
  }
}

void WIFI_Connect() {
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.hostname(ALIAS);
  //  WiFi.config(local_ip, gateway, subnet);
  WiFi.begin(ssid, password);
  for (int i = 0; i < 60; i++) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(1000);
    }
  }
  Serial.print("\n");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected");
    Serial.print("SSID Name         : ");
    Serial.println(WiFi.SSID());
    Serial.print("SSID MAC Address  : ");
    Serial.println(WiFi.BSSIDstr());
    Serial.print("Device Name       : ");
    Serial.println(WiFi.hostname());
    Serial.print("Local MAC Address : ");
    Serial.println(WiFi.macAddress());
    Serial.print("Local IP Address  : ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway           : ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Subset            : ");
    Serial.println(WiFi.subnetMask());
    Serial.print("Connection Mode   : ");
    Serial.println("Station");
    Serial.print("Connection Status : ");
    Serial.println(WiFi.waitForConnectResult());
    // Serial.print("AP IP Address     : ");
    // Serial.println(WiFi.softAPIP());
    Serial.print("Application Name  : ");
    Serial.println(APPID);
    Serial.print("ALIAS             : ");
    Serial.println(ALIAS);
    Serial.print("Key               : ");
    Serial.println(KEY);
    Serial.print("Secret            : ");
    Serial.println(SECRET);
  }
}

void Microgear_Connect() {
  microgear.connect(APPID);
  delay(1000);
}

void onMsghandler(char *topic, uint8_t *msg,
                  unsigned int msglen) { // New message arrives, do this
  Serial.print("Incoming message --> ");
  msg[msglen] = '\0';
  Serial.println((char *)msg);
}

void onFoundgear(
    char *attribute, uint8_t *msg,
    unsigned int msglen) { // Found device connected with NETPIE, do this
  Serial.print("Found new member --> ");
  for (int i = 0; i < msglen; i++)
    Serial.print((char)msg[i]);
  Serial.println();
}

void onLostgear(
    char *attribute, uint8_t *msg,
    unsigned int msglen) { // Found device disconnect with NETPIE, do this
  Serial.print("Lost member --> ");
  for (int i = 0; i < msglen; i++)
    Serial.print((char)msg[i]);
  Serial.println();
}

void onConnected(
    char *attribute, uint8_t *msg,
    unsigned int msglen) {   // When a microgear is connected, do this
  microgear.setAlias(ALIAS); // Set the alias of this microgear ALIAS
  // microgear.setName(ALIAS);
}

void Serial_Loop() {
  while (Serial4.available() > 0) {
    String line = Serial4.readStringUntil('\r');
    if (line.length() > 3) {

      microgear.publish("/IM000", line, true);
      Serial.println(line);
      Serial4.print(">> Send OK");
      Serial4.print('\r');
    }
  }
}

/* Remark
  const char* ssid     = "NSET";
  const char* password = "ea40f5c8de350717aebfd3f1a8";

  ets Jan  8 2013,rst cause:2, boot mode:(3,6)

  if (line.startsWith("IM016")) {
        microgear.publish("/IM016", line, true);
        Serial.println(line);
        Serial4.print(">> Send IM016 OK");
        Serial4.print('\r');
      }
*/
