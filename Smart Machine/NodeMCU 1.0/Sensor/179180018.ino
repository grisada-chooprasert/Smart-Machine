#include "Arduino.h"
#include <Grisada.h>

Grisada PE;
WiFiClient client;
AuthClient *authclient;
MicroGear microgear(client);

String currentStatus;

void setup() {
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  digitalWrite(MAX6675_CS, HIGH);

  PE.connectWIFI();

  pinMode(FLOW_RATE_PIN, INPUT);
  attachInterrupt(FLOW_RATE_PIN, YFS201_RPM, RISING);

  microgear.on(MESSAGE, onMsghandler);
  microgear.on(CONNECTED, onConnected);
  // microgear.resetToken();
  microgear.init(PE.NETPIE_KEY, PE.NETPIE_SECRET, PE.NETPIE_ALIAS);
}

void loop() {
  PE.currentTime = millis();
  if (PE.currentTime - PE.previousTime >= PE.CycleTime) {
    PE.previousTime = PE.currentTime;

    Serial.print("\n");
    Serial.print(WiFi.status());
    Serial.print("  ");
    delay(1);

    Serial.print(PE.currentTime / 1000);
    Serial.print("  ");
    delay(1);

    int currentPowerTime = PE.currentTime / 1000 / 60;
    Serial.print(currentPowerTime);
    Serial.print("Min");
    Serial.print("  ");
    delay(1);

    int currentSignal = WiFi.RSSI();
    Serial.print(currentSignal);
    Serial.print("dBm");
    Serial.print("  ");
    delay(1);

    float currentTemperature = PE.getTemperature();
    Serial.print(currentTemperature);
    Serial.print("C");
    Serial.print("  ");
    delay(1);

    float currentFlowRate = PE.getFlowRate();
    Serial.print(currentFlowRate);
    Serial.print("l/m");
    Serial.print("  ");
    delay(1);

    /*if (currentTemperature < 55 && currentFlowRate > 3) {
        currentStatus = "Normal";
    }
    if ((currentTemperature >= 55 && currentTemperature < 60) ||
    (currentFlowRate > 1 && currentFlowRate <= 3)) {
        currentStatus = "Warning";
    }
    if (currentTemperature >= 60 || currentFlowRate <= 1) {
        currentStatus = "Abnormal";
    }*/

    if (currentTemperature < 55) {
      currentStatus = "Normal";
    }
    if (currentTemperature >= 55 && currentTemperature < 60) {
      currentStatus = "Warning";
    }
    if (currentTemperature >= 60) {
      currentStatus = "Abnormal";
    }

    /*if (currentFlowRate > 3) {
        currentStatus = "Normal";
    }
    if (currentFlowRate > 1 && currentFlowRate <= 3) {
        currentStatus = "Warning";
    }
    if (currentFlowRate <= 1) {
        currentStatus = "Abnormal";
    }*/

    if (WiFi.status() == WL_CONNECTED) {
      if (microgear.connected()) {
        microgear.loop();
        delay(1);

        static int previousPowerTime, previousSignal;
        static float previousTemperature, previousFlowRate;

        if (currentPowerTime != previousPowerTime ||
            // currentSignal      != previousSignal ||
            currentTemperature != previousTemperature ||
            currentFlowRate != previousFlowRate) {

          previousPowerTime = currentPowerTime;
          previousSignal = currentSignal;
          previousTemperature = currentTemperature;
          previousFlowRate = currentFlowRate;

          String FreeBoard_Data =
              (String)currentPowerTime + "|" + (String)currentSignal + "|" +
              (String)currentTemperature + "|" + (String)currentFlowRate + "|" +
              (String)currentStatus + "|" + "V17.8.30";

          char
              topic[sizeof("/") + sizeof(PE.NETPIE_ALIAS) + sizeof("/Monitor")];
          sprintf(topic, "%s%s%s", "/", PE.NETPIE_ALIAS, "/Monitor");
          microgear.publish(topic, FreeBoard_Data, true);
          // microgear.chat(topic, FreeBoard_Data);
          delay(1);

          String Feed_Data = "{\"PowerTime\":" + (String)currentPowerTime +
                             ",\"Signal\":" + (String)currentSignal +
                             ",\"Temperature\":" + (String)currentTemperature +
                             ",\"FlowRate\":" + (String)currentFlowRate + "}";

          char FeedID[sizeof(PE.NETPIE_APPCODE) +
                      sizeof(PE.NETPIE_ALIAS)]; // char
          // FeedID[sizeof(PE.NETPIE_ALIAS)
          // + sizeof("Feed")];
          sprintf(FeedID, "%s%s", PE.NETPIE_APPCODE,
                  PE.NETPIE_ALIAS); // sprintf(FeedID, "%s%s", PE.NETPIE_ALIAS,
          // "Feed");
          microgear.writeFeed(FeedID, Feed_Data);
          delay(1);

          Serial.print("Send");
        }
      } else {
        connectMicrogear();
      }
    } else {
      PE.connectWIFI();
    }
    PE.maintenanceWIFI();
  }
  // PE.maintenanceWIFI();
}

void connectMicrogear() {
  microgear.connect(PE.NETPIE_APPID);
  delay(1000);
}

void onConnected(char *attribute, uint8_t *msg, unsigned int msglen) {
  Serial.print("\n\n");
  Serial.println("Connected to MQTT");
  Serial.print("Application Name  : ");
  Serial.println(PE.NETPIE_APPID);
  Serial.print("ALIAS             : ");
  Serial.println(PE.NETPIE_ALIAS);
  Serial.print("Key               : ");
  Serial.println(PE.NETPIE_KEY);
  Serial.print("SECRET            : ");
  Serial.println(PE.NETPIE_SECRET);
  microgear.setAlias(PE.NETPIE_ALIAS);
  microgear.setName(PE.NETPIE_ALIAS);
}

void onMsghandler(char *topic, uint8_t *msg, unsigned int msglen) {
  Serial.print("\n\n");
  Serial.print("<-- ");
  Serial.print(topic);
  Serial.print(" : ");
  char strState[msglen];
  for (unsigned int i = 0; i < msglen; i++) {
    strState[i] = (char)msg[i];
    Serial.print((char)msg[i]);
  }
  Serial.println();

  String stateStr = String(strState).substring(0, msglen);
  // Serial.println(stateStr);

  char topic1[sizeof(PE.NETPIE_ALIAS) + sizeof("/InterLock")];
  sprintf(topic1, "%s%s", PE.NETPIE_ALIAS, "/InterLock");

  if (stateStr == "InterLock_On") {
    microgear.chat(topic1, "ON");
  } else if (stateStr == "InterLock_Off") {
    microgear.chat(topic1, "OFF");
  }
}

void YFS201_RPM() { PE.NbTopsFan++; }
