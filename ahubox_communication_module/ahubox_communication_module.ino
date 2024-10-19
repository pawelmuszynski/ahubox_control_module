#include <Timer.h>
#include <Wire.h>
#include <ENC28J60lwIP.h>
#include <CRC8.h>
#include <AsyncMqtt_Generic.h>

#define SLAVE_ADDR 8
#define CSPIN 16
//#define ETH_HOSTNAME "pompa_ciepla"

#define MQTT_CHECK_INTERVAL_MS        2000
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "pompa_ciepla_dev"
//#define MQTT_WILL_TOPIC "/topic/test"     // You can change
//#define MQTT_WILL_MSG   "I am leaving..." // You can change
#define MQTT_SERVER     "10.0.2.10"
#define PAYLOAD_SIZE    50

#define WIRE_DATA_FRAME 0xAA // It identifies data frame (crc is not enough). It should be equal on both sides.

Timer timer;
ENC28J60lwIP eth(CSPIN);

const char *PubTopic = "test/pompa_ciepla_dev";
const char *SettingsTopicIn = "heat_pump_dev/settings/in";
const char *SettingsTopicOut = "heat_pump_dev/settings/out";

AsyncMqttClient mqttClient;
bool connectedMQTT = false;

struct EnergyData {
  uint16_t voltage, current, power, energy, pf;
};

struct PidParams {
  float kp, ki, kd;
  uint8_t omin, omax;
};

struct PidData {
  uint16_t sv;
  uint8_t output;
};

struct TempData {
  uint16_t heat, ret;
  int16_t outside;
};

struct HC {
  uint16_t hc_minus5, hc_0, hc_5, hc_10;
};

TempData temp_data;
EnergyData energy_data;
PidData pid_data;

byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

void setup() {
  delay(1000);
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\n\n================== START ====================");
  Serial.print(ESP.getFullVersion());
  Serial.println();
  Serial.print(ESP.getCoreVersion());
  Serial.println();

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(1000000);

  eth.setDefault();
  //eth.setHostname(ETH_HOSTNAME);
  if (!eth.begin(mac)) Serial.println("ERROR: No ENC28J60 Ethernet hardware module.");
  //else last_conn_status = true;

  //timer.every(10000, on10sec);
  timer.every(5000, checkEthernetConnection);
  timer.every(60000, on60sec);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  timer.every(MQTT_CHECK_INTERVAL_MS, connectToMqttCheck);
}

uint8_t wire_val[2];

void loop() {
  timer.update();

  // ======== UART commands support ==========
/*  char received = 0x00;
  static char serial_buf[50];
  static uint8_t buf_index = 0;
  if (Serial.available()) {
    received = Serial.read();
    if (received == '\n' || received == '\r') {  // command fully received
      processCommand(serial_buf);
      serial_buf[0] = '\0';  // clear buffer
      buf_index = 0;
    } else {  // command not fully received, attaching the last character
      serial_buf[buf_index] = received;
      buf_index++;
      serial_buf[buf_index] = '\0';
    }
  }
*/
}

//void processCommand(char *buf) {
//  Serial.println(buf);
//}

/*
void processCommand(char *buf) {
  Serial.println("Received command:");
  Serial.println(buf);
  if (!strncmp(buf, "get ", 4)) {  // get command
    processGet(buf + 4);
  } else if (!strncmp(buf, "set ", 4)) {  // set command
    //processSet(buf + 4);
  } else {
    Serial.println("Command not found. Use help command.");
  }
  Serial.println("==================");
}
*/
/*
void processGet(char *buf2) {
  PidParams pp;

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.requestFrom(SLAVE_ADDR, sizeof(pp));
  Serial.println("Receiving pid params");
  Wire.readBytes((byte*) &pp, sizeof(pp));

  char buf[50];

   //Serial.println(sizeof(pp.kp));

  //snprintf(buf, sizeof(buf), "%u %u %.2f %.2f %.2f", pp.omin, pp.omax, pp.kp, pp.ki, pp.kd);
  //Serial.println(buf);
  Serial.println(pp.kp);
}
*/
/*
void on10sec() {
  Serial.println("Wire ");
  Wire.beginTransmission(SLAVE_ADDR);
  //Wire.write(2); // send set command
  //Wire.write("kd 5");
  Wire.write(3);

  Wire.endTransmission();
  char buf[10];
  if(Wire.requestFrom(SLAVE_ADDR, sizeof(buf))) {
    Serial.println("jest");
    Wire.readBytes(buf, sizeof(buf));
    Serial.println(buf);
  }

}
*/

void on60sec() {
  Serial.println("---------------------");
  Serial.println("Wire get values");
  Serial.println("---------------------");

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0x01); // get values command
  Wire.endTransmission();

  TempData td;
  EnergyData ed;
  PidData pd;
  uint8_t wire_crc;
  uint8_t frame_type;

  Wire.requestFrom(SLAVE_ADDR, sizeof(frame_type) + sizeof(td) + sizeof(ed) + sizeof(pd) + sizeof(wire_crc)); // request all data

  if(!Wire.readBytes(&frame_type, sizeof(frame_type))) {
    Serial.println("Error: Empty Domoticz Data frame received");
    return;
  }
  if(frame_type != WIRE_DATA_FRAME) {
    Serial.println("Error: Wrong frame header byte received in Domoticz Data frame");
    return;
  }

  Wire.readBytes((byte*) &td, sizeof(td));
  Wire.readBytes((byte*) &ed, sizeof(ed));
  Wire.readBytes((byte*) &pd, sizeof(pd));
  Wire.readBytes(&wire_crc, sizeof(wire_crc));

  CRC8 crc;
  crc.add(&frame_type, sizeof(frame_type));
  crc.add((uint8_t *)&td, sizeof(td));
  crc.add((uint8_t *)&ed, sizeof(ed));
  crc.add((uint8_t *)&pd, sizeof(pd));

  if(crc.calc() != wire_crc) {
    Serial.println("Error: Wrong CRC received in Domoticz Data frame");
    return;
  }

  showValues(td, ed, pd);

  char buf[50];
  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%01u.%02u\"}", 117, ed.voltage / 10, ed.voltage % 10);
  mqttClient.publish("domoticz/in", 0, false, buf);
  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%01u.%02u\"}", 118, ed.current / 1000, ed.current % 1000);
  mqttClient.publish("domoticz/in", 0, false, buf);
  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%01u.%02u\"}", 116, ed.power / 10, ed.power % 10);
  mqttClient.publish("domoticz/in", 0, false, buf);
  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%01u.%02u\"}", 119, ed.pf / 100, ed.pf % 100);
  mqttClient.publish("domoticz/in", 0, false, buf);

  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%01u.%02u\"}", 4, td.heat / 100, td.heat % 100);
  mqttClient.publish("domoticz/in", 0, false, buf);
  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%01u.%02u\"}", 6, td.ret / 100, td.ret % 100);
  mqttClient.publish("domoticz/in", 0, false, buf);
  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%01d.%02d\"}", 7, td.outside / 100, td.outside % 100);
  mqttClient.publish("domoticz/in", 0, false, buf);

  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%01u.%02u\"}", 114, pd.sv / 100, pd.sv % 100);
  mqttClient.publish("domoticz/in", 0, false, buf);
  snprintf(buf, sizeof(buf), "{\"idx\": %u, \"svalue\": \"%u\"}", 11, map(pd.output, 0, 255, 0, 100));
  mqttClient.publish("domoticz/in", 0, false, buf);
}

void printIPInfo() {
  Serial.print("ethernet ip address: ");
  Serial.println(eth.localIP());
  Serial.print("ethernet subnetMask: ");
  Serial.println(eth.subnetMask());
  Serial.print("ethernet gateway: ");
  Serial.println(eth.gatewayIP());
}

void checkEthernetConnection() {
    static bool connectedETH_last_state;
    if(eth.connected()) {
      //Serial.println("eth connected");
      if(!connectedETH_last_state) {
        Serial.println("eth now connected");
        printIPInfo();
        connectToMqtt();
      }
      connectedETH_last_state = true;
    }
    else {
      //Serial.println("eth not connected");
      if(connectedETH_last_state) {
        Serial.println("eth now disconnected");
      }
      connectedETH_last_state = false;
    }
}

void showValues(TempData td, EnergyData ed, PidData pd) {
  char buf[30];
  snprintf(buf, sizeof(buf), "Voltage: %01u.%01u V", ed.voltage / 10, ed.voltage % 10);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "Current: %01u.%03u A", ed.current / 1000, ed.current % 1000);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "Power: %01u.%01u W", ed.power / 10, ed.power % 10);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "PF: %01u.%02u", ed.pf / 100, ed.pf % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "heat_temp: %01d.%02d °C", td.heat / 100, td.heat % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "return_temp: %01d.%02d °C", td.ret / 100, td.ret % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "outside_temp: %01d.%02d °C", td.outside / 100, td.outside % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "pid_sv: %01d.%02d °C", pd.sv / 100, pd.sv % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "set power: %d (%d%%)", pd.output, map(pd.output, 0, 255, 0, 100));
  Serial.println(buf);
}

void connectToMqttCheck() {
  if (eth.connected()) {
    AMQTT_LOGDEBUG("C");
    if (!connectedMQTT) {
      mqttClient.connect();
    }
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.print("Connected to MQTT broker: ");
  Serial.print(MQTT_SERVER);
  Serial.print(", port: ");
  Serial.println(MQTT_PORT);
  Serial.print("PubTopic: ");
  Serial.println(PubTopic);

  connectedMQTT = true;

  Serial.println("------------------------------");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  mqttClient.publish(SettingsTopicIn, 0, false, "Put Heat Pump command here.");
  mqttClient.publish(SettingsTopicOut, 0, false, "Read Heat Pump command output from here.");

  uint16_t packetIdSub = mqttClient.subscribe(SettingsTopicIn, 0);
  Serial.print("Subscribing: ");
  Serial.println(SettingsTopicIn);

  mqttClient.publish(PubTopic, 0, true, "ESP8266_Ethernet Test1");
  Serial.println("Publishing at QoS 0");

  uint16_t packetIdPub1 = mqttClient.publish(PubTopic, 1, true, "ESP8266_Ethernet Test2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);

  uint16_t packetIdPub2 = mqttClient.publish(PubTopic, 2, true, "ESP8266_Ethernet Test3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);

  Serial.println("------------------------------");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  (void) reason;
  connectedMQTT = false;
  Serial.println("Disconnected from MQTT.");
  if (eth.connected()) {
    timer.after(2000, connectToMqtt);
  }
}

void onMqttSubscribe(const uint16_t& packetId, const uint8_t& qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(const uint16_t& packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, const AsyncMqttClientMessageProperties& properties,
                   const size_t& len, const size_t& index, const size_t& total) {
  char message[len + 1];

  memcpy(message, payload, len);
  message[len] = 0;

  if(!strcmp(topic, SettingsTopicIn)) {
    char cmd[5];
    uint8_t cmd_len = min(strcspn(message, " "), sizeof(cmd)-1);
    Serial.println(cmd_len);
    memcpy(cmd, message, cmd_len);
    cmd[cmd_len] = 0x00; // terminate string

    Serial.println(cmd);

    if(!strcmp(cmd, "set")) {
      processSet(message+cmd_len+1);
    }
    else if(!strcmp(cmd, "get")) {
      processGet(message+cmd_len+1);
    }
    else if(!strcmp(cmd, "save")) {
      processSave();
    }
    else {
      Serial.println("NIEZNANA KOMENDA");
      mqttClient.publish(SettingsTopicOut, 0, false, "NIEZNANA KOMENDA");
    }
  }
/*
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  message: ");
  Serial.println(message);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
*/
}

void onMqttPublish(const uint16_t& packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void processGet(char *param) {
  char buf[70];
  if(!strcmp(param, "pid")) {
    Serial.println("Calling for pid");
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x02); // get pid
    Wire.endTransmission();
    PidParams pid_params;
    Wire.requestFrom(SLAVE_ADDR, sizeof(pid_params));
    Wire.readBytes((byte*) &pid_params, sizeof(pid_params));
    snprintf(buf, sizeof(buf), "%.2f %.2f %.2f %hhu %hhu", pid_params.kp, pid_params.ki, pid_params.kd, pid_params.omin, pid_params.omax);
  }
  else if(!strcmp(param, "hc")) {
    Serial.println("Calling for hc");
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x03); // get hc
    Wire.endTransmission();
    HC hc;
    Wire.requestFrom(SLAVE_ADDR, sizeof(hc));
    Wire.readBytes((byte*) &hc, sizeof(hc));
    snprintf(buf, sizeof(buf), "%01u.%02u %01u.%02u %01u.%02u %01u.%02u", hc.hc_minus5 / 100, hc.hc_minus5 % 100, hc.hc_0 / 100, hc.hc_0 % 100,
              hc.hc_5 / 100, hc.hc_5 % 100, hc.hc_10 / 100, hc.hc_10 % 100);
  }
  else if(!strcmp(param, "temp")) {
    Serial.println("Calling for temp");
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x04); // get temp
    Wire.endTransmission();
    TempData td;
    Wire.requestFrom(SLAVE_ADDR, sizeof(td));
    Wire.readBytes((byte*) &td, sizeof(td));
    snprintf(buf, sizeof(buf), "%01u.%02u %01u.%02u %01d.%02d", td.heat / 100, td.heat % 100, td.ret / 100, td.ret % 100, td.outside / 100, td.outside % 100);
  }
  else if(!strcmp(param, "energy")) {
    Serial.println("Calling for energy");
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x05); // get energy
    Wire.endTransmission();
    EnergyData ed;
    Wire.requestFrom(SLAVE_ADDR, sizeof(ed));
    Wire.readBytes((byte*) &ed, sizeof(ed));
    snprintf(buf, sizeof(buf), "Voltage: %01u.%01u V\nCurrent: %01u.%03u A\nPower: %01u.%01u W\nPF: %01u.%02u",
             ed.voltage / 10, ed.voltage % 10, ed.current / 1000, ed.current % 1000, ed.power / 10, ed.power % 10, ed.pf / 100, ed.pf % 100);
  }
  else {
    strcpy(buf, "unknown parameter ");
    snprintf(buf, sizeof(buf), "unknown parameter %s", param);
  }
  mqttClient.publish(SettingsTopicOut, 0, false, buf);
  Serial.println(buf);
}

void processSet(char *param) {
  char buf[50];
  Serial.println("SET COMMAND");
  Serial.println(param);
  char *values;
  uint8_t args_assigned;
  if(!strncmp(param, "pid", 3)) {
    values = param + 4;
    Serial.println("Trying set pid");
    PidParams pid_params;
    Serial.println("Before sscanf");
    args_assigned = sscanf(values, "%f %f %f %hhu %hhu", &pid_params.kp, &pid_params.ki, &pid_params.kd, &pid_params.omin, &pid_params.omax);
    Serial.println("After sscanf");
    if(args_assigned == 5) {
      Serial.println("Setting pid");
      strcpy(buf, "Setting pid");

      Serial.println(pid_params.kp);
      Serial.println(pid_params.ki);
      Serial.println(pid_params.kd);
      Serial.println(pid_params.omin);
      Serial.println(pid_params.omax);
      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write(0x12); // set pid cmd
      Wire.write((byte *) &pid_params, sizeof(pid_params));
      Wire.endTransmission();
    }
    else {
      strcpy(buf, "Wrong values for pid");
    }
  }
  else if(!strncmp(param, "hc", 2)) {
    values = param + 3;
    Serial.println("Trying set hc");
    HC hc;
    Serial.println("Before sscanf");
    args_assigned = sscanf(values, "%hu %hu %hu %hu", &hc.hc_minus5, &hc.hc_0, &hc.hc_5, &hc.hc_10);
    Serial.println("After sscanf");
    if(args_assigned == 4) {
      Serial.println("Setting hc");
      strcpy(buf, "Setting hc");

      Serial.println(hc.hc_minus5);
      Serial.println(hc.hc_0);
      Serial.println(hc.hc_5);
      Serial.println(hc.hc_10);
      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write(0x13); // set hc cmd
      Wire.write((byte *) &hc, sizeof(hc));
      Wire.endTransmission();
    }
    else {
      strcpy(buf, "Wrong values for hc");
    }
  }
  else {
    strcpy(buf, "unknown parameter ");
    snprintf(buf, sizeof(buf), "unknown parameter %s", param);
  }
  mqttClient.publish(SettingsTopicOut, 0, false, buf);
}

void processSave() {
  Serial.println("Saving to EEPROM");
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0x20); // save cmd
  Wire.endTransmission();
}
