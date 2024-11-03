#include <Wire.h>
#include <ENC28J60lwIP.h>
#include <CRC8.h>
#include <AsyncMqtt_Generic.h> // by Mavin Roger, Khoi Hoang
#include <EEPROM.h>

#define CONN_CHECK_INTERVAL_S 5

#define WIRE_DATA_FRAME 0xAA  // It identifies data frame (crc is not enough). It should be equal on both sides.
#define SLAVE_ADDR 8
#define CSPIN 16
#define DEFROST_SIGNAL_PIN 0

#define DEFAULT_SETTINGS_INDICATOR 0xAA  // If it's not set in EEPROM, default settings will be writen.
#define DEFAULT_MAC_ADDR 0x00, 0x01, 0xDE, 0xAD, 0xBE, 0xEF
#define DEFAULT_MQTT_SERVER "10.0.2.10"
#define DEFAULT_MQTT_PORT 1883
#define DEFAULT_MQTT_CLIENT_ID "heat_pump_dev"
#define MQTT_CONSOLE_IN "heat_pump_dev/console/in"
#define MQTT_CONSOLE_OUT "heat_pump_dev/console/out"
#define MQTT_DOMOTICZ_IN "domoticz/in"

#define DOMOTICZ_VOLTAGE_IDX 117
#define DOMOTICZ_CURRENT_IDX 118
#define DOMOTICZ_POWER_IDX 116
#define DOMOTICZ_PF_IDX 119
#define DOMOTICZ_HEAT_IDX 4
#define DOMOTICZ_RETURN_IDX 6
#define DOMOTICZ_OUTSIDE_IDX 7
#define DOMOTICZ_SV_IDX 114
#define DOMOTICZ_OUTPUT_IDX 11

#define DEFAULT_SETTINGS_INDICATOR_ADDR 0x00  // uint8_t
#define MAC_ADDR_ADDR 0x01  // to 0x06 (6x uint8_t)
#define MQTT_SERVER_ADDR 0x07  // to 0x46 (63x char + null)
#define MQTT_PORT_ADDR 0x47  // to 0x48 (uint16_t)
#define MQTT_CLIENT_ID_ADDR 0x49 // to 0x7A (31x char + null)

//uint8_t mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };
const char *console_in_topic = MQTT_CONSOLE_IN;
const char *console_out_topic = MQTT_CONSOLE_OUT;
const char *domoticz_in_topic = MQTT_DOMOTICZ_IN;

ENC28J60lwIP eth(CSPIN);
AsyncMqttClient mqttClient;
bool connectedMQTT = false;

bool defrost_last_state = false;

struct EnergyData {
  uint16_t voltage, current, power, energy, pf;
};

struct PidParams {
  float kp, ki, kd;
  uint8_t omin, omax;
};

struct PidData {
  int16_t sv;
  uint8_t output;
};

struct TempData {
  int16_t heat, ret, outside;
};

struct HC {
  uint16_t hc_minus5, hc_0, hc_5, hc_10;
};

uint8_t timer_start, timer_start_60;

void setup() {
  delay(5000);
  Serial.begin(115200);
  EEPROM.begin(0x7F);
  Serial.println(F("\n\n================== START ===================="));
  Serial.println(ESP.getFullVersion());
  Serial.println(ESP.getCoreVersion());

  pinMode(DEFROST_SIGNAL_PIN, INPUT_PULLUP);

  uint8_t mac[] = { DEFAULT_MAC_ADDR };
  char mqtt_server[64] = DEFAULT_MQTT_SERVER;
  uint16_t mqtt_port = DEFAULT_MQTT_PORT;
  char mqtt_client_id[32] = DEFAULT_MQTT_CLIENT_ID;

  uint8_t default_settings_indicator;
  EEPROM.get(DEFAULT_SETTINGS_INDICATOR_ADDR, default_settings_indicator);
  if(default_settings_indicator == (uint8_t)DEFAULT_SETTINGS_INDICATOR) {
    EEPROM.get(MAC_ADDR_ADDR, mac);
    EEPROM.get(MQTT_SERVER_ADDR, mqtt_server);
    EEPROM.get(MQTT_PORT_ADDR, mqtt_port);
    EEPROM.get(MQTT_CLIENT_ID_ADDR, mqtt_client_id);
  }
  else {
    EEPROM.put(DEFAULT_SETTINGS_INDICATOR_ADDR, (uint8_t)DEFAULT_SETTINGS_INDICATOR);
    EEPROM.put(MAC_ADDR_ADDR, mac);
    EEPROM.put(MQTT_SERVER_ADDR, mqtt_server);
    EEPROM.put(MQTT_PORT_ADDR, mqtt_port);
    EEPROM.put(MQTT_CLIENT_ID_ADDR, mqtt_client_id);
    Serial.println(F("Setting factory defaults"));
  }
  EEPROM.commit();
  Serial.print(F("Ethernet mac addr: "));
  for(uint8_t i=0; i<sizeof(mac); i++) {
    Serial.print(mac[i], HEX);
    if(i<sizeof(mac)-1) Serial.print(':');
  }
  Serial.println();

  Wire.begin();
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(1000000);

  eth.setDefault();
  if (!eth.begin(mac)) Serial.println(F("ERROR: No ENC28J60 Ethernet hardware module."));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  Serial.print("mqtt_server: ");
  Serial.println(mqtt_server);
  Serial.print("mqtt_port: ");
  Serial.println(mqtt_port);
  Serial.print("mqtt_client_id: ");
  Serial.println(mqtt_client_id);
  char *mqtt_server_dyn = (char*)malloc(strlen(mqtt_server)+1);
  strncpy(mqtt_server_dyn, mqtt_server, strlen(mqtt_server)+1);
  char *mqtt_client_id_dyn = (char*)malloc(strlen(mqtt_client_id)+1);
  strncpy(mqtt_client_id_dyn, mqtt_client_id, strlen(mqtt_client_id)+1);
  mqttClient.setClientId(mqtt_client_id_dyn);
  mqttClient.setServer(mqtt_server_dyn, mqtt_port);

  timer_start = millis()/1000;
  timer_start_60 = timer_start;

  Serial.println(F("-= STARTED =-"));
}

void loop() {
  if(!defrost_last_state && !digitalRead(DEFROST_SIGNAL_PIN)) {
    defrost_last_state = true;
    Serial.println("START DEFROST");
  }

  if(defrost_last_state && digitalRead(DEFROST_SIGNAL_PIN)) {
    defrost_last_state = false;
    Serial.println("END DEFROST");
  }

  // ======== UART commands support ==========
  char received = 0x00;
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

  if((uint8_t)((uint8_t)(millis()/1000) - timer_start) >= CONN_CHECK_INTERVAL_S) {
    timer_start = millis()/1000;
    checkEthernetConnection();
    connectToMqttCheck();
  }

  if((uint8_t)((uint8_t)(millis()/1000) - timer_start_60) >= 60) {
    timer_start_60 = millis()/1000;
    on60sec();
  }
}

//void processCommand(char *buf) {
//  Serial.println(buf);
//}

void processCommand(char *buf) {
  Serial.print(F("Received command: "));
  Serial.println(buf);
  if (!strncmp(buf, "get ", 4)) {  // get command
    //processGet(buf + 4);
  } else if (!strncmp(buf, "set ", 4)) {  // set command
    processSet(buf + 4);
  } else {
    Serial.println("Command not found. Use help command.");
  }
  Serial.println("==================");
}

void processSet(char *buf) {
  if(!strncmp(buf, "mqtt_server ", 12)) {
    Serial.print(F("Setting mqtt_server: "));
    Serial.println(buf + 12);
    writeStringToEEPROM(MQTT_SERVER_ADDR, buf + 12, strlen(buf + 12));
    Serial.println(F("Please reset the board."));
  } else if (!strncmp(buf, "mqtt_port ", 10)) {
    Serial.print(F("Setting mqtt_port: "));
    Serial.println(buf + 10);
    uint16_t port = atoi(buf + 10);
    EEPROM.put(MQTT_PORT_ADDR, port);
    Serial.println(F("Please reset the board."));
  } else if (!strncmp(buf, "mqtt_client_id ", 15)) {
    Serial.print(F("Setting mqtt_client_id: "));
    Serial.println(buf + 15);
    writeStringToEEPROM(MQTT_CLIENT_ID_ADDR, buf + 15, strlen(buf + 15));
    Serial.println(F("Please reset the board."));
  }
  EEPROM.commit();
}

void writeStringToEEPROM(uint16_t addr, const char* str, const uint8_t len) {
  for(uint8_t i=0; i<=len; i++) {
    EEPROM.write(addr + i, str[i]);
  }
}
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
  Serial.println(F("\n---------------------"));
  Serial.println(F("Get Domoticz Data"));
  Serial.println(F("---------------------"));

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0x01);  // get values command
  Wire.endTransmission();

  TempData td;
  EnergyData ed;
  PidData pd;
  uint8_t wire_crc;
  uint8_t frame_type;

  Wire.requestFrom(SLAVE_ADDR, sizeof(frame_type) + sizeof(td) + sizeof(ed) + sizeof(pd) + sizeof(wire_crc));  // request all data

  if (!Wire.readBytes(&frame_type, sizeof(frame_type))) {
    Serial.println(F("Error: Empty Domoticz Data frame received"));
    return;
  }
  if (frame_type != WIRE_DATA_FRAME) {
    Serial.println(F("Error: Wrong frame header byte received in Domoticz Data frame"));
    return;
  }

  Wire.readBytes((byte *)&td, sizeof(td));
  Wire.readBytes((byte *)&ed, sizeof(ed));
  Wire.readBytes((byte *)&pd, sizeof(pd));
  Wire.readBytes(&wire_crc, sizeof(wire_crc));

  CRC8 crc;
  crc.add(&frame_type, sizeof(frame_type));
  crc.add((uint8_t *)&td, sizeof(td));
  crc.add((uint8_t *)&ed, sizeof(ed));
  crc.add((uint8_t *)&pd, sizeof(pd));

  if (crc.calc() != wire_crc) {
    Serial.println(F("Error: Wrong CRC received in Domoticz Data frame"));
    return;
  }

  showValues(td, ed, pd);

  char buf[10];
  snprintf(buf, sizeof(buf), "%01hu.%02hu", (uint16_t)(ed.voltage / 10), (uint16_t)(ed.voltage % 10));
  updateDomoticz(DOMOTICZ_VOLTAGE_IDX, buf);
  snprintf(buf, sizeof(buf), "%01hu.%02hu", (uint16_t)(ed.current / 1000), (uint16_t)(ed.current % 1000));
  updateDomoticz(DOMOTICZ_CURRENT_IDX, buf);
  snprintf(buf, sizeof(buf), "%01hu.%02hu", (uint16_t)(ed.power / 10), (uint16_t)(ed.power % 10));
  updateDomoticz(DOMOTICZ_POWER_IDX, buf);
  snprintf(buf, sizeof(buf), "%01hu.%02hu", (uint16_t)(ed.pf / 100), (uint16_t)(ed.pf % 100));
  updateDomoticz(DOMOTICZ_PF_IDX, buf);

  snprintf(buf, sizeof(buf), "%01hd.%02hd", (int16_t)(td.heat / 100), (int16_t)(td.heat % 100));
  updateDomoticz(DOMOTICZ_HEAT_IDX, buf);
  snprintf(buf, sizeof(buf), "%01hd.%02hd", (int16_t)(td.ret / 100), (int16_t)(td.ret % 100));
  updateDomoticz(DOMOTICZ_RETURN_IDX, buf);
  snprintf(buf, sizeof(buf), "%01hd.%02hd", (int16_t)(td.outside / 100), (int16_t)(td.outside % 100));
  updateDomoticz(DOMOTICZ_OUTSIDE_IDX, buf);

  snprintf(buf, sizeof(buf), "%01hd.%02hd", (int16_t)(pd.sv / 100), (int16_t)(pd.sv % 100));
  updateDomoticz(DOMOTICZ_SV_IDX, buf);
  ultoa(map(pd.output, 0, 255, 0, 100), buf, 10);
  updateDomoticz(DOMOTICZ_OUTPUT_IDX, buf);
}

void printIPInfo() {
  Serial.print(F("Ethernet ip address: "));
  Serial.println(eth.localIP());
  Serial.print(F("Ethernet subnetMask: "));
  Serial.println(eth.subnetMask());
  Serial.print(F("Ethernet gateway: "));
  Serial.println(eth.gatewayIP());
}

void checkEthernetConnection() {
  static bool connectedETH_last_state;
  if (eth.connected()) {
    //Serial.println(F("eth connected"));
    if (!connectedETH_last_state) {
      Serial.println(F("eth now connected"));
      printIPInfo();
      connectToMqtt();
    }
    connectedETH_last_state = true;
  } else {
    //Serial.println("eth not connected");
    if (connectedETH_last_state) {
      Serial.println(F("eth now disconnected"));
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
  Serial.println(F("Connecting to MQTT..."));
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println(F("Connected to MQTT broker."));
  //Serial.print(F("Connected to MQTT broker: "));
  //Serial.print(MQTT_SERVER);
  //Serial.print(F(", port: "));
  //Serial.println(MQTT_PORT);

  connectedMQTT = true;

  Serial.println(F("------------------------------"));
  Serial.print(F("Session present: "));
  Serial.println(sessionPresent);

  mqttClient.publish(console_in_topic, 0, false, "Put Heat Pump command here.");
  mqttClient.publish(console_out_topic, 0, false, "Read Heat Pump command output from here.");

  uint16_t packetIdSub = mqttClient.subscribe(console_in_topic, 0);
  Serial.print(F("Subscribing: "));
  Serial.println(console_in_topic);
  Serial.println(F("------------------------------"));
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  (void)reason;
  connectedMQTT = false;
  Serial.println(F("Disconnected from MQTT."));
  if (eth.connected()) {
    connectToMqtt();
  }
}

void onMqttMessage(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties,
                   const size_t &len, const size_t &index, const size_t &total) {
  char message[len + 1];

  memcpy(message, payload, len);
  message[len] = 0x00;

  if (!strcmp(topic, console_in_topic)) {
    char cmd[5];
    uint8_t cmd_len = min(strcspn(message, " "), sizeof(cmd) - 1);
    Serial.println(cmd_len);
    memcpy(cmd, message, cmd_len);
    cmd[cmd_len] = 0x00;  // terminate string

    Serial.println(cmd);

    if (!strcmp(cmd, "set")) {
      processMQTTSet(message + cmd_len + 1);
    } else if (!strcmp(cmd, "get")) {
      processMQTTGet(message + cmd_len + 1);
    } else if (!strcmp(cmd, "save")) {
      processMQTTSave();
    } else {
      Serial.println(F("Unknown command"));
      mqttClient.publish(console_out_topic, 0, false, "Unknown command");
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

// =============== TEMPORARY CODE ===================

void switchWP(bool toggle) {
  Wire.beginTransmission(SLAVE_ADDR);
  if(toggle) {
    Wire.write(0x31);
    Serial.println(F("Switching watter pump on"));
    mqttClient.publish(console_out_topic, 0, false, "Switching watter pump on");
  } else {
    Wire.write(0x30);
    Serial.println(F("Switching watter pump off"));
    mqttClient.publish(console_out_topic, 0, false, "Switching watter pump off");
  }
  Wire.endTransmission();
}

void switchHP(bool toggle) {
  Wire.beginTransmission(SLAVE_ADDR);
  if(toggle) {
    Wire.write(0x41);
    Serial.println(F("Switching heating pump on"));
  } else {
    Wire.write(0x40);
    Serial.println(F("Switching heating pump off"));
  }
  Wire.endTransmission();
}

// ==================================================

void onMqttPublish(const uint16_t &packetId) {
  Serial.println(F("Publish acknowledged."));
  Serial.print(F("  packetId: "));
  Serial.println(packetId);
}

void processMQTTGet(char *param) {
  char buf[70];
  if (!strcmp(param, "pid")) {
    Serial.println(F("Calling for pid"));
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x02);  // get pid
    Wire.endTransmission();
    PidParams pid_params;
    Wire.requestFrom(SLAVE_ADDR, sizeof(pid_params));
    Wire.readBytes((byte *)&pid_params, sizeof(pid_params));
    snprintf(buf, sizeof(buf), "%.2f %.2f %.2f %hhu %hhu", pid_params.kp, pid_params.ki, pid_params.kd, pid_params.omin, pid_params.omax);
  } else if (!strcmp(param, "hc")) {
    Serial.println(F("Calling for hc"));
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x03);  // get hc
    Wire.endTransmission();
    HC hc;
    Wire.requestFrom(SLAVE_ADDR, sizeof(hc));
    Wire.readBytes((byte *)&hc, sizeof(hc));
    snprintf(buf, sizeof(buf), "%hu %hu %hu %hu", hc.hc_minus5, hc.hc_0, hc.hc_5, hc.hc_10);
  } else if (!strcmp(param, "temp")) {
    Serial.println(F("Calling for temp"));
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x04);  // get temp
    Wire.endTransmission();
    TempData td;
    Wire.requestFrom(SLAVE_ADDR, sizeof(td));
    Wire.readBytes((byte *)&td, sizeof(td));
    snprintf(buf, sizeof(buf), "%hd %hd %hd", td.heat, td.ret, td.outside);
  } else if (!strcmp(param, "energy")) {
    Serial.println(F("Calling for energy"));
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x05);  // get energy
    Wire.endTransmission();
    EnergyData ed;
    Wire.requestFrom(SLAVE_ADDR, sizeof(ed));
    Wire.readBytes((byte *)&ed, sizeof(ed));
    snprintf(buf, sizeof(buf), "Voltage: %01u.%01u V\nCurrent: %01u.%03u A\nPower: %01u.%01u W\nPF: %01u.%02u",
             ed.voltage / 10, ed.voltage % 10, ed.current / 1000, ed.current % 1000, ed.power / 10, ed.power % 10, ed.pf / 100, ed.pf % 100);
  } else {
    snprintf(buf, sizeof(buf), "Unknown parameter %s", param);
  }
  mqttClient.publish(console_out_topic, 0, false, buf);
  Serial.println(buf);
}

void processMQTTSet(char *param) {
  char buf[50];
  Serial.println(F("Set command"));
  Serial.println(param);
  char *values;
  uint8_t args_assigned;
  if (!strncmp(param, "pid", 3)) {
    values = param + 4;
    Serial.println(F("Trying to set pid"));
    PidParams pid_params;
    Serial.println(F("Before sscanf"));
    args_assigned = sscanf(values, "%f %f %f %hhu %hhu", &pid_params.kp, &pid_params.ki, &pid_params.kd, &pid_params.omin, &pid_params.omax);
    Serial.println(F("After sscanf"));
    if (args_assigned == 5) {
      Serial.println(F("Setting pid"));
      strcpy(buf, "Setting pid");

      Serial.println(pid_params.kp);
      Serial.println(pid_params.ki);
      Serial.println(pid_params.kd);
      Serial.println(pid_params.omin);
      Serial.println(pid_params.omax);
      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write(0x12);  // set pid cmd
      Wire.write((byte *)&pid_params, sizeof(pid_params));
      Wire.endTransmission();
    } else {
      strcpy(buf, "Wrong values for pid");
    }
  } else if (!strncmp(param, "hc", 2)) {
    values = param + 3;
    Serial.println(F("Trying to set hc"));
    HC hc;
    Serial.println(F("Before sscanf"));
    args_assigned = sscanf(values, "%hu %hu %hu %hu", &hc.hc_minus5, &hc.hc_0, &hc.hc_5, &hc.hc_10);
    Serial.println(F("After sscanf"));
    if (args_assigned == 4) {
      Serial.println(F("Setting hc"));
      strcpy(buf, "Setting hc");

      Serial.println(hc.hc_minus5);
      Serial.println(hc.hc_0);
      Serial.println(hc.hc_5);
      Serial.println(hc.hc_10);
      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write(0x13);  // set hc cmd
      Wire.write((byte *)&hc, sizeof(hc));
      Wire.endTransmission();
    } else {
      strcpy(buf, "Wrong values for hc");
    }
  // ================ TEMPORARY CODE =============
  } else if (!strncmp(param, "hp_on", 5)) {
    switchHP(true);
    strcpy(buf, "HP ON");
  } else if (!strncmp(param, "hp_off", 6)) {
    switchHP(false);
    strcpy(buf, "HP OFF");
  } else if (!strncmp(param, "wp_on", 5)) {
    switchWP(true);
    strcpy(buf, "WP ON");
  } else if (!strncmp(param, "wp_off", 6)) {
    switchWP(false);
    strcpy(buf, "WP OFF");
  // =============================================
  } else {
    snprintf(buf, sizeof(buf), "Unknown parameter %s", param);
  }
  mqttClient.publish(console_out_topic, 0, false, buf);
}

void processMQTTSave() {
  Serial.println(F("Saving to EEPROM"));
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0x20);  // save cmd
  Wire.endTransmission();
}

void updateDomoticz(uint16_t idx, char *svalue) {
  char buf[50];
  snprintf(buf, sizeof(buf), "{\"idx\": %hu, \"svalue\": \"%s\"}", idx, svalue);
  mqttClient.publish(domoticz_in_topic, 0, false, buf);
}
