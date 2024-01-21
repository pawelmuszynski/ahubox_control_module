/*
 * Almost all calculations are fixed point. So values are multiplied
 * by 10, 100 or 1000 to get decimal precision. For instance temperatures
 * and are multiplied by 100. Keep it in mind during PID tuning.
 *
 * It's based on:
 * Timer library from https://github.com/JChristensen/Timer
 * Modified PIDv1 library from https://github.com/br3ttb/Arduino-PID-Library
 * Modified PZEM004Tv30 library from https://github.com/mandulaj/PZEM-004T-v30
 *
*/

#include <EEPROM.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "Timer.h"
#include "PID_v1.h"
#include "PZEM004Tv30.h"

#define CHECK_PERIOD 1000UL
#define CHECK_PULSE_THRESHOLD 107  // ~ 0.8 m^3 / h

#define INTERRUPT_PIN 2
#define DS_PIN 4
#define HEAT_SWITCH_PIN 5
#define PWM_PIN 6

// EEPROM addresses
#define KP_ADDR 0x00  // to 0x03 (float)
#define KI_ADDR 0x04  // to 0x07 (float)
#define KD_ADDR 0x08  // to 0x0b (float)

#define HEAT_DS_ADDR 0x10     // to 0x17 (8x uint8_t)
#define RETURN_DS_ADDR 0x18   // to 0x1f (8x uint8_t)
#define OUTSIDE_DS_ADDR 0x20  // to 0x27 (8x uint8_t)

#define PID_MIN_ADDR 0x30  // uint8_t
#define PID_MAX_ADDR 0x31  // uint8_t

#define HC_MINUS5_ADDR 0x40   // to 0x41 (uint16_t)
#define HC_0_ADDR 0x42        // to 0x43 (uint16_t)
#define HC_5_ADDR 0x44        // to 0x45 (uint16_t)
#define HC_10_ADDR 0x46       // to 0x47 (uint16_t)

const char line00[] PROGMEM = "Available commands:";
const char line01[] PROGMEM = "  help - this help";
const char line02[] PROGMEM = "  set <param> <value> - to set a value to the parameter";
const char line03[] PROGMEM = "  get <param> - to get value of the paramater";
const char line04[] PROGMEM = "";
const char line05[] PROGMEM = "  get hc - to get all heat curve parameters";
const char line06[] PROGMEM = "  get ds - to get all available ds termometers";
const char line07[] PROGMEM = "  get pid - to get all pid parameters";
const char line08[] PROGMEM = "  set factory defaults - reset to factory defaults";
const char line09[] PROGMEM = "";
const char line10[] PROGMEM = "Available parameters:";
const char line11[] PROGMEM = "  kp - pid proportional multiplier";
const char line12[] PROGMEM = "  ki - pid integral multiplier";
const char line13[] PROGMEM = "  kd - pid derivative multiplier";
const char line14[] PROGMEM = "  pid_min - min pid output value";
const char line15[] PROGMEM = "  pid_max - max pid output value";
const char line16[] PROGMEM = "";
const char line17[] PROGMEM = "  hc_minus5 - heat curve heating operating point for -5 degree Celsius outside";
const char line18[] PROGMEM = "  hc_0 - heat curve heating operating point for 0 degree Celsius outside";
const char line19[] PROGMEM = "  hc_5 - heat curve heating operating point for 5 degree Celsius outside";
const char line20[] PROGMEM = "  hc_10 - heat curve heating operating point for 10 degree Celsius outside";
const char line21[] PROGMEM = "";
const char line22[] PROGMEM = "Remark: almost all calculations are fixed point, so they are multiplied by 10, 100 or 100.";
const char line23[] PROGMEM = "For instance temperatures and PID kp, ki, kd parameters are multiplied by 100.";
const char line24[] PROGMEM = "";

const char * const help_content[] PROGMEM = {
  line00, line01, line02, line03, line04, line05, line06, line07, line08, line09,
  line10, line11, line12, line13, line14, line15, line16, line17, line18, line19,
  line20, line21, line22, line23, line24
};

const uint8_t ds_precision=12;

float kp, ki, kd;
uint8_t heat_ds[8], return_ds[8], outside_ds[8];
uint8_t pid_min, pid_max;
uint16_t hc_minus5, hc_0, hc_5, hc_10;

Timer timer;
PZEM004Tv30 pzem(Serial2);
OneWire ow(DS_PIN);
DallasTemperature ds(&ow);

const uint8_t n_probes = 6;

uint16_t heat_temp_avg, return_temp_avg;
uint16_t heat_temp[n_probes], return_temp[n_probes];

int16_t outside_temp_avg;
int16_t outside_temp[n_probes];

uint16_t voltage_avg, current_avg, power_avg, energy_avg, pf_avg;
uint16_t voltage[n_probes], current[n_probes], power[n_probes], pf[n_probes];

uint16_t pid_output, pid_sv;
PID pid(&heat_temp_avg, &pid_output, &pid_sv, kp, ki, kd, DIRECT);

uint16_t pulse_counter = 0;
bool alarm_flag = false;

void printPIDParams() {
  Serial.println("--------- PID --------");
  Serial.println("kp = " + String(kp));
  Serial.println("ki = " + String(ki));
  Serial.println("kd = " + String(kd));
  Serial.println("pid_min = " + String(pid_min));
  Serial.println("pid_max = " + String(pid_max));
  Serial.println();
}

void printHCParams() {
  Serial.println("---- Heating curve ----");
  Serial.println("hc_minus5 = " + String(hc_minus5));
  Serial.println("hc_0 = " + String(hc_0));
  Serial.println("hc_5 = " + String(hc_5));
  Serial.println("hc_10 = " + String(hc_10));
  Serial.println();
}

void printDSAddr(uint8_t *array) {
  Serial.print("Address: ");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(array[i], HEX);
    if (i < 7) Serial.print("-");
    else Serial.println();
  }
}

void printAvailableDS() {
  uint8_t addr[8];
  Serial.println("Available DS devices:");
  while (ow.search(addr)) {  // List all DS devices
    printDSAddr(addr);
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("Error: DS CRC is invalid.");
      return;
    }
    Serial.print("Power mode: ");
    if (ds.isParasitePowerMode()) Serial.println("Parasite");
    else Serial.println("External");
    Serial.println("---------------------");
  }
  ow.reset_search();
}

void showValues() {
  char buf[30];
  sprintf(buf, "Voltage: %01u.%01u V", voltage_avg/10, voltage_avg%10);
  Serial.println(buf);
  sprintf(buf, "Current: %01u.%03u A", current_avg/1000, current_avg%1000);
  Serial.println(buf);
  sprintf(buf, "Power: %01u.%01u W", power_avg/10, current_avg%10);
  Serial.println(buf);
  sprintf(buf, "PF: %01u.%02u", pf_avg/100, pf_avg%100);
  Serial.println(buf);
  sprintf(buf, "heat_temp: %01d.%02d °C", heat_temp_avg/100, heat_temp_avg%100);
  Serial.println(buf);
  sprintf(buf, "return_temp: %01d.%02d °C", return_temp_avg/100, return_temp_avg%100);
  Serial.println(buf);
  sprintf(buf, "outside_temp: %01d.%02d °C", outside_temp_avg/100, outside_temp_avg%100);
  Serial.println(buf);
  sprintf(buf, "pid_sv: %01d.%02d °C", pid_sv/100, pid_sv%100);
  Serial.println(buf);
  sprintf(buf, "set power: %d (%d%%)", pid_output, map(pid_output, 0, 255, 0, 100));
  Serial.println(buf);
}

int16_t avgCalc(int16_t *probes, const uint8_t &n_probes) {
  int16_t sum=0;
  for(uint8_t i=0; i<n_probes; i++) {
    sum += probes[i];
  }
  return sum /= n_probes;
}

void avgCalcAll() {
  voltage_avg = avgCalc(voltage, n_probes);
  current_avg = avgCalc(current, n_probes);
  power_avg = avgCalc(power, n_probes);
  pf_avg = avgCalc(pf, n_probes);
  heat_temp_avg = avgCalc(heat_temp, n_probes);
  return_temp_avg = avgCalc(return_temp, n_probes);
  outside_temp_avg = avgCalc(outside_temp, n_probes);
}

void printHelp() {
  char buf[100];
  for(uint8_t i=0; i<sizeof(help_content)/sizeof(help_content[0]); i++) {
    strcpy_P(buf, (char *)pgm_read_word(&(help_content[i])));
    Serial.println(buf);
  }
}

int16_t getHCValue(int16_t outside_temp) {
  /* 
    Linear approximation between given setpoints
    Generic math formula:
    to1 = outside temp1
    to2 = outside temp2
    th1 = heating temp1 for to1
    th2 = heating temp2 for to2

    f(to) = ((th1 - th2)/(to1 - to2)) * (to - to1) + th1
    
    In this case we have 7 setpoints for each 5 degrees (to2 = to1 + 5).
    The simplified formula is following:
    f(to) = ((th2 - th1)/5) * (to - to1) + th1
  */

  int16_t th1, th2, to1;

  if (outside_temp >= 1000) {
    return hc_10;
  } else if (outside_temp >= 500 and outside_temp < 1000) {
    th1 = hc_5;
    th2 = hc_10;
    to1 = 500;
  } else if (outside_temp >= 0 and outside_temp < 500) {
    th1 = hc_0;
    th2 = hc_5;
    to1 = 0;
  } else if (outside_temp >= -500 and outside_temp < 0) {
    th1 = hc_minus5;
    th2 = hc_0;
    to1 = -500;
  } else if (outside_temp < -500) {
    return hc_minus5;
  }
  return ((th2 - th1) / 500) * (outside_temp - to1) + th1;
}

void processGet(char *param) {
  if(!strcmp(param, "ds")) {
    printAvailableDS();
  }
  else if(!strcmp(param, "hc")) {
    printHCParams();
  }
  else if(!strcmp(param, "pid")) {
    printPIDParams();
  }
  else {
    Serial.print("Param: ");
    Serial.println(param);
    Serial.print("Value: ");
    if(!strcmp(param, "kp")) {
      Serial.println(kp);
    }
    else if(!strcmp(param, "ki")) {
      Serial.println(ki);
    }
    else if(!strcmp(param, "kd")) {
      Serial.println(kd);
    }
    else if(!strcmp(param, "pid_min")) {
      Serial.println(pid_min);
    }
    else if(!strcmp(param, "pid_max")) {
      Serial.println(pid_max);
    }
    else if(!strcmp(param, "hc_minus5")) {
      Serial.println(hc_minus5);
    }
    else if(!strcmp(param, "hc_0")) {
      Serial.println(hc_0);
    }
    else if(!strcmp(param, "hc_5")) {
      Serial.println(hc_5);
    }
    else if(!strcmp(param, "hc_10")) {
      Serial.println(hc_10);
    }
    else {
      Serial.println("Parameter not found.");
    }
  }
}

void processSet(char *buf) {
  char param[30], value[30];
  //char *delimiter_ptr=strchr(buf, ' ');
  //char *param = (char*) malloc(delimiter_ptr - buf + 1);
  //char *value = (char*) malloc(strlen(delimiter_ptr));
  sscanf(buf, "%s %s", param, value);
  if(!strcmp(param, "kp")) {
    kp = atof(value);
    EEPROM.put(KP_ADDR, kp);
    pid.SetTunings(kp, ki, kd);
  }
  else if(!strcmp(param, "ki")) {
    ki = atof(value);
    EEPROM.put(KI_ADDR, ki);
    pid.SetTunings(kp, ki, kd);
  }
  else if(!strcmp(param, "kd")) {
    kd = atof(value);
    EEPROM.put(KD_ADDR, kd);
    pid.SetTunings(kp, ki, kd);
  }
  else if(!strcmp(param, "pid_min")) {
    pid_min = atoi(value);
    EEPROM.put(PID_MIN_ADDR, pid_min);
    pid.SetOutputLimits(pid_min, pid_max);
  }
  else if(!strcmp(param, "pid_max")) {
    pid_max = atoi(value);
    EEPROM.put(PID_MAX_ADDR, pid_max);
    pid.SetOutputLimits(pid_min, pid_max);
  }
  else if(!strcmp(param, "hc_minus5")) {
    hc_minus5 = atof(value);
    EEPROM.put(HC_MINUS5_ADDR, hc_minus5);
  }
  else if(!strcmp(param, "hc_0")) {
    hc_0 = atof(value);
    EEPROM.put(HC_0_ADDR, hc_0);
  }
  else if(!strcmp(param, "hc_5")) {
    hc_5 = atof(value);
    EEPROM.put(HC_5_ADDR, hc_5);
  }
  else if(!strcmp(param, "hc_10")) {
    hc_10 = atof(value);
    EEPROM.put(HC_10_ADDR, hc_10);
  }
  else if(!strcmp(param, "heat_ds")) {
    sscanf(value, "%x-%x-%x-%x-%x-%x-%x-%x", &heat_ds[0], &heat_ds[1], &heat_ds[2], &heat_ds[3], &heat_ds[4], &heat_ds[5], &heat_ds[6], &heat_ds[7]);
    EEPROM.put(HEAT_DS_ADDR, heat_ds);
  }
  else if(!strcmp(param, "return_ds")) {
    sscanf(value, "%x-%x-%x-%x-%x-%x-%x-%x", &return_ds[0], &return_ds[1], &return_ds[2], &return_ds[3], &return_ds[4], &return_ds[5], &return_ds[6], &return_ds[7]);
    EEPROM.put(RETURN_DS_ADDR, return_ds);
  }
  else if(!strcmp(param, "outside_ds")) {
    sscanf(value, "%x-%x-%x-%x-%x-%x-%x-%x", &outside_ds[0], &outside_ds[1], &outside_ds[2], &outside_ds[3], &outside_ds[4], &outside_ds[5], &outside_ds[6], &outside_ds[7]);
    EEPROM.put(OUTSIDE_DS_ADDR, outside_ds);
  }
  else if(!strcmp(param, "factory") && !strcmp(value, "defaults")) {
    processSet("kp 20");
    processSet("ki 0.02");
    processSet("kd 0");
    processSet("pid_min 26");
    processSet("pid_max 255");
    processSet("hc_minus5 4200");
    processSet("hc_0 3700");
    processSet("hc_5 3200");
    processSet("hc_10 2700");
  }
  else {
    Serial.println("Parameter not found.");
  }
}

void processCommand(char *buf) {
  Serial.println("Received command:");
  Serial.println(buf);
  if(!strncmp(buf, "get ", 4)) {  // get command
    processGet(buf + 4);
  }
  else if(!strncmp(buf, "set ", 4)) {  // set command
    processSet(buf + 4);
  }
  else if(!strncmp(buf, "help", 4)) {  // help command
    printHelp();
  }
  else {
    Serial.println("Command not found. Use help command.");
  }
  Serial.println("==================");
}

void setup() {
  Serial.begin(9600);
  Serial.println("================== START ====================");

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(HEAT_SWITCH_PIN, OUTPUT);
  digitalWrite(HEAT_SWITCH_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onPulse, FALLING);

  // ====== Heating curve ======
  EEPROM.get(HC_MINUS5_ADDR, hc_minus5);
  EEPROM.get(HC_0_ADDR, hc_0);
  EEPROM.get(HC_5_ADDR, hc_5);
  EEPROM.get(HC_10_ADDR, hc_10);
  printHCParams();

  // =========== PID ===========
  EEPROM.get(KP_ADDR, kp);
  EEPROM.get(KI_ADDR, ki);
  EEPROM.get(KD_ADDR, kd);
  EEPROM.get(PID_MIN_ADDR, pid_min);
  EEPROM.get(PID_MAX_ADDR, pid_max);
  printPIDParams();
  pid.SetTunings(kp, ki, kd);
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10000);
  pid.SetOutputLimits(pid_min, pid_max);

  // ========== DS =============
  ds.begin();
  printAvailableDS();
  EEPROM.get(HEAT_DS_ADDR, heat_ds);
  EEPROM.get(RETURN_DS_ADDR, return_ds);
  EEPROM.get(OUTSIDE_DS_ADDR, outside_ds);

  Serial.print("Heat DS: ");
  printDSAddr(heat_ds);
  Serial.print("Return DS ");
  printDSAddr(return_ds);
  Serial.print("Outisde DS: ");
  printDSAddr(outside_ds);

  // setup DS precision
  if (ds.getResolution(heat_ds) != ds_precision) ds.setResolution(ds_precision);
  if (ds.getResolution(return_ds) != ds_precision) ds.setResolution(ds_precision);
  if (ds.getResolution(outside_ds) != ds_precision) ds.setResolution(ds_precision);

  // --- pzem ---
  char buf[18];
  char buf2[4];
  strcpy(buf, "PZEM Address: ");
  itoa(pzem.readAddress(), buf2, 16);
  strcat(buf, buf2);
  Serial.println(buf);

  // --- timers ---
  timer.every(1000, on1Sec);
  timer.every(10000, on10Sec);
  timer.every(1000, flowAlarmCheck);
}

void loop() {
  timer.update();
  pid.Compute();

  // ======== UART commands support ==========
  char received = NULL;
  static char serial_buf[50];
  static uint8_t buf_index = 0;
  if (Serial.available()) {
    received = Serial.read();
    if(received == '\n' || received == '\r') {  // command fully received
      processCommand(serial_buf);
      serial_buf[0] = '\0';  // clear buffer
      buf_index = 0;
    } else {  // command not fully received, attaching the last character
      serial_buf[buf_index] = received;
      serial_buf[buf_index+1] = '\0';
      buf_index++;
    }
  }

  if(alarm_flag) {
    digitalWrite(HEAT_SWITCH_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void onPulse() {
  pulse_counter++;
  //Serial.println("pulse");
}

uint8_t i = 0;
void on1Sec() {
    voltage[i] = pzem.voltage();  // x 0.1V
    current[i] = pzem.current();  // x 0.001A
    power[i] = pzem.power();      // x 0.1W
    pf[i] = pzem.pf();            // x 0.01
    ds.requestTemperatures();
    heat_temp[i] = (ds.getTemp(heat_ds)*78125+50000)/100000;        // x 0.01 C
    return_temp[i] = (ds.getTemp(return_ds)*78125+50000)/100000;    // x 0.01 C
    outside_temp[i] = (ds.getTemp(outside_ds)*78125+50000)/100000;  // x 0.01 C
    i++;
    if(i>=n_probes) i=0;
}

void on10Sec() {
  Serial.println("====== ON 10 sec ======");
  avgCalcAll();
  pid_sv = getHCValue(outside_temp_avg);
  digitalWrite(PWM_PIN, pid_sv);
  showValues();
  Serial.println("=======================");
}

void flowAlarmCheck() {
  //char str_buf[3];
  //itoa(pulse_counter, str_buf, 10);
  //Serial.println(str_buf);
  if(pulse_counter < CHECK_PULSE_THRESHOLD) {
    alarm_flag = true;
    //Serial.println(String(pulse_counter) + " is below " + String(CHECK_PULSE_THRESHOLD));
    //Serial.println("rise ALARM!");
  }
  pulse_counter = 0;
}
