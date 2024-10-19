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
#include <CRC8.h>
//#include <OneButton.h>
//#include "Timer.h"
#include "PID_v1.h"
#include "PZEM004Tv30.h"

//#define CHECK_PERIOD 1000UL
#define CHECK_PULSE_THRESHOLD 300  // 107/s = ~ 0.8 m^3 / h

#define INTERRUPT_PIN 2
#define DS_PIN 4
#define PROG_DS_PIN 3
#define HEAT_SWITCH_PIN 5
#define PWM_PIN 6
#define HEAT_DS_PROG_BUTTON 10
#define RETURN_DS_PROG_BUTTON 11
#define OUTSIDE_DS_PROG_BUTTON 12
#define BUZZER_PIN LED_BUILTIN  //20
#define SOFTSERIAL_RX_PIN 23
#define SOFTSERIAL_TX_PIN 24

// EEPROM addresses
#define KP_ADDR 0x00  // to 0x03 (float)
#define KI_ADDR 0x04  // to 0x07 (float)
#define KD_ADDR 0x08  // to 0x0b (float)
//#define KP_ADDR 0x00  // to 0x01 (uint16_t)
//#define KI_ADDR 0x02  // to 0x03 (uint16_t)
//#define KD_ADDR 0x04  // to 0x07 (uint32_t)

#define HEAT_DS_ADDR 0x10     // to 0x17 (8x uint8_t)
#define RETURN_DS_ADDR 0x18   // to 0x1f (8x uint8_t)
#define OUTSIDE_DS_ADDR 0x20  // to 0x27 (8x uint8_t)

#define OMIN_ADDR 0x30  // uint8_t
#define OMAX_ADDR 0x31  // uint8_t

#define HC_MINUS5_ADDR 0x40  // to 0x41 (uint16_t)
#define HC_0_ADDR 0x42       // to 0x43 (uint16_t)
#define HC_5_ADDR 0x44       // to 0x45 (uint16_t)
#define HC_10_ADDR 0x46      // to 0x47 (uint16_t)

#define WIRE_SLAVE_ADDR 8
#define WIRE_DATA_FRAME 0xAA // It identifies data frame (crc is not enough). It should be equal on both sides.

/*
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
const char line14[] PROGMEM = "  omin - min pid output value";
const char line15[] PROGMEM = "  omax - max pid output value";
const char line16[] PROGMEM = "";
const char line17[] PROGMEM = "  hc_minus5 - heat curve heating operating point for -5 degree Celsius outside";
const char line18[] PROGMEM = "  hc_0 - heat curve heating operating point for 0 degree Celsius outside";
const char line19[] PROGMEM = "  hc_5 - heat curve heating operating point for 5 degree Celsius outside";
const char line20[] PROGMEM = "  hc_10 - heat curve heating operating point for 10 degree Celsius outside";
const char line21[] PROGMEM = "";
const char line22[] PROGMEM = "Remark: almost all calculations are fixed point, so they are multiplied by 10, 100 or 100.";
const char line23[] PROGMEM = "For instance temperatures and PID kp, ki, kd parameters are multiplied by 100.";
const char line24[] PROGMEM = "";

const char *const help_content[] PROGMEM = {
  line00, line01, line02, line03, line04, line05, line06, line07, line08, line09,
  line10, line11, line12, line13, line14, line15, line16, line17, line18, line19,
  line20, line21, line22, line23, line24
};
*/
const uint8_t n_probes = 16;

struct EnergyData {
  uint16_t voltage, current, power, energy, pf;
};

struct __attribute__((aligned(4))) PidParams {
  float kp, ki, kd;
  uint8_t omin, omax;
};

struct __attribute__((aligned(4))) PidData {
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

EnergyData energy_avg;
EnergyData energy_probes[n_probes];
PidParams pid_params;
PidData pid_data;
TempData temp_avg;
TempData temp_probes[n_probes];
HC heat_curve;
//OneButton heat_ds_prog_button, return_ds_prog_button, outside_ds_prog_button;

uint8_t timer_start_millis;
uint8_t loop_counter = 15;


bool ds_prog_button_last_state = HIGH;
bool ds_prog_button_current_state;

bool heat_ds_prog_button_last_state = HIGH;
bool return_ds_prog_button_last_state = HIGH;
bool outside_ds_prog_button_last_state = HIGH;
bool heat_ds_prog_button_current_state;
bool return_ds_prog_button_current_state;
bool outside_ds_prog_button_current_state;
uint8_t button_pressed_time = 0;
uint8_t button_released_time = 0;

const uint8_t ds_precision = 12;
uint8_t heat_ds[8], return_ds[8], outside_ds[8];

//Timer timer;
//PZEM004Tv30 pzem(Serial2);
OneWire ow(DS_PIN);
DallasTemperature ds(&ow);

PID pid(&temp_avg.heat, &pid_data.output, &pid_data.sv, pid_params.kp, pid_params.ki, pid_params.kd, DIRECT);

uint16_t pulse_counter = 0;
bool alarm_flag = false;

bool beepOK, beepNOK, beep_last_state;
uint8_t beep_start_millis;

uint8_t wire_cmd;

void printPIDParams() {
  char buf[20];
  Serial.println("- PID -");
  sprintf(buf, "kp = %f", pid_params.kp);
  Serial.println(buf);
  sprintf(buf, "ki = %f", pid_params.ki);
  Serial.println(buf);
  sprintf(buf, "kd = %f", pid_params.kd);
  Serial.println(buf);
  sprintf(buf, "pid_min = %uud", pid_params.omin);
  Serial.println(buf);
  sprintf(buf, "pid_max = %uud", pid_params.omax);
  Serial.println(buf);
  Serial.println();
}

void printHCParams() {
  char buf[20];
  Serial.println("- Heating curve -");
  sprintf(buf, "hc_minus5 = %d", heat_curve.hc_minus5);
  Serial.println(buf);
  sprintf(buf, "hc_0 = %d", heat_curve.hc_0);
  Serial.println(buf);
  sprintf(buf, "hc_5 = %d", heat_curve.hc_5);
  Serial.println(buf);
  sprintf(buf, "hc_10 = %d", heat_curve.hc_10);
  Serial.println(buf);
  Serial.println();
}
/*
void printDSAddr(uint8_t *array) {
  Serial.print("Address: ");
  for(uint8_t i = 0; i < 8; i++) {
    Serial.print(array[i], HEX);
    if(i < 7) Serial.print("-");
    else Serial.println();
  }
}
*/
/*
void printAvailableDS() {
  uint8_t addr[8];
  Serial.println("Available DS devices:");
  while (ow.search(addr)) {  // List all DS devices
    printDSAddr(addr);
    if(OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("Error: DS CRC is invalid.");
      return;
    }
    Serial.print("Power mode: ");
    if(ds.isParasitePowerMode()) Serial.println("Parasite");
    else Serial.println("External");
    Serial.println("---------------------");
  }
  ow.reset_search();
}
*/

void showValues() {
  char buf[30];
  snprintf(buf, sizeof(buf), "Voltage: %01u.%01u V", energy_avg.voltage / 10, energy_avg.voltage % 10);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "Current: %01u.%03u A", energy_avg.current / 1000, energy_avg.current % 1000);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "Power: %01u.%01u W", energy_avg.power / 10, energy_avg.power % 10);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "PF: %01u.%02u", energy_avg.pf / 100, energy_avg.pf % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "heat_temp: %01d.%02d °C", temp_avg.heat / 100, temp_avg.heat % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "return_temp: %01d.%02d °C", temp_avg.ret / 100, temp_avg.ret % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "outside_temp: %01d.%02d °C", temp_avg.outside / 100, temp_avg.outside % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "pid_sv: %01d.%02d °C", pid_data.sv / 100, pid_data.sv % 100);
  Serial.println(buf);
  snprintf(buf, sizeof(buf), "set power: %d (%d%%)", pid_data.output, map(pid_data.output, 0, 255, 0, 100));
  Serial.println(buf);
}
/*
int16_t avgCalc(int16_t *probes, const uint8_t &n_probes) {
  int32_t sum = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    sum += probes[i];
  }
  return sum /= n_probes;
}
*/

void avgCalcAll() {

  energy_avg.voltage = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    energy_avg.voltage += energy_probes[i].voltage / n_probes;
  }

  energy_avg.current = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    energy_avg.current += energy_probes[i].current / n_probes;
  }

  energy_avg.power = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    energy_avg.power += energy_probes[i].power / n_probes;
  }

  energy_avg.pf = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    energy_avg.pf += energy_probes[i].pf / n_probes;
  }

  temp_avg.heat = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    temp_avg.heat += temp_probes[i].heat / n_probes;
  }

  temp_avg.ret = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    temp_avg.ret += temp_probes[i].ret / n_probes;
  }

  temp_avg.outside = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    temp_avg.outside += temp_probes[i].outside / n_probes;
  }


  /*
  int32_t sum = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    sum += energy_probes[i].voltage;
  }
  energy_avg.voltage = sum/n_probes;

  sum = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    sum += energy_probes[i].current;
  }
  energy_avg.current = sum/n_probes;

  sum = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    sum += energy_probes[i].power;
  }
  energy_avg.power = sum/n_probes;

  sum = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    sum += energy_probes[i].pf;
  }
  energy_avg.pf = sum/n_probes;

  sum = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    sum += temp_probes[i].heat;
  }
  temp_avg.heat = sum/n_probes;

  sum = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    sum += temp_probes[i].ret;
  }
  temp_avg.ret = sum/n_probes;

  sum = 0;
  for(uint8_t i = 0; i < n_probes; i++) {
    sum += temp_probes[i].outside;
  }
  temp_avg.outside = sum/n_probes;
*/

  /*
int32_t voltage_sum = 0;
int32_t current_sum = 0;
int32_t power_sum = 0;
int32_t pf_sum = 0;
int32_t heat_sum = 0;
int32_t ret_sum = 0;
int32_t outside_sum = 0;

  for(uint8_t i = 0; i < n_probes; i++) {
    voltage_sum += energy_probes[i].voltage;
    current_sum += energy_probes[i].current;
    power_sum += energy_probes[i].power;
    pf_sum += energy_probes[i].pf;
    heat_sum += temp_probes[i].heat;
    ret_sum += temp_probes[i].ret;
    outside_sum += temp_probes[i].outside;
  }
  Serial.println("outside sum: " + String(outside_sum));
  energy_avg.voltage = voltage_sum/n_probes;
  energy_avg.current = current_sum/n_probes;
  energy_avg.power = power_sum/n_probes;
  energy_avg.pf = pf_sum/n_probes;
  temp_avg.heat = heat_sum/n_probes;
  temp_avg.ret = ret_sum/n_probes;
  temp_avg.outside = outside_sum/n_probes;
*/
}
/*
void printHelp() {
  char buf[100];
  for(uint8_t i = 0; i < sizeof(help_content) / sizeof(help_content[0]); i++) {
    strcpy_P(buf, (char *)pgm_read_word(&(help_content[i])));
    Serial.println(buf);
  }
}
*/
int16_t getHCValue(HC hc, int16_t outside_temp) {
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

  if(outside_temp >= 1000) {
    return hc.hc_10;
  } else if(outside_temp >= 500 and outside_temp < 1000) {
    th1 = hc.hc_5;
    th2 = hc.hc_10;
    to1 = 500;
  } else if(outside_temp >= 0 and outside_temp < 500) {
    th1 = hc.hc_0;
    th2 = hc.hc_5;
    to1 = 0;
  } else if(outside_temp >= -500 and outside_temp < 0) {
    th1 = hc.hc_minus5;
    th2 = hc.hc_0;
    to1 = -500;
  } else if(outside_temp < -500) {
    return hc.hc_minus5;
  }
  return ((th2 - th1) / 500) * (outside_temp - to1) + th1;
}
/*
void processGet(char *param) {
  if(!strcmp(param, "ds")) {
    //printAvailableDS();
  } else if(!strcmp(param, "hc")) {
    printHCParams();
  } else if(!strcmp(param, "pid")) {
    printPIDParams();
  } else {
    Serial.print("Param: ");
    Serial.println(param);
    Serial.print("Value: ");
    if(!strcmp(param, "kp")) {
      Serial.println(pid_params.kp);
    } else if(!strcmp(param, "ki")) {
      Serial.println(pid_params.ki);
    } else if(!strcmp(param, "kd")) {
      Serial.println(pid_params.kd);
    } else if(!strcmp(param, "omin")) {
      Serial.println(pid_params.omin);
    } else if(!strcmp(param, "omax")) {
      Serial.println(pid_params.omax);
    } else if(!strcmp(param, "hc_minus5")) {
      Serial.println(heat_curve.hc_minus5);
    } else if(!strcmp(param, "hc_0")) {
      Serial.println(heat_curve.hc_0);
    } else if(!strcmp(param, "hc_5")) {
      Serial.println(heat_curve.hc_5);
    } else if(!strcmp(param, "hc_10")) {
      Serial.println(heat_curve.hc_10);
    } else {
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
    pid_params.kp = atof(value);
    EEPROM.put(KP_ADDR, pid_params.kp);
    pid.SetTunings(pid_params.kp, pid_params.ki, pid_params.kd);
  } else if(!strcmp(param, "ki")) {
    pid_params.ki = atof(value);
    EEPROM.put(KI_ADDR, pid_params.ki);
    pid.SetTunings(pid_params.kp, pid_params.ki, pid_params.kd);
  } else if(!strcmp(param, "kd")) {
    pid_params.kd = atof(value);
    EEPROM.put(KD_ADDR, pid_params.kd);
    pid.SetTunings(pid_params.kp, pid_params.ki, pid_params.kd);
  } else if(!strcmp(param, "omin")) {
    pid_params.omin = atoi(value);
    EEPROM.put(OMIN_ADDR, pid_params.omin);
    pid.SetOutputLimits(pid_params.omin, pid_params.omax);
  } else if(!strcmp(param, "omax")) {
    pid_params.omax = atoi(value);
    EEPROM.put(OMAX_ADDR, pid_params.omax);
    pid.SetOutputLimits(pid_params.omin, pid_params.omin);
  } else if(!strcmp(param, "hc_minus5")) {
    heat_curve.hc_minus5 = atof(value);
    EEPROM.put(HC_MINUS5_ADDR, heat_curve.hc_minus5);
  } else if(!strcmp(param, "hc_0")) {
    heat_curve.hc_0 = atof(value);
    EEPROM.put(HC_0_ADDR, heat_curve.hc_0);
  } else if(!strcmp(param, "hc_5")) {
    heat_curve.hc_5 = atof(value);
    EEPROM.put(HC_5_ADDR, heat_curve.hc_5);
  } else if(!strcmp(param, "hc_10")) {
    heat_curve.hc_10 = atof(value);
    EEPROM.put(HC_10_ADDR, heat_curve.hc_10);
  } else if(!strcmp(param, "heat_ds")) {
    sscanf(value, "%x-%x-%x-%x-%x-%x-%x-%x", &heat_ds[0], &heat_ds[1], &heat_ds[2], &heat_ds[3], &heat_ds[4], &heat_ds[5], &heat_ds[6], &heat_ds[7]);
    EEPROM.put(HEAT_DS_ADDR, heat_ds);
  } else if(!strcmp(param, "return_ds")) {
    sscanf(value, "%x-%x-%x-%x-%x-%x-%x-%x", &return_ds[0], &return_ds[1], &return_ds[2], &return_ds[3], &return_ds[4], &return_ds[5], &return_ds[6], &return_ds[7]);
    EEPROM.put(RETURN_DS_ADDR, return_ds);
  } else if(!strcmp(param, "outside_ds")) {
    sscanf(value, "%x-%x-%x-%x-%x-%x-%x-%x", &outside_ds[0], &outside_ds[1], &outside_ds[2], &outside_ds[3], &outside_ds[4], &outside_ds[5], &outside_ds[6], &outside_ds[7]);
    EEPROM.put(OUTSIDE_DS_ADDR, outside_ds);
  } else if(!strcmp(param, "factory") && !strcmp(value, "defaults")) {
    processSet("kp 20");
    processSet("ki 0.02");
    processSet("kd 0");
    processSet("omin 26");
    processSet("omax 255");
    processSet("hc_minus5 4200");
    processSet("hc_0 3700");
    processSet("hc_5 3200");
    processSet("hc_10 2700");
  } else {
    Serial.println("Parameter not found.");
  }
}

void processCommand(char *buf) {
  Serial.println("Received command:");
  Serial.println(buf);
  if(!strncmp(buf, "get ", 4)) {  // get command
    processGet(buf + 4);
  } else if(!strncmp(buf, "set ", 4)) {  // set command
    processSet(buf + 4);
  } else if(!strncmp(buf, "help", 4)) {  // help command
    printHelp();
  } else {
    Serial.println("Command not found. Use help command.");
  }
  Serial.println("==================");
} */

void setup() {
  Serial.begin(9600);
  Serial.println("= STARTING =");

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(HEAT_SWITCH_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(HEAT_SWITCH_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onPulse, FALLING);

  pinMode(HEAT_DS_PROG_BUTTON, INPUT_PULLUP);
  pinMode(RETURN_DS_PROG_BUTTON, INPUT_PULLUP);
  pinMode(OUTSIDE_DS_PROG_BUTTON, INPUT_PULLUP);

  digitalWrite(BUZZER_PIN, HIGH);
  pinMode(BUZZER_PIN, OUTPUT);

  //heat_ds_prog_button.setup(HEAT_DS_PROG_BUTTON);
  //return_ds_prog_button.setup(RETURN_DS_PROG_BUTTON);
  //outside_ds_prog_button.setup(OUTSIDE_DS_PROG_BUTTON);

  //heat_ds_prog_button.setPressMs(3000);
  //return_ds_prog_button.setPressMs(3000);
  //outside_ds_prog_button.setPressMs(3000);

  //heat_ds_prog_button.attachLongPressStart(programHeatDS);
  //return_ds_prog_button.attachLongPressStart(programReturnDS);
  //outside_ds_prog_button.attachLongPressStart(programOutsideDS);

  // ====== Heating curve ======
  EEPROM.get(HC_MINUS5_ADDR, heat_curve.hc_minus5);
  EEPROM.get(HC_0_ADDR, heat_curve.hc_0);
  EEPROM.get(HC_5_ADDR, heat_curve.hc_5);
  EEPROM.get(HC_10_ADDR, heat_curve.hc_10);
  //printHCParams();

  // =========== PID ===========
  EEPROM.get(KP_ADDR, pid_params.kp);
  EEPROM.get(KI_ADDR, pid_params.ki);
  EEPROM.get(KD_ADDR, pid_params.kd);
  EEPROM.get(OMIN_ADDR, pid_params.omin);
  EEPROM.get(OMAX_ADDR, pid_params.omax);
  //printPIDParams();
  pid.SetTunings(pid_params.kp, pid_params.ki, pid_params.kd);
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(60);
  pid.SetOutputLimits(pid_params.omin, pid_params.omax);

  // ========== DS =============
  ds.begin();
  //printAvailableDS();
  EEPROM.get(HEAT_DS_ADDR, heat_ds);
  EEPROM.get(RETURN_DS_ADDR, return_ds);
  EEPROM.get(OUTSIDE_DS_ADDR, outside_ds);
  /*
  Serial.print("Heat DS: ");
  printDSAddr(heat_ds);
  Serial.print("Return DS ");
  printDSAddr(return_ds);
  Serial.print("Outisde DS: ");
  printDSAddr(outside_ds);
*/
  // setup DS precision
  if(ds.getResolution(heat_ds) != ds_precision) ds.setResolution(ds_precision);
  if(ds.getResolution(return_ds) != ds_precision) ds.setResolution(ds_precision);
  if(ds.getResolution(outside_ds) != ds_precision) ds.setResolution(ds_precision);

  // --- pzem ---
  //char pzem_addr[4];
  //Serial.print("PZEM Address: ");
  //itoa(pzem.readAddress(), pzem_addr, 16);
  //Serial.println(pzem_addr);

  // --- Wire ---
  Wire.begin(WIRE_SLAVE_ADDR);
  Wire.onRequest(onWireResponseRequest);
  Wire.onReceive(onWireReceive);

  // === Fill in all probes ===
  for(uint8_t i = 0; i < n_probes; i++) {
    getSensorProbeValues(i);
  }

  // --- timers ---
  //timer.every(4000, on4Sec);
  //timer.every(60000, on60Sec);
  //timer.every(5000, flowAlarmCheck);
  timer_start_millis = millis()/1000;

  Serial.println("= STARTED =");
}

void loop() {
  //timer.update();
  pid.Compute();
  //heat_ds_prog_button.tick();
  //return_ds_prog_button.tick();
  //outside_ds_prog_button.tick();

  // ======== UART commands support ==========
  /*  char received = NULL;
  static char serial_buf[20];
  static uint8_t buf_index = 0;
  if(Serial.available()) {
    received = Serial.read();
    if(received == '\n' || received == '\r') {  // command fully received
      processCommand(serial_buf);
      serial_buf[0] = '\0';  // clear buffer
      buf_index = 0;
    } else {  // command not fully received, attaching the last character
      serial_buf[buf_index] = received;
      buf_index++;
      serial_buf[buf_index] = '\0';
    }
  } */

  if(alarm_flag) {
    digitalWrite(HEAT_SWITCH_PIN, HIGH);
    //digitalWrite(LED_BUILTIN, HIGH);
  }

  if((uint8_t)((uint8_t)(millis()/1000) - timer_start_millis) >= 4) {
    timer_start_millis = millis()/1000;
    on4Sec();
    flowAlarmCheck();
    loop_counter--;
    if(!loop_counter) {
      loop_counter = 15;
      on60Sec();
    }
  }

  // Program Heat DS button support
  heat_ds_prog_button_current_state = digitalRead(HEAT_DS_PROG_BUTTON);
  if(heat_ds_prog_button_last_state == HIGH && heat_ds_prog_button_current_state == LOW) {
    button_pressed_time = millis() / 1000;
  } else if(heat_ds_prog_button_last_state == LOW && heat_ds_prog_button_current_state == HIGH) {
    button_released_time = millis() / 1000;
    if(button_released_time >= button_pressed_time + 3) {
      programHeatDS();
    }
  }
  heat_ds_prog_button_last_state = heat_ds_prog_button_current_state;

  // Program Return DS button support
  return_ds_prog_button_current_state = digitalRead(RETURN_DS_PROG_BUTTON);
  if(return_ds_prog_button_last_state == HIGH && return_ds_prog_button_current_state == LOW) {
    button_pressed_time = millis() / 1000;
  } else if(return_ds_prog_button_last_state == LOW && return_ds_prog_button_current_state == HIGH) {
    button_released_time = millis() / 1000;
    if(button_released_time >= button_pressed_time + 3) {
      programReturnDS();
    }
  }
  return_ds_prog_button_last_state = return_ds_prog_button_current_state;

  // Program Outside DS button support
  outside_ds_prog_button_current_state = digitalRead(OUTSIDE_DS_PROG_BUTTON);
  if(outside_ds_prog_button_last_state == HIGH && outside_ds_prog_button_current_state == LOW) {
    button_pressed_time = millis() / 1000;
  } else if(outside_ds_prog_button_last_state == LOW && outside_ds_prog_button_current_state == HIGH) {
    button_released_time = millis() / 1000;
    if(button_released_time >= button_pressed_time + 3) {
      programOutsideDS();
    }
  }
  outside_ds_prog_button_last_state = outside_ds_prog_button_current_state;

  //buzzer support
  if(beepOK) {
    if(!beep_last_state) beep_start_millis = millis() / 100;
    digitalWrite(BUZZER_PIN, LOW);
    //Serial.println("beep");

    if((uint8_t)((uint8_t)(millis()/100) - beep_start_millis) >= 2 ) {
      beepOK = false;
      digitalWrite(BUZZER_PIN, HIGH);
      //Serial.println("stop");
    }
  }
  if(beepNOK) {
    if(!beep_last_state) beep_start_millis = millis() / 100;
    digitalWrite(BUZZER_PIN, LOW);
    //Serial.println("beep");
    if((uint8_t)((uint8_t)(millis()/100) - beep_start_millis) >= 10 ) {
      beepNOK = false;
      digitalWrite(BUZZER_PIN, HIGH);
      //Serial.println("stop");
    }
  }
  beep_last_state = beepOK || beepNOK;
}

void onPulse() {
  pulse_counter++;
  //Serial.println("pulse");
}

void getSensorProbeValues(uint8_t i) {
  PZEM004Tv30 pzem(SOFTSERIAL_RX_PIN, SOFTSERIAL_TX_PIN);
  energy_probes[i].voltage = pzem.voltage();  // x 0.1 V
  energy_probes[i].current = pzem.current();  // x 0.001 A
  energy_probes[i].power = pzem.power();      // x 0.1 W
  energy_probes[i].pf = pzem.pf();            // x 0.01
  ds.requestTemperatures();
  temp_probes[i].heat = (ds.getTemp(heat_ds) * 78125 + 50000) / 100000;        // x 0.01 °C
  temp_probes[i].ret = (ds.getTemp(return_ds) * 78125 + 50000) / 100000;       // x 0.01 °C
  temp_probes[i].outside = (ds.getTemp(outside_ds) * 78125 + 50000) / 100000;  // x 0.01 °C
}

void on4Sec() {
  static uint8_t i = 0;
  getSensorProbeValues(i);

  /*
  char heat[10], ret[10], outside[10], buf[50];
  itoa(temp_probes[i].heat, heat, 10);
  itoa(temp_probes[i].ret, ret, 10);
  itoa(temp_probes[i].outside, outside, 10);
  snprintf(buf, sizeof(buf), "%s %s %s\n----------------------", heat, ret, outside);

  Serial.println(buf);
*/

  i++;
  if(i >= n_probes) i = 0;
}

void on60Sec() {
  Serial.println("====== ON 60 sec ======");
  avgCalcAll();
  pid_data.sv = getHCValue(heat_curve, temp_avg.outside);
  analogWrite(PWM_PIN, pid_data.output);
  showValues();
}

void flowAlarmCheck() {
  //char str_buf[6];
  //itoa(pulse_counter, str_buf, 10);
  //Serial.println(str_buf);
  if(pulse_counter < CHECK_PULSE_THRESHOLD) {
    alarm_flag = true;
    Serial.print(pulse_counter);
    Serial.print(" is below ");
    Serial.println(CHECK_PULSE_THRESHOLD);
    Serial.println("rise ALARM!");
  }
  pulse_counter = 0;
}

void wireRespondDomoticzValues() {
  Serial.println("Respond with domoticz values");
  avgCalcAll();

  CRC8 crc;
  crc.add(WIRE_DATA_FRAME);
  crc.add((uint8_t *) &temp_avg, sizeof(temp_avg));
  crc.add((uint8_t *) &energy_avg, sizeof(energy_avg));
  crc.add((uint8_t *) &pid_data, sizeof(pid_data));
  uint8_t wire_crc = crc.calc();

  Wire.write(WIRE_DATA_FRAME);
  Wire.write((byte *)&temp_avg, sizeof(temp_avg));
  Wire.write((byte *)&energy_avg, sizeof(energy_avg));
  Wire.write((byte *)&pid_data, sizeof(pid_data));
  Wire.write(&wire_crc, sizeof(wire_crc));

  Serial.println("=======================");
  Serial.print("CRC: ");
  Serial.println(wire_crc);
}

void wireRespondPidParams() {
  Serial.println("respond pid params");
  Wire.write((byte *)&pid_params, sizeof(pid_params));
}

void wireRespondHeatCurveParams() {
  Serial.println("respond heat curve");
  Wire.write((byte *)&heat_curve, sizeof(heat_curve));
}

void wireRespondTempValues() {
  Serial.println("respond temp values");
  avgCalcAll();
  Wire.write((byte *)&temp_avg, sizeof(temp_avg));
}

void wireRespondEnergyValues() {
  Serial.println("respond energy values");
  avgCalcAll();
  Wire.write((byte *)&energy_avg, sizeof(energy_avg));
}

void onWireResponseRequest() {
  Serial.println("Wire response request");

  switch (wire_cmd) {
    case 0x01:
      wireRespondDomoticzValues();
      break;
    case 0x02:
      wireRespondPidParams();
      break;
    case 0x03:
      wireRespondHeatCurveParams();
      break;
    case 0x04:
      wireRespondTempValues();
      break;
    case 0x05:
      wireRespondEnergyValues();
      break;
    default:
      Serial.println("responde default (uknonwn command)");
      Wire.write(0x00);
  }
  wire_cmd = 0x00;
}

void onWireReceive(int bytes) {
  Serial.print("WIRE RECEIVE ");
  Serial.print(bytes);
  Serial.println(" bytes.");
  if(Wire.available()) wire_cmd = Wire.read();
  Serial.print("Received cmd code: ");
  Serial.println(wire_cmd);
  switch (wire_cmd) {
    case 0x01:
      Serial.println("get domoticz values cmd");
      break;
    case 0x02:
      Serial.println("get pid cmd");
      break;
    case 0x03:
      Serial.println("get hc cmd");
      break;
    case 0x04:
      Serial.println("get temp cmd");
      break;
    case 0x05:
      Serial.println("get energy cmd");
      break;

    case 0x12:
      Serial.println("set pid params");
      if(Wire.available()) Wire.readBytes((byte *)&pid_params, sizeof(pid_params));
      pid.SetTunings(pid_params.kp, pid_params.ki, pid_params.kd);
      pid.SetOutputLimits(pid_params.omin, pid_params.omax);
      printPIDParams();
      break;
    case 0x13:
      Serial.println("set hc params");
      if(Wire.available()) Wire.readBytes((byte *)&heat_curve, sizeof(heat_curve));
      printHCParams();
      break;
    case 0x20:
      Serial.println("save data to EEPROM");
      saveDataToEEPROM();
  }
}

void saveDataToEEPROM() {
  EEPROM.put(KP_ADDR, pid_params.kp);
  EEPROM.put(KI_ADDR, pid_params.ki);
  EEPROM.put(KD_ADDR, pid_params.kd);
  EEPROM.put(OMIN_ADDR, pid_params.omin);
  EEPROM.put(OMAX_ADDR, pid_params.omax);
  EEPROM.put(HC_MINUS5_ADDR, heat_curve.hc_minus5);
  EEPROM.put(HC_0_ADDR, heat_curve.hc_0);
  EEPROM.put(HC_5_ADDR, heat_curve.hc_5);
  EEPROM.put(HC_10_ADDR, heat_curve.hc_10);
  /*PidParams pi;
  HC hc;
  EEPROM.get(KP_ADDR, pi.kp);
  EEPROM.get(KI_ADDR, pi.ki);
  EEPROM.get(KD_ADDR, pi.kd);
  EEPROM.get(OMIN_ADDR, pi.omin);
  EEPROM.get(OMAX_ADDR, pi.omax);
  EEPROM.get(HC_MINUS5_ADDR, hc.hc_minus5);
  EEPROM.get(HC_0_ADDR, hc.hc_0);
  EEPROM.get(HC_5_ADDR, hc.hc_5);
  EEPROM.get(HC_10_ADDR, hc.hc_10);
  Serial.println(pi.kp);
  Serial.println(pi.ki);
  Serial.println(pi.kd);
  Serial.println(pi.omin);
  Serial.println(pi.omax);
  Serial.println(hc.hc_minus5);
  Serial.println(hc.hc_0);
  Serial.println(hc.hc_5);
  Serial.println(hc.hc_10);*/
}

bool getDSAddress(uint8_t *ds_addr) {
  Serial.println("Get DS address");
  uint8_t addr[8];
  OneWire prog_ow(PROG_DS_PIN);
  prog_ow.search(addr);
  if(OneWire::crc8(addr, 7) == addr[7]) {
    memcpy(ds_addr, addr, 8);
    return true;
  }
  return false;
}

void programHeatDS() {
  //Serial.println("Programing Heat DS");
  if(getDSAddress(heat_ds)) {
    EEPROM.put(HEAT_DS_ADDR, heat_ds);
    Serial.println("OK");
    beepOK = true;
  } else {
    Serial.println("CRC error");
    beepNOK = true;
  }
}

void programReturnDS() {
  //Serial.println("Programing Return DS");
  if(getDSAddress(return_ds)) {
    EEPROM.put(RETURN_DS_ADDR, return_ds);
    Serial.println("OK");
    beepOK = true;
  } else {
    Serial.println("CRC error");
    beepNOK = true;
  }
}

void programOutsideDS() {
  //Serial.println("Programing Outside DS");
  if(getDSAddress(outside_ds)) {
    EEPROM.put(OUTSIDE_DS_ADDR, outside_ds);
    Serial.println("OK");
    beepOK = true;
  } else {
    Serial.println("CRC error");
    beepNOK = true;
  }
}
