/*
  created by : - Raj Mehta
  sudo killall -9 python; ./teensy_reboot -s; ./teensy_loader_cli --mcu=TEENSY41 -wv robot4-NUC8i3BEH_kake_di_hatti_Tennsy.ino.TEENSY41.hex
  chane log:
  30-1-23:
    1) add teensy reset in BMI null error.
*/

/* Ethernet */
#include <SPI.h>
#include <NativeEthernet.h>
/* ADC */
#include <Mcp320x.h>
/* BMI */
#include "BMI088.h"
/* Thread */
//#include "TeensyThreads.h"
/* WS2812b LED */
//#include <FastLED.h>

/* IR emitter */
#define IR1_pin 23
#define Laser_pin 22

/* temp */
//#define NUC_SSR 32
#define FAN1 36
#define FAN2 37

/* smartSwitch() */
#define main_relay_pin 35
#define main_SW_FB     34
#define Estop_FB       33
#define Main_SW_FB digitalRead(main_SW_FB)
#define estop_FB digitalRead(Estop_FB)

String BTStr;
char *BT_data[2];
char *ptr2 = NULL;
char temp_data_array2[10];

/* motion mm/s */
#define deg_to_pos 8500
#define deg_to_pos_2 4250
#define deg_to_pos_4 2125
//#define ang_to_pos 0.08
#define ang_to_pos 0.245
#define dis_to_pos 41.2 // 10.3
/* ADC */
#define SPI_CS      10       // SPI slave select
#define ADC_VREF    5000     // 3.3V Vref
#define ADC_CLK     1600000  // SPI clock 1.6MHz
MCP3208 adc(ADC_VREF, SPI_CS);

/* BMI088 */
#define TO_DEG(x) (x * 57.2957795131)

Bmi088Gyro gyro(Wire, 0x69);
double gxr, gyr, gzr;
uint32_t t1, t2, _yaw2;
double dt;
int temp1;
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float yaw2 , _yaw;
char x;
int32_t adddata, _BMI_YAW, _BMI_Add_YAW;

//int8_t error_Code = 999;

/* ZL Motor */
int32_t enc_count_left, enc_count_right;
uint16_t temperature;
int err;

/* smartSwitch() */
bool flag_shutdown, Estop_FB_flag = false;
unsigned long shut_down_count;

/* ADC */
uint16_t raw_CS_24, raw_CS_24_vref, raw_CS_19, raw_CS_5, raw_VD_24, raw_VD_24_af, raw_VD_NUC, raw_VD_19;
uint16_t vol_CS_24, vol_CS_24_vref, vol_CS_19, vol_CS_5, vol_VD_24, vol_VD_24_af, vol_VD_NUC, vol_VD_19;
float bat_current, bat_current_filtered, bat_current_sum;
float current_5, current_5_filtered, current_5_sum;
float current_19, current_19_filtered, current_19_sum;
float voltage_24, voltage_24_filtered, voltage_24_sum;
float voltage_24_af, voltage_24_af_filtered, voltage_24_af_sum;
float voltage_NUC, voltage_NUC_filtered, voltage_NUC_sum;
float voltage_19, voltage_19_filtered, voltage_19_sum;

//********************************************************NUC data ************************************************************
char reqStr;
String aprilTagStr, iputStr;
char *NUC_data[7];
char *ptr = NULL;
byte NUC_to_BMI_yaw;
byte NUC_tag_state, tagid_to_ignore, NUC_state, prev_NUC_state, prev_NUC_tag_state;
uint16_t NUC_yaw, NUC_yaw_setpoint, NUC_distance, pre_NUC_distance;
float NUC_x, NUC_x_setpoint, NUC_y, NUC_y_setpoint, NUC_yaw_t;
byte NUC_send[12];
String str, pre_str;
char temp_data_array[100];

bool flag_position_control = true, flag_velocity_control = false;
bool flag_stop_velocity = false, flag_stop_position = false;
bool flag_start_velocity = false, flag_start_position = false;
bool flag_velocity_cal = false;
bool flag_motor_free = true;
bool flag_once = false, flag_once2 = true;
bool flag_motion = false;
bool flag_error = false;
bool prev_NUC_state4 = false;
bool init_sync_pos_flag = false;
uint8_t flag_dcc_time = 0;

int encoder_count, encoder_count0;
int ErrorCode = 999, timeOut = 0, timeOut1 = 0;
int mode8 = 0, nuc_mode8 = 0;
int dis_to_Travel, travel_read;
int16_t motor_right_speed, motor_left_speed, diff_in_speed;
int16_t motionAcc = 0, motiondeacc = 0;
int16_t speedl_data = 0, speedr_data = 0;
unsigned long position_cal_count, angle_set_time_count;
unsigned long init_syncpos_count, init_sync_pos_flag_count;
unsigned long deacc_count, error_time_count;
unsigned long estop_FB_counter = millis();

unsigned long dataReceive_counter = millis();
unsigned long motion_counter150 = millis();
unsigned long motion_counter60 = millis();
unsigned long motion_counter20 = millis();
unsigned long motion_counter10 = millis();
unsigned long motion_counter4 = millis();
unsigned long motion_counter5 = millis();
unsigned long motion_counter6 = millis();
unsigned long motion_counter7 = millis();
unsigned long motion_counter0 = millis();
unsigned long motionEstop_counter = millis();
int estop_FB_to_NUC = 0, _maxSpeed = 140;

uint32_t combined_motor_speed;
int16_t left_motor_speed, right_motor_speed;

int32_t temp_ang_dif, change_in_angle;
int32_t angle_diff;
float angle_pos, angle_pos_y;
int32_t cur_pos;

float error_angle, pre_error_angle;
float kp = 0.17, ki = 0, kd = 0.16;// kp=0.166, kd=0.1, 0.15
float P, I, D;
/* Ethernet */
// gateway and subnet are optional:
byte mac[6];

// ip as per the ethernet 192.168.10.2-255 random
IPAddress ip(192, 168, 2, 3);

EthernetServer server(80);
boolean alreadyConnected = false; // whether or not the client was connected previously
char c;
int status;

void teensyMAC(uint8_t *mac) {
  for (uint8_t by = 0; by < 2; by++) mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
  for (uint8_t by = 0; by < 4; by++) mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
  //  // Serial1.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void setup()
{
  /* Ethernet */
  teensyMAC(mac);
  Ethernet.begin(mac, ip);
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    // Serial1.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      ErrorCode = 201;
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    ErrorCode = 202;
    // Serial1.println("Ethernet cable is not connected.");
  }
  // start listening for clients
  server.begin();

  /* ADC */
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings);

  /* BMI088 */
  status = gyro.begin();
  while (status < 0) {
    //    // Serial1.println("Gyro Initialization Error");
    //    // Serial1.println(status);
    ErrorCode = 208;
    status = gyro.begin();
    delay(1000);
  }

  //  status = gyro.setOdr(Bmi088Gyro::ODR_200HZ_BW_64HZ);
  //  ODR_200HZ_BW_23HZ
  status = gyro.setOdr(Bmi088Gyro::ODR_200HZ_BW_23HZ);

  /* Bluetooth */
  Serial7.begin(9600);

  /* ZL Motor */
  init_can();
  clear_error();
  clear_encoder();
  //  en_motor();
  stop_motor();
  /* IR emitter */
  pinMode(IR1_pin, OUTPUT);  digitalWrite(IR1_pin, 1);
  pinMode(Laser_pin, OUTPUT);  digitalWrite(Laser_pin, 0);
  /* NUC */
  //  pinMode(NUC_SSR, OUTPUT);  digitalWrite(NUC_SSR, 1);
  /* smartSwitch() */
  pinMode(main_relay_pin, OUTPUT);  pinMode(main_SW_FB, INPUT_PULLUP);  pinMode(Estop_FB, INPUT_PULLUP);
  digitalWrite(main_relay_pin, 1);
  ErrorCode = 999;
}


void loop()
{
  EthernetClient client = server.available();
  if (client) {
    en_motor();
    while (client.connected()) {
      while (client.available() > 0) {
        str = client.readStringUntil('\n');
        /* NUC command */
        NUC_command();
        client.print('<');
        client.print(',');
        client.print(Main_SW_FB);
        client.print(',');
        client.print(_BMI_YAW);  // 5
        client.print(',');    // 6
        client.print(bat_current_filtered);
        client.print(',');
        client.print(voltage_24_filtered * 6);
        client.print(',');
        client.print(encoder_count0);
        client.print(',');
        client.print(ErrorCode);
        client.print(',');
        client.print(estop_FB_to_NUC);
        client.print(',');
        client.print('>');
        /* solve error */
        ErrorCode = 999;
        /* Voltage and Current */
        Sys_Pow_Monitoring();
        /* smartSwitch() */
        smartSwitch();
        /* Motion Algo */
        Motion_algo();
      }
    }
  }
  prev_NUC_state = 50;
  NUC_state = 25;
  /* stop */
  stop_motor();

  /* Voltage and Current */
  Sys_Pow_Monitoring();

  /* smartSwitch() */
  smartSwitch();

  /* reset teensy and relay*/
  if (Serial7.available()) {
    BTStr = Serial7.readStringUntil('\n');
    BTStr.toCharArray(temp_data_array2, BTStr.length() + 1);
    byte index = 0;
    ptr2 = strtok(temp_data_array2, ",");
    while (ptr2 != NULL) {
      BT_data[index] = ptr2;
      index++;
      ptr2 = strtok(NULL, ",");
    }
    if (atoi(BT_data[0]) == 101) {
      Serial7.println("Teensy Reset Start");
      delay(2);
      SCB_AIRCR = 0x05FA0004;
    }
    /* data print on bt 22/11/22 6:09pm */
    else if (atoi(BT_data[0]) == 1) {
      Serial7.print("current=");
      Serial7.print(bat_current_filtered);
      Serial7.print(", vol24=");
      Serial7.println(voltage_24_filtered * 6);
    }
    /* reset relay */
    else if (atoi(BT_data[0]) == 2) {
      Serial7.print("main_relay_pin=");
      Serial7.println(main_relay_pin);
      digitalWrite(main_relay_pin, !digitalRead(main_relay_pin));
      Serial7.print("main_relay_pin=");
      Serial7.println(main_relay_pin);
    }
  }
  //  SCB_AIRCR = 0x05FA0004;
}
