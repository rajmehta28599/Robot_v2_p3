/*
  1) for  uploading code from NUC to ESP.
    arduino-cli compile --fqbn esp32:esp32:esp32 ESP32_blink/ESP32_blink.ino
    arduino-cli upload -p /dev/ESP32 --fqbn esp32:esp32:esp32 ESP32_blink/ESP32_blink.ino

  2) for esp update:
    arduino-cli core update-index --config-file arduino-cli.yaml
    arduino-cli core install esp32:esp32

  3) NOTE: libraries for RF communication is RadioHead-master

  4) Application: for ligth indication patterns
    Pattern number      pattern data
    0 LED off
    1 Serve tray        1,2,3 for 3 trays
    2 Charging          voltage
    3 Warm whight
    4 left side light
    5 right side light
    6 both side light
    7 motion indication
    10 Error
    101 teensy reboot
*/

#include <FastLED.h>
#include <RH_ASK.h>

#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
#define NUM_LEDS 97
// #define tray1_leds 100
#define DATA_PIN_t2 4
#define DATA_PIN_t3 15
#define DATA_PIN_t1 2
#define CLOCK_PIN 13
CRGB leds_t1[NUM_LEDS];
CRGB leds_t2[NUM_LEDS];
CRGB leds_t3[NUM_LEDS];
int pattern_data, led_pattern = 3;

uint16_t led_num_t1, active_led_num_t1;
uint16_t led_num_t2, active_led_num_t2;
uint16_t led_num_t3, active_led_num_t3;

bool flag_SL = false;

uint16_t led_value;
#define colour_orange 0x25FB00;
#define colour_orange_dim 0x137E00;
#define warm_whight 0x96FF1A;
#define warm_whight_dim 0x325509;
bool flag_pat_2 = true;
uint8_t SL_led_num, SL_led_num_2, SL_led_num_3;  // SD = Side Light
uint8_t pattern_var1, pattern_var2, pattern_var3, pattern_var4, pattern_var5;
uint8_t pattern_var0, pattern_var6, pattern_var7, pattern_var8, pattern_var9;
unsigned long pat7_count;

#define left_side_light_led_start 75
#define left_side_light_led_end 36
#define right_side_light_led_start 86
#define right_side_light_led_end 28

#define colour_orange 0x25FB00;
#define warm_whight 0x96FF1A;
#define warm_whight_dim 0x325509;

#define teensy_reboot 18

String NUCStr;
char *NUC_data[2];
char *ptr = NULL;
char temp_data_array[10];

/* initialise the driver at 2000 bps, recieve on D27, transmit on 12, PTT on 0*/
RH_ASK driver(2000, 27, 12, 0);
#define CS 26
uint8_t buf[2], device_id;
uint8_t buflen = sizeof(buf);
unsigned long count_RF_call;

void setup() {
  Serial.begin(115200);
  pinMode(teensy_reboot, INPUT_PULLUP);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  if (!driver.init())
    Serial.println("init failed");

  // digitalWrite(teensy_reboot,1);
  //  pinMode(2,OUTPUT);
  FastLED.addLeds<WS2812B, DATA_PIN_t1, RGB>(leds_t1, NUM_LEDS);  // GRB ordering is typical
  FastLED.addLeds<WS2812B, DATA_PIN_t2, RGB>(leds_t2, NUM_LEDS);
  FastLED.addLeds<WS2812B, DATA_PIN_t3, RGB>(leds_t3, NUM_LEDS);
  SerialBT.enableSSP();
  Serial.println(SerialBT.setPin("1234"));
  SerialBT.begin("Robot_ESP32_2");
}

void loop() {
  if (Serial.available()) {
    NUCStr = Serial.readStringUntil('\n');
    NUCStr.toCharArray(temp_data_array, NUCStr.length() + 1);
    byte index = 0;
    ptr = strtok(temp_data_array, ",");
    while (ptr != NULL) {
      NUC_data[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
    }
    led_pattern = atoi(NUC_data[0]);
    // SerialBT.println(led_pattern);
    if (led_pattern == 1) {
      pattern_data = atof(NUC_data[1]);
      pattern_var0 = 0;
      pattern_var1 = 1;
      pattern_var2 = 2;
      pattern_var3 = 3;
      pattern_var4 = 4;
      pattern_var5 = 5;
      pattern_var6 = 6;
      pattern_var7 = 7;
      pattern_var8 = 8;
      pattern_var9 = 9;
      SerialBT.println(pattern_data);
    } else if (led_pattern == 2) {
      pattern_data = atof(NUC_data[1]) * 100;
      active_led_num_t1 = active_led_num_t2 = active_led_num_t3 = (float(pattern_data - 1800) / 720) * NUM_LEDS;
      SerialBT.println(pattern_data);
    } else if (led_pattern == 4) {
      SL_led_num_3 = left_side_light_led_start;
    } else if (led_pattern == 5) {
      SL_led_num = right_side_light_led_start;
      SL_led_num_2 = 0;
      flag_SL = false;
    } else if (led_pattern == 6) {
      SL_led_num_3 = left_side_light_led_start;
      SL_led_num = right_side_light_led_start;
      SL_led_num_2 = 0;
      flag_SL = false;
    } else if (led_pattern == 7) {
      pattern_var1 = 0;
      pattern_var2 = 1;
      pattern_var3 = 2;
      pattern_var4 = 3;
      pattern_var5 = 4;
      pat7_count = 0;
    } else if (led_pattern == 101) {
      SerialBT.println("rebooting teensy");
      pinMode(teensy_reboot, OUTPUT);
      digitalWrite(teensy_reboot, 0);
      delay(50);
      pinMode(teensy_reboot, INPUT_PULLUP);
    }
  }
  if (SerialBT.available()) {
    NUCStr = SerialBT.readStringUntil('\n');
    NUCStr.toCharArray(temp_data_array, NUCStr.length() + 1);
    byte index = 0;
    ptr = strtok(temp_data_array, ",");
    while (ptr != NULL) {
      NUC_data[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
    }
    led_pattern = atoi(NUC_data[0]);
    // SerialBT.println(led_pattern);
    if (led_pattern == 1) {
      pattern_data = atof(NUC_data[1]);
      pattern_var0 = 0;
      pattern_var1 = 1;
      pattern_var2 = 2;
      pattern_var3 = 3;
      pattern_var4 = 4;
      pattern_var5 = 5;
      pattern_var6 = 6;
      pattern_var7 = 7;
      pattern_var8 = 8;
      pattern_var9 = 9;
      // SerialBT.println(pattern_data);
    } else if (led_pattern == 2) {
      pattern_data = atof(NUC_data[1]) * 100;
      active_led_num_t1 = active_led_num_t2 = active_led_num_t3 = (float(pattern_data - 1900) / 720) * NUM_LEDS;
      // SerialBT.println(pattern_data);
    } else if (led_pattern == 4) {
      SL_led_num_3 = left_side_light_led_start;
    } else if (led_pattern == 5) {
      SL_led_num = right_side_light_led_start;
      SL_led_num_2 = 0;
      flag_SL = false;
    } else if (led_pattern == 6) {
      SL_led_num_3 = left_side_light_led_start;
      SL_led_num = right_side_light_led_start;
      SL_led_num_2 = 0;
      flag_SL = false;
    } else if (led_pattern == 7) {
      pattern_var1 = 0;
      pattern_var2 = 1;
      pattern_var3 = 2;
      pattern_var4 = 3;
      pattern_var5 = 4;
      pat7_count = 0;
    } else if (led_pattern == 101) {
      SerialBT.println("rebooting teensy");
      pinMode(teensy_reboot, OUTPUT);
      digitalWrite(teensy_reboot, 0);
      delay(50);
      pinMode(teensy_reboot, INPUT_PULLUP);
    }
  }
  switch (led_pattern) {
    case 0:  // turn off all LEDs
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = 0x000000;
        leds_t2[i] = 0x000000;
        leds_t3[i] = 0x000000;
      }
      FastLED.show();
      break;
    case 1:
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = 0x000000;
        leds_t2[i] = 0x000000;
        leds_t3[i] = 0x000000;
      }
      switch (pattern_data) {
        case 1:                              // tray 1
          leds_t1[pattern_var9] = 0xFF00FF;  // 255
          leds_t1[pattern_var8] = 0xC800C8;  // 200
          leds_t1[pattern_var7] = 0xAF00AF;  // 175
          leds_t1[pattern_var6] = 0x960096;  // 150
          leds_t1[pattern_var5] = 0x7D007D;  // 125
          leds_t1[pattern_var4] = 0x640064;  // 100
          leds_t1[pattern_var3] = 0x4B004B;  // 75
          leds_t1[pattern_var2] = 0x320032;  // 50
          leds_t1[pattern_var1] = 0x230023;  // 35
          leds_t1[pattern_var0] = 0x140014;  // 20
          pattern_var0++;
          pattern_var1++;
          pattern_var2++;
          pattern_var3++;
          pattern_var4++;
          pattern_var5++;
          pattern_var6++;
          pattern_var7++;
          pattern_var8++;
          pattern_var9++;
          if (pattern_var0 == NUM_LEDS) pattern_var0 = 0;
          if (pattern_var1 == NUM_LEDS) pattern_var1 = 0;
          if (pattern_var2 == NUM_LEDS) pattern_var2 = 0;
          if (pattern_var3 == NUM_LEDS) pattern_var3 = 0;
          if (pattern_var4 == NUM_LEDS) pattern_var4 = 0;
          if (pattern_var5 == NUM_LEDS) pattern_var5 = 0;
          if (pattern_var6 == NUM_LEDS) pattern_var6 = 0;
          if (pattern_var7 == NUM_LEDS) pattern_var7 = 0;
          if (pattern_var8 == NUM_LEDS) pattern_var8 = 0;
          if (pattern_var9 == NUM_LEDS) pattern_var9 = 0;
          break;
        case 2:                              // tray 2
          leds_t2[pattern_var8] = 0xC800C8;  // 200
          leds_t2[pattern_var7] = 0xAF00AF;  // 175
          leds_t2[pattern_var6] = 0x960096;  // 150
          leds_t2[pattern_var9] = 0xFF00FF;  // 255
          leds_t2[pattern_var5] = 0x7D007D;  // 125
          leds_t2[pattern_var4] = 0x640064;  // 100
          leds_t2[pattern_var3] = 0x4B004B;  // 75
          leds_t2[pattern_var2] = 0x320032;  // 50
          leds_t2[pattern_var1] = 0x230023;  // 35
          leds_t2[pattern_var0] = 0x140014;  // 20
          pattern_var0++;
          pattern_var1++;
          pattern_var2++;
          pattern_var3++;
          pattern_var4++;
          pattern_var5++;
          pattern_var6++;
          pattern_var7++;
          pattern_var8++;
          pattern_var9++;
          if (pattern_var0 == NUM_LEDS) pattern_var0 = 0;
          if (pattern_var1 == NUM_LEDS) pattern_var1 = 0;
          if (pattern_var2 == NUM_LEDS) pattern_var2 = 0;
          if (pattern_var3 == NUM_LEDS) pattern_var3 = 0;
          if (pattern_var4 == NUM_LEDS) pattern_var4 = 0;
          if (pattern_var5 == NUM_LEDS) pattern_var5 = 0;
          if (pattern_var6 == NUM_LEDS) pattern_var6 = 0;
          if (pattern_var7 == NUM_LEDS) pattern_var7 = 0;
          if (pattern_var8 == NUM_LEDS) pattern_var8 = 0;
          if (pattern_var9 == NUM_LEDS) pattern_var9 = 0;
          break;
        case 3:                              // tray 3
          leds_t3[pattern_var9] = 0xFF00FF;  // 255
          leds_t3[pattern_var8] = 0xC800C8;  // 200
          leds_t3[pattern_var7] = 0xAF00AF;  // 175
          leds_t3[pattern_var6] = 0x960096;  // 150
          leds_t3[pattern_var5] = 0x7D007D;  // 125
          leds_t3[pattern_var4] = 0x640064;  // 100
          leds_t3[pattern_var3] = 0x4B004B;  // 75
          leds_t3[pattern_var2] = 0x320032;  // 50
          leds_t3[pattern_var1] = 0x230023;  // 35
          leds_t3[pattern_var0] = 0x140014;  // 20
          pattern_var0++;
          pattern_var1++;
          pattern_var2++;
          pattern_var3++;
          pattern_var4++;
          pattern_var5++;
          pattern_var6++;
          pattern_var7++;
          pattern_var8++;
          pattern_var9++;
          if (pattern_var0 == NUM_LEDS) pattern_var0 = 0;
          if (pattern_var1 == NUM_LEDS) pattern_var1 = 0;
          if (pattern_var2 == NUM_LEDS) pattern_var2 = 0;
          if (pattern_var3 == NUM_LEDS) pattern_var3 = 0;
          if (pattern_var4 == NUM_LEDS) pattern_var4 = 0;
          if (pattern_var5 == NUM_LEDS) pattern_var5 = 0;
          if (pattern_var6 == NUM_LEDS) pattern_var6 = 0;
          if (pattern_var7 == NUM_LEDS) pattern_var7 = 0;
          if (pattern_var8 == NUM_LEDS) pattern_var8 = 0;
          if (pattern_var9 == NUM_LEDS) pattern_var9 = 0;
          break;
        case 4:                                                      // tray 1 & 2
          leds_t1[pattern_var9] = leds_t2[pattern_var9] = 0xFF00FF;  // 255
          leds_t1[pattern_var8] = leds_t2[pattern_var8] = 0xC800C8;  // 200
          leds_t1[pattern_var7] = leds_t2[pattern_var7] = 0xAF00AF;  // 175
          leds_t1[pattern_var6] = leds_t2[pattern_var6] = 0x960096;  // 150
          leds_t1[pattern_var5] = leds_t2[pattern_var5] = 0x7D007D;  // 125
          leds_t1[pattern_var4] = leds_t2[pattern_var4] = 0x640064;  // 100
          leds_t1[pattern_var3] = leds_t2[pattern_var3] = 0x4B004B;  // 75
          leds_t1[pattern_var2] = leds_t2[pattern_var2] = 0x320032;  // 50
          leds_t1[pattern_var1] = leds_t2[pattern_var1] = 0x230023;  // 35
          leds_t1[pattern_var0] = leds_t2[pattern_var0] = 0x140014;  // 20
          pattern_var0++;
          pattern_var1++;
          pattern_var2++;
          pattern_var3++;
          pattern_var4++;
          pattern_var5++;
          pattern_var6++;
          pattern_var7++;
          pattern_var8++;
          pattern_var9++;
          if (pattern_var0 == NUM_LEDS) pattern_var0 = 0;
          if (pattern_var1 == NUM_LEDS) pattern_var1 = 0;
          if (pattern_var2 == NUM_LEDS) pattern_var2 = 0;
          if (pattern_var3 == NUM_LEDS) pattern_var3 = 0;
          if (pattern_var4 == NUM_LEDS) pattern_var4 = 0;
          if (pattern_var5 == NUM_LEDS) pattern_var5 = 0;
          if (pattern_var6 == NUM_LEDS) pattern_var6 = 0;
          if (pattern_var7 == NUM_LEDS) pattern_var7 = 0;
          if (pattern_var8 == NUM_LEDS) pattern_var8 = 0;
          if (pattern_var9 == NUM_LEDS) pattern_var9 = 0;
          break;
        case 5:                                                      // tray 2 & 3
          leds_t3[pattern_var9] = leds_t2[pattern_var9] = 0xFF00FF;  // 255
          leds_t3[pattern_var8] = leds_t2[pattern_var8] = 0xC800C8;  // 200
          leds_t3[pattern_var7] = leds_t2[pattern_var7] = 0xAF00AF;  // 175
          leds_t3[pattern_var6] = leds_t2[pattern_var6] = 0x960096;  // 150
          leds_t3[pattern_var5] = leds_t2[pattern_var5] = 0x7D007D;  // 125
          leds_t3[pattern_var4] = leds_t2[pattern_var4] = 0x640064;  // 100
          leds_t3[pattern_var3] = leds_t2[pattern_var3] = 0x4B004B;  // 75
          leds_t3[pattern_var2] = leds_t2[pattern_var2] = 0x320032;  // 50
          leds_t3[pattern_var1] = leds_t2[pattern_var1] = 0x230023;  // 35
          leds_t3[pattern_var0] = leds_t2[pattern_var0] = 0x140014;  // 20
          pattern_var0++;
          pattern_var1++;
          pattern_var2++;
          pattern_var3++;
          pattern_var4++;
          pattern_var5++;
          pattern_var6++;
          pattern_var7++;
          pattern_var8++;
          pattern_var9++;
          if (pattern_var0 == NUM_LEDS) pattern_var0 = 0;
          if (pattern_var1 == NUM_LEDS) pattern_var1 = 0;
          if (pattern_var2 == NUM_LEDS) pattern_var2 = 0;
          if (pattern_var3 == NUM_LEDS) pattern_var3 = 0;
          if (pattern_var4 == NUM_LEDS) pattern_var4 = 0;
          if (pattern_var5 == NUM_LEDS) pattern_var5 = 0;
          if (pattern_var6 == NUM_LEDS) pattern_var6 = 0;
          if (pattern_var7 == NUM_LEDS) pattern_var7 = 0;
          if (pattern_var8 == NUM_LEDS) pattern_var8 = 0;
          if (pattern_var9 == NUM_LEDS) pattern_var9 = 0;
          break;
        case 6:                                                      // tray 1 & 3
          leds_t1[pattern_var9] = leds_t3[pattern_var9] = 0xFF00FF;  // 255
          leds_t1[pattern_var8] = leds_t3[pattern_var8] = 0xC800C8;  // 200
          leds_t1[pattern_var7] = leds_t3[pattern_var7] = 0xAF00AF;  // 175
          leds_t1[pattern_var6] = leds_t3[pattern_var6] = 0x960096;  // 150
          leds_t1[pattern_var5] = leds_t3[pattern_var5] = 0x7D007D;  // 125
          leds_t1[pattern_var4] = leds_t3[pattern_var4] = 0x640064;  // 100
          leds_t1[pattern_var3] = leds_t3[pattern_var3] = 0x4B004B;  // 75
          leds_t1[pattern_var2] = leds_t3[pattern_var2] = 0x320032;  // 50
          leds_t1[pattern_var1] = leds_t3[pattern_var1] = 0x230023;  // 35
          leds_t1[pattern_var0] = leds_t3[pattern_var0] = 0x140014;  // 20
          pattern_var0++;
          pattern_var1++;
          pattern_var2++;
          pattern_var3++;
          pattern_var4++;
          pattern_var5++;
          pattern_var6++;
          pattern_var7++;
          pattern_var8++;
          pattern_var9++;
          if (pattern_var0 == NUM_LEDS) pattern_var0 = 0;
          if (pattern_var1 == NUM_LEDS) pattern_var1 = 0;
          if (pattern_var2 == NUM_LEDS) pattern_var2 = 0;
          if (pattern_var3 == NUM_LEDS) pattern_var3 = 0;
          if (pattern_var4 == NUM_LEDS) pattern_var4 = 0;
          if (pattern_var5 == NUM_LEDS) pattern_var5 = 0;
          if (pattern_var6 == NUM_LEDS) pattern_var6 = 0;
          if (pattern_var7 == NUM_LEDS) pattern_var7 = 0;
          if (pattern_var8 == NUM_LEDS) pattern_var8 = 0;
          if (pattern_var9 == NUM_LEDS) pattern_var9 = 0;
          break;
        case 7:                                                                              // tray 1,2 & 3
          leds_t1[pattern_var9] = leds_t2[pattern_var9] = leds_t3[pattern_var9] = 0xFF00FF;  // 255
          leds_t1[pattern_var8] = leds_t2[pattern_var8] = leds_t3[pattern_var8] = 0xC800C8;  // 200
          leds_t1[pattern_var7] = leds_t2[pattern_var7] = leds_t3[pattern_var7] = 0xAF00AF;  // 175
          leds_t1[pattern_var6] = leds_t2[pattern_var6] = leds_t3[pattern_var6] = 0x960096;  // 150
          leds_t1[pattern_var5] = leds_t2[pattern_var5] = leds_t3[pattern_var5] = 0x7D007D;  // 125
          leds_t1[pattern_var4] = leds_t2[pattern_var4] = leds_t3[pattern_var4] = 0x640064;  // 100
          leds_t1[pattern_var3] = leds_t2[pattern_var3] = leds_t3[pattern_var3] = 0x4B004B;  // 75
          leds_t1[pattern_var2] = leds_t2[pattern_var2] = leds_t3[pattern_var2] = 0x320032;  // 50
          leds_t1[pattern_var1] = leds_t2[pattern_var1] = leds_t3[pattern_var1] = 0x230023;  // 35
          leds_t1[pattern_var0] = leds_t2[pattern_var0] = leds_t3[pattern_var0] = 0x140014;  // 20
          pattern_var0++;
          pattern_var1++;
          pattern_var2++;
          pattern_var3++;
          pattern_var4++;
          pattern_var5++;
          pattern_var6++;
          pattern_var7++;
          pattern_var8++;
          pattern_var9++;
          if (pattern_var0 == NUM_LEDS) pattern_var0 = 0;
          if (pattern_var1 == NUM_LEDS) pattern_var1 = 0;
          if (pattern_var2 == NUM_LEDS) pattern_var2 = 0;
          if (pattern_var3 == NUM_LEDS) pattern_var3 = 0;
          if (pattern_var4 == NUM_LEDS) pattern_var4 = 0;
          if (pattern_var5 == NUM_LEDS) pattern_var5 = 0;
          if (pattern_var6 == NUM_LEDS) pattern_var6 = 0;
          if (pattern_var7 == NUM_LEDS) pattern_var7 = 0;
          if (pattern_var8 == NUM_LEDS) pattern_var8 = 0;
          if (pattern_var9 == NUM_LEDS) pattern_var9 = 0;
          break;
      }
      FastLED.show();
      delay(10);
      break;
    case 2:
      for (uint16_t i = 0; i < active_led_num_t1; i++) {
        leds_t1[i] = led_value << 16;
      }
      for (uint16_t i = active_led_num_t1; i < NUM_LEDS; i++) {
        leds_t1[i] = 0;
      }
      for (uint16_t i = 0; i < active_led_num_t2; i++) {
        leds_t2[i] = led_value << 16;
      }
      for (uint16_t i = active_led_num_t2; i < NUM_LEDS; i++) {
        leds_t2[i] = 0;
      }
      for (uint16_t i = 0; i < active_led_num_t3; i++) {
        leds_t3[i] = led_value << 16;
      }
      for (uint16_t i = active_led_num_t3; i < NUM_LEDS; i++) {
        leds_t3[i] = 0;
      }
      FastLED.show();
      delay(5);
      if (flag_pat_2)
        led_value++;
      else
        led_value--;
      if (led_value == 100) {
        flag_pat_2 = false;
      } else if (led_value == 10) {
        flag_pat_2 = true;
      }
      break;
    case 3:
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = leds_t2[i] = leds_t3[i] = warm_whight_dim;
        leds_t1[i] = leds_t2[i] = leds_t3[i] = 100;  // blue
      }
      FastLED.show();
      break;
    case 4:  // side light left
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = 0x000000;
        leds_t2[i] = 0x000000;
        leds_t3[i] = 0x000000;
      }
      for (uint8_t i = left_side_light_led_start; i >= SL_led_num_3; i--) {
        leds_t1[i] = colour_orange_dim;
        leds_t2[i] = colour_orange_dim;
        leds_t3[i] = colour_orange_dim;
      }
      FastLED.show();
      SL_led_num_3--;
      if (SL_led_num_3 == left_side_light_led_end)
        SL_led_num_3 = left_side_light_led_start;
      delay(20);
      break;
    case 5:  // side light right
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = 0x000000;
        leds_t2[i] = 0x000000;
        leds_t3[i] = 0x000000;
      }
      for (uint8_t i = right_side_light_led_start; i <= SL_led_num; i++) {
        leds_t1[i] = colour_orange_dim;
        leds_t2[i] = colour_orange_dim;
        leds_t3[i] = colour_orange_dim;
      }
      for (uint8_t i = 0; (i <= SL_led_num_2) && (flag_SL); i++) {
        leds_t1[i] = colour_orange_dim;
        leds_t2[i] = colour_orange_dim;
        leds_t3[i] = colour_orange_dim;
      }
      FastLED.show();
      if (!flag_SL) {
        SL_led_num++;
        if (SL_led_num == NUM_LEDS) {
          flag_SL = true;
        }
      } else {
        SL_led_num_2++;
        if (SL_led_num_2 == right_side_light_led_end) {
          SL_led_num = right_side_light_led_start;
          SL_led_num_2 = 0;
          flag_SL = false;
        }
      }
      delay(20);
      break;
    case 6:  // side light left & right
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = 0x000000;
        leds_t2[i] = 0x000000;
        leds_t3[i] = 0x000000;
      }
      for (uint8_t i = left_side_light_led_start; i >= SL_led_num_3; i--) {
        leds_t1[i] = colour_orange_dim;
        leds_t2[i] = colour_orange_dim;
        leds_t3[i] = colour_orange_dim;
      }
      for (uint8_t i = right_side_light_led_start; i <= SL_led_num; i++) {
        leds_t1[i] = colour_orange_dim;
        leds_t2[i] = colour_orange_dim;
        leds_t3[i] = colour_orange_dim;
      }
      for (uint8_t i = 0; (i <= SL_led_num_2) && (flag_SL); i++) {
        leds_t1[i] = colour_orange_dim;
        leds_t2[i] = colour_orange_dim;
        leds_t3[i] = colour_orange_dim;
      }
      FastLED.show();
      SL_led_num_3--;
      if (SL_led_num_3 == left_side_light_led_end)
        SL_led_num_3 = left_side_light_led_start;
      if (!flag_SL) {
        SL_led_num++;
        if (SL_led_num == NUM_LEDS) {
          flag_SL = true;
        }
      } else {
        SL_led_num_2++;
        if (SL_led_num_2 == right_side_light_led_end) {
          SL_led_num = right_side_light_led_start;
          SL_led_num_2 = 0;
          flag_SL = false;
        }
      }
      delay(20);
      break;
    case 7:  // motion
      if ((millis() - pat7_count) > 50) {
        for (uint16_t i = 0; i < NUM_LEDS; i++) {
          // leds_t1[i] = leds_t2[i] = leds_t3[i] = 64;
          leds_t1[i] = leds_t2[i] = leds_t3[i] = warm_whight_dim;
        }
        // leds_t1[pattern_var1] = leds_t2[pattern_var1] = leds_t3[pattern_var1] = 100;
        // leds_t1[pattern_var2] = leds_t2[pattern_var2] = leds_t3[pattern_var2] = 150;
        // leds_t1[pattern_var3] = leds_t2[pattern_var3] = leds_t3[pattern_var3] = 200;
        // leds_t1[pattern_var4] = leds_t2[pattern_var4] = leds_t3[pattern_var4] = 150;
        // leds_t1[pattern_var5] = leds_t2[pattern_var5] = leds_t3[pattern_var5] = 100;
        // warm_white_dim = 0x325509
        // warm_whight = 0x96FF1A
        leds_t1[pattern_var1] = leds_t2[pattern_var1] = leds_t3[pattern_var1] = 0x538D0F;
        leds_t1[pattern_var2] = leds_t2[pattern_var2] = leds_t3[pattern_var2] = 0x75C715;
        leds_t1[pattern_var3] = leds_t2[pattern_var3] = leds_t3[pattern_var3] = 0x96FF1A;
        leds_t1[pattern_var4] = leds_t2[pattern_var4] = leds_t3[pattern_var4] = 0x75C715;
        leds_t1[pattern_var5] = leds_t2[pattern_var5] = leds_t3[pattern_var5] = 0x538D0F;
        pattern_var1++;
        pattern_var2++;
        pattern_var3++;
        pattern_var4++;
        pattern_var5++;
        if (pattern_var1 == NUM_LEDS) pattern_var1 = 0;
        if (pattern_var2 == NUM_LEDS) pattern_var2 = 0;
        if (pattern_var3 == NUM_LEDS) pattern_var3 = 0;
        if (pattern_var4 == NUM_LEDS) pattern_var4 = 0;
        if (pattern_var5 == NUM_LEDS) pattern_var5 = 0;
        FastLED.show();
        pat7_count = millis();
      }
      break;
    case 10:
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = 0x006400;
        leds_t2[i] = 0x006400;
        leds_t3[i] = 0x006400;
      }
      FastLED.show();
      break;
    case 101:  // reboot
      break;
    case 201:
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = leds_t2[i] = leds_t3[i] = warm_whight;
      }
      FastLED.show();
      break;
    case 202:
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = 0xC0FFCB;  // pink
        leds_t2[i] = 0x69FFB4;  // hot pink
        leds_t3[i] = 0x14FF93;  // deep pink
      }
      FastLED.show();
      break;
    case 203:
      for (uint16_t i = 0; i < NUM_LEDS; i++) {
        leds_t1[i] = leds_t2[i] = leds_t3[i] = 0xFFFFFF;  // full white
      }
      FastLED.show();
      break;
  }
  if (driver.recv(buf, &buflen))  // Non-blocking
  {
    if ((((millis() - count_RF_call) > 5000) || device_id != buf[1]) && ((buf[0] >> 1) == 2)) {
      device_id = buf[1];
      SerialBT.print("Address :  ");
      SerialBT.println(device_id);
      //      SerialBT.print("\t");
      //      SerialBT.print("Button Pressed :  ");
      //      SerialBT.print(buf[0] >> 1);
      //      SerialBT.print("\t");
      //      SerialBT.print("Require charging ? :  ");
      //      SerialBT.println(buf[0] & 1);
      Serial.println(device_id);
      count_RF_call = millis();
    }
  }
}
//GRB
