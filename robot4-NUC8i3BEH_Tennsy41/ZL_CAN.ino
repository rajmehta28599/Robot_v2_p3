/*
  Function directory

  void init_can() // initialise CAN // ID : 0x601 , baudrate : 500kbps , data length : 8
  void init_sync_position_control(uint16_t speed, uint16_t acc, uint16_t dcc)
  void drive_pos_straight(int32_t pos)
  void drive_pos_turn(int32_t pos)
  void init_sync_velocity_control(uint16_t acc, uint16_t dcc)
  void drive_vel_motor_straight(int16_t speedl, int16_t speedr)
  void stop_motor()
  void acc_dcc_time(uint16_t acc, uint16_t dcc)
  void acc_time(uint16_t acc)
  void dcc_time(uint16_t dcc)
  int32_t encoder_read_left()
  int32_t encoder_read_right()
  void error_check()
  void clear_error()
  void clear_encoder()
  void quick_stop_response(int8_t option) // set action of QS
  uint16_t read_temperature() // show temperature of driver, left , right motors
  uint8_t CAN_baudrate() // read baudrate
  uint32_t speed_read() // read motorspeed
  bool emergency_switch_read() // read estop 30-09-2022
*/

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> ZL;
CAN_message_t msg;
CAN_message_t rec;
CAN_message_t rec1;


void init_can() {
  ZL.begin(); ZL.setBaudRate(500000);
  msg.id = 0x601;  msg.len = 8;
}


byte emergency_switch_read()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x41;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  ZL.read(rec);
  timeOut1 = millis();
  while (rec.buf[1] != 0x41 && rec.buf[2] != 0x60) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 304;
      break;
    }
  }
  if (rec.buf[4] == 0x27)
  {
    return (rec.buf[5] >> 7); // 0b0x ; x : 0 switch open ; 1 switch presssed
  }
  return 0b11; // not applicable
}


void init_sync_position_control(uint16_t speed, uint16_t acc, uint16_t dcc) {
  msg.buf[0] = 0x2F;
  msg.buf[1] = 0x60;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x01;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set position mode
  timeOut1 = millis();
  while ((rec.buf[1] != 0x60) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 305;
      break;
    }
  }
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x83;
  msg.buf[3] = 0x01;
  msg.buf[4] = acc;
  msg.buf[5] = acc >> 8;
  ZL.write(msg);       // set left acc
  timeOut1 = millis();
  while ((rec.buf[1] != 0x83) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 311;
      break;
    }
  }
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  timeOut1 = millis();
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 312;
      break;
    }
  }
  msg.buf[1] = 0x84;
  msg.buf[3] = 0x01;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  ZL.write(msg);       // set left dcc
  timeOut1 = millis();
  while ((rec.buf[1] != 0x84) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 313;
      break;
    }
  }
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right dcc
  timeOut1 = millis();
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 314;
      break;
    }
  }
  msg.buf[1] = 0x81;
  msg.buf[3] = 0x01;
  msg.buf[4] = speed;
  msg.buf[5] = speed >> 8;
  ZL.write(msg);       // set left speed
  timeOut1 = millis();
  while ((rec.buf[1] != 0x81) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 315;
      break;
    }
  }
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right speed
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
  }
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x06;
  msg.buf[5] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while (rec.buf[1] != 0x40) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 316;
      break;
    }
  }
  msg.buf[4] = 0x07;
  ZL.write(msg);
  msg.buf[4] = 0x0F;
  ZL.write(msg);         // enable
}


void en_motor()
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x06;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  rec.buf[1] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x40) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 301;
      break;
    }
  }
  msg.buf[4] = 0x07;
  ZL.write(msg);
  msg.buf[4] = 0x0F;
  ZL.write(msg);         // enable
  //  // Serial1.print(" en_motor()");
}

void drive_pos_straight(int32_t pos)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x7A;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = (pos ^ 0xFFFFFFFF) + 1;
  msg.buf[5] = ((pos ^ 0xFFFFFFFF) + 1) >> 8;
  msg.buf[6] = ((pos ^ 0xFFFFFFFF) + 1) >> 16;
  msg.buf[7] = ((pos ^ 0xFFFFFFFF) + 1) >> 24;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x7A) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 306;
      break;
    }
  }
  msg.buf[3] = 0x02;
  msg.buf[4] = pos;
  msg.buf[5] = pos >> 8;
  msg.buf[6] = pos >> 16;
  msg.buf[7] = pos >> 24;
  ZL.write(msg);
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
  }
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x4F;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while ((rec.buf[1] != 0x40) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
  }
  msg.buf[4] = 0x5F;
  ZL.write(msg);
}

void drive_pos_turn(int32_t pos)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x7A;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = pos;
  msg.buf[5] = pos >> 8;
  msg.buf[6] = pos >> 16;
  msg.buf[7] = pos >> 24;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x7A) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 307;
      break;
    }
  }
  msg.buf[3] = 0x02;
  msg.buf[4] = pos;
  msg.buf[5] = pos >> 8;
  msg.buf[6] = pos >> 16;
  msg.buf[7] = pos >> 24;
  ZL.write(msg);
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
  }
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x4F;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while ((rec.buf[1] != 0x40) || (rec.buf[3] != 0x00)) {
    ZL.read(rec);
  }
  msg.buf[4] = 0x5F;
  ZL.write(msg);
  //  // Serial1.println("drive_pos_turn");
}

void init_sync_velocity_control(uint16_t acc, uint16_t dcc)
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x0F;
  msg.buf[2] = 0x20;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x01;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set sync control
  timeOut1 = millis();
  while ((rec.buf[1] != 0x0F) || (rec.buf[2] != 0x20)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 308;
      break;
    }
  }
  msg.buf[0] = 0x2F;
  msg.buf[1] = 0x60;
  msg.buf[2] = 0x60;
  msg.buf[4] = 0x03;
  ZL.write(msg);       // set velocity mode
  while ((rec.buf[1] != 0x60) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
  }
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x83;
  msg.buf[3] = 0x01;
  msg.buf[4] = acc;
  msg.buf[5] = acc >> 8;
  ZL.write(msg);       // set left acc
  timeOut1 = millis();
  while ((rec.buf[1] != 0x83) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 311;
      break;
    }
  }
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  timeOut1 = millis();
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 312;
      break;
    }
  }
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x84;
  msg.buf[3] = 0x01;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  ZL.write(msg);       // set left dcc
  timeOut1 = millis();
  while ((rec.buf[1] != 0x84) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 313;
      break;
    }
  }
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right dcc
  timeOut1 = millis();
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 314;
      break;
    }
  }
  //  msg.buf[0] = 0x2B;
  //  msg.buf[1] = 0x40;
  //  msg.buf[3] = 0x00;
  //  msg.buf[4] = 0x06;
  //  msg.buf[5] = 0x00;
  //  ZL.write(msg);
  //  while (!ZL.read(rec));
  //  msg.buf[4] = 0x07;
  //  ZL.write(msg);
  //  while (!ZL.read(rec));
  //  msg.buf[4] = 0x0F;
  //  ZL.write(msg);         // enable
  //  while (!ZL.read(rec));
}


void drive_vel_motor_straight(int16_t speedl, int16_t speedr)
{
  /*
     controlle speed according 100 rpm. and for error solving max limit +- 50.
  */

  /* pid speed filter 24/11/22 12:43pm */
  if (speedl > (_maxSpeed + 30)) speedl = (_maxSpeed + 30);
  else if (speedl < (-_maxSpeed - 30)) speedl = (-_maxSpeed - 30);

  if (speedr > (_maxSpeed + 30)) speedr = (_maxSpeed + 30);
  else if (speedr < (-_maxSpeed - 30)) speedr = (-_maxSpeed - 30);

  /* final speed filter 24/11/22 12:43pm */
  if (speedl > 280 ) speedl = 280;
  else if (speedl < -280) speedl = -280;
  if (speedr > 280 ) speedr = 280;
  else if (speedr < -280) speedr = -280;

  speedl_data = speedl;
  speedr_data = speedr;

  msg.buf[0] = 0x23;
  msg.buf[1] = 0xFF;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x03;
  msg.buf[4] = (speedl ^ 0xFFFF) + 1;
  msg.buf[5] = ((speedl ^ 0xFFFF) + 1) >> 8;
  msg.buf[6] = speedr;
  msg.buf[7] = speedr >> 8;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0xFF) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x03)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 309;
      break;
    }
  }
}

void stop_motor()
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  rec.buf[1] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x40) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 302;
      break;
    }
  }
  //  // Serial1.print(" stop_motor()");
}

void acc_dcc_time(uint16_t acc, uint16_t dcc)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x83;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = acc;
  msg.buf[5] = acc >> 8;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set left acc
  timeOut1 = millis();
  while ((rec.buf[0] != 0x60) || (rec.buf[1] != 0x83) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 311;
      break;
    }
  }
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  timeOut1 = millis();
  while ((rec.buf[0] != 0x60) || (rec.buf[3] != 0x02)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 312;
      break;
    }
  }
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x84;
  msg.buf[3] = 0x01;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  ZL.write(msg);       // set left dcc
  timeOut1 = millis();
  while ((rec.buf[0] != 0x60) || (rec.buf[1] != 0x84) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 313;
      break;
    }
  }
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x84;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x02;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set right dcc
  timeOut1 = millis();
  while ((rec.buf[0] != 0x60) || (rec.buf[1] != 0x84) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x02)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 314;
      break;
    }
  }
}

void acc_time(uint16_t acc)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x83;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = acc;
  msg.buf[5] = acc >> 8;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set left acc
  timeOut1 = millis();
  while ((rec.buf[1] != 0x83) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 311;
      break;
    }
  }
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right acc
  timeOut1 = millis();
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 312;
      break;
    }
  }
}

void dcc_time(uint16_t dcc)
{
  msg.buf[0] = 0x23;
  msg.buf[1] = 0x84;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = dcc;
  msg.buf[5] = dcc >> 8;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);       // set left dcc
  timeOut1 = millis();
  while ((rec.buf[1] != 0x84) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 313;
      break;
    }
  }
  msg.buf[3] = 0x02;
  ZL.write(msg);       // set right dcc
  timeOut1 = millis();
  while (rec.buf[3] != 0x02) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 314;
      break;
    }
  }
}

uint32_t read_dcc_left()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x84;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while ((rec.buf[1] != 0x84) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
  }
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4];
}
uint32_t read_dcc_right()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x84;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x02;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while ((rec.buf[1] != 0x84) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x02)) {
    ZL.read(rec);
  }
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4];
}

int encoder_read_left() {
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x64;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x01;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x64) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x01)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 317;
      break;
    }
  }
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4];
}


int encoder_read_right() {
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x64;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x02;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x64) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x02)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 318;
      break;
    }
  }
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4];
}


bool error_check() {
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x3F;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while ((rec.buf[1] != 0x3F) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
  }
  if (!rec.buf[4] && !rec.buf[5] && !rec.buf[6] && !rec.buf[7]) {
    //    // Serial1.println("No Error");
    return 0;
  }
  else {
    switch (rec.buf[4]) {
      case 1:
        // Serial1.println("Over voltage");
        return 1;
        ErrorCode = 319;
        break;
      case 2:
        // Serial1.println("Under voltage");
        return 1;
        ErrorCode = 321;
        break;
      default:
        if (rec.buf[5] == 1) {
          // Serial1.println("EEPROM read write error");
          ErrorCode = 322;
        }
        else
        {
          switch (rec.buf[6])
          {
            case 0x04:
              // Serial1.println("err_right : Over current");
              ErrorCode = 320;
              break;
            case 0x08:
              // Serial1.println("err_right : Over load");
              ErrorCode = 327;
              break;
            case 0x10:
              // Serial1.println("err_right : Current out of tolerance");
              ErrorCode = 323;
              break;
            case 0x20:
              // Serial1.println("err_right : Encoder out of tolerance");
              ErrorCode = 325;
              break;
            case 0x40:
              // Serial1.println("err_right : velocity out of tolerance");
              ErrorCode = 324;
              break;
            case 0x80:
              // Serial1.println("err_right : reference voltage error");
              ErrorCode = 326;
              break;
          }
          switch (rec.buf[4])
          {
            case 0x04:
              // Serial1.println("err_left : Over current");
              ErrorCode = 320;
              break;
            case 0x08:
              // Serial1.println("err_left : Over load");
              ErrorCode = 327;
              break;
            case 0x10:
              // Serial1.println("err_left : Current out of tolerance");
              ErrorCode = 323;
              break;
            case 0x20:
              // Serial1.println("err_left : Encoder out of tolerance");
              ErrorCode = 325;
              break;
            case 0x40:
              // Serial1.println("err_left : velocity out of tolerance");
              ErrorCode = 324;
              break;
            case 0x80:
              // Serial1.println("err_left : reference voltage error");
              ErrorCode = 326;
              break;
          }
        }
    }
    if (rec.buf[5] == 2)
    {
      // Serial1.println("err_left : Hall error");
      ErrorCode = 328;
    }
    if (rec.buf[7] == 2)
    {
      // Serial1.println("err_right : Hall error");
      ErrorCode = 328;
    }
  }
}

void clear_error()
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x40;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0X80;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  rec.buf[1] = 0x00;
  ZL.write(msg);
  while ((rec.buf[1] != 0x40) || (rec.buf[2] != 0x60)) {
    ZL.read(rec);
  }
}

void quick_stop_response(int8_t option)
{
  /*
    5 : normal stop
    6 : decelerate to stop
    7 : Emergency stop
  */
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x5A;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x00;
  msg.buf[4] = option;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
}

uint16_t read_temperature()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x32;
  msg.buf[2] = 0x20;
  msg.buf[3] = 0x03;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  // Serial1.print(" ,");
  // Serial1.print(((rec.buf[5] << 8) + rec.buf[4]) / 10); //// Serial1.print("\t"); // driver
  msg.buf[3] = 0x01;
  ZL.write(msg);
  while (!ZL.read(rec));
  // Serial1.print(" ,");
  // Serial1.print(((rec.buf[5] << 8) + rec.buf[4]) / 10); //// Serial1.print("\t"); // left motor
  msg.buf[3] = 0x02;
  ZL.write(msg);
  while (!ZL.read(rec));
  // Serial1.print(" ,");
  // Serial1.println(((rec.buf[5] << 8) + rec.buf[4]) / 10); // right motor
  return (rec.buf[5] << 8) + rec.buf[4];
}

uint8_t CAN_baudrate()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x0B;
  msg.buf[2] = 0x20;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  while (!ZL.read(rec));
  return rec.buf[4];
}

void clear_encoder()
{
  msg.buf[0] = 0x2B;
  msg.buf[1] = 0x05;
  msg.buf[2] = 0x20;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x03;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x05) || (rec.buf[2] != 0x20)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 303;
      break;
    }
  }
  // Serial1.print("clear_encoder");
}

uint32_t speed_read()
{
  msg.buf[0] = 0x40;
  msg.buf[1] = 0x6C;
  msg.buf[2] = 0x60;
  msg.buf[3] = 0x03;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  ZL.write(msg);
  timeOut1 = millis();
  while ((rec.buf[1] != 0x6C) || (rec.buf[2] != 0x60) || (rec.buf[3] != 0x03)) {
    ZL.read(rec);
    timeOut = millis() - timeOut1;
    if (timeOut > 3) {
      ErrorCode = 310;
      break;
    }
  }
  return (rec.buf[7] << 24) + (rec.buf[6] << 16) + (rec.buf[5] << 8) + rec.buf[4]; // High 16 left, Low 16 right
}
