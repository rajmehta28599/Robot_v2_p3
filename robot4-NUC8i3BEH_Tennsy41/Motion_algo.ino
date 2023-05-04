
/* distance between two wheel in mm */
//#define wheel_dis 266 // 258
#define wheel_center 133 //129
/* one whell radius in mm*/
#define wheel_dia 130
//#define wheel_radius 64

int16_t pos_enc_count, pre_pos_enc_count;
int speed_position, adspeed_position;
float turning_radius, w_robot;
int TurnDirection;
int rpm_l, rpm_r;
int speedSety, adspeedSety;
float speedSetyFloat;

void set_y() {
  //  angle_pos_y = (NUC_y - NUC_y_setpoint) * (dis_to_pos + 0.7);
  angle_pos_y = NUC_y - NUC_y_setpoint;
  speedSetyFloat = (angle_pos_y * 0.1);

  if (speedSetyFloat == 0) speedSety = 0;
  else if (speedSetyFloat >= 0 and speedSetyFloat < 6) speedSety = 5;
  else if (speedSetyFloat < 0 and speedSetyFloat > -6) speedSety = -5;
  else speedSety = int(speedSetyFloat);  // 30/11/22 12:33pm
  /* acc deacc for turning */
  if (adspeedSety != speedSety) {
    if (speedSety > adspeedSety) {
      adspeedSety++;
    }
    else {
      adspeedSety--;
    }
  }
  /* Speed limit for straight */
  if (adspeedSety > 20) adspeedSety = 20;
  else if (adspeedSety < -20) adspeedSety = -20;
  drive_vel_motor_straight(adspeedSety, adspeedSety);
  if (mode8) {  //A
    mode8 = 0; nuc_mode8 = 0;
  }
}


void set_angle()
{
  //  Serial7.println("1");
  if (NUC_tag_state || NUC_to_BMI_yaw)
  {
    encoder_count = 0;
    encoder_count0 = encoder_read_right(); //A
    if (init_sync_pos_flag)
    {
      init_sync_velocity_control(0, 0);
      motor_left_speed = motor_right_speed = 0;
      init_sync_pos_flag = false;
      clear_encoder();
      adspeedSety = speedl_data;
    }
    else if (abs(error_angle) < 4) {
      set_y();
    }
    else {
      adspeedSety = speedl_data;
      if (abs(error_angle) <= 20) {
        if (error_angle > 10) adspeed_position = speed_position = 3;
        else if (error_angle < -10) adspeed_position = speed_position = -3;
        else if (error_angle < -2) adspeed_position = speed_position = -2;
        else if (error_angle > 2) adspeed_position = speed_position = 2;
        else if (error_angle < -1) adspeed_position = speed_position = -1;
        else if (error_angle > 1) adspeed_position = speed_position = 1;
      }
      else if (abs(error_angle) <= 500) {
        if (error_angle > 40) speed_position = 5;
        else if (error_angle < -40) speed_position = -5;
        else if (error_angle > 20) speed_position = 4;
        else if (error_angle < -20) speed_position = -4;
      }
      else {
        if (error_angle > 0) {
          speed_position = 20;
        }
        else {
          speed_position = -20;
        }
      }
      if (adspeed_position != speed_position) {
        if (adspeed_position > speed_position) adspeed_position--;
        else adspeed_position++;
      }

      /* Speed limit for turning */
      if (adspeed_position > 20) adspeed_position = 20;
      else if (adspeed_position < -20) adspeed_position = -20;

      drive_vel_motor_straight(adspeed_position, -adspeed_position);
    }
  }
  else {
    drive_vel_motor_straight(0, 0);
  }
}


void Motion_algo() {

  /* robot will dis_to_Travel robot was travel_read*/
  encoder_count0 = encoder_read_right();
  travel_read = encoder_count0 - encoder_count;
  dis_to_Travel = (NUC_distance - 150.85) * dis_to_pos;

  combined_motor_speed = speed_read();
  right_motor_speed = combined_motor_speed;
  right_motor_speed = right_motor_speed * 0.1;
  left_motor_speed = combined_motor_speed >> 16;
  left_motor_speed = left_motor_speed * 0.1;
  //  Serial7.print(right_motor_speed);
  //  Serial7.print(" ");
  //  Serial7.println(left_motor_speed);
  //  Serial7.print(error_angle);
  /* angle error solver (PID)  15/11/22 4:31pm*/
  if (abs(error_angle) < 10) kp = 0.1;  // 0.17
  else kp = 0.07;  //12

  P = (error_angle * kp);  // 6 //58
  D = (pre_error_angle - error_angle) * kd;
  I = ki * (I + error_angle);
  diff_in_speed = P + I + D;
  pre_error_angle = error_angle;

  /* controling motor speen when error is big */
  if (speedl_data == 0 && (abs(diff_in_speed) > 10) && NUC_state != 2) {
    if (diff_in_speed > 10) diff_in_speed = 10;
    else if (diff_in_speed < -10) diff_in_speed = -10;
  }
  // ****** motion drive *****
  if (NUC_state == 8) {
    digitalWrite(Laser_pin, 0);
    if (mode8 == 0 && abs(NUC_yaw - NUC_yaw_setpoint) == 0 && abs(NUC_y_setpoint - NUC_y) == 0) {
      stop_motor();
      en_motor();
      flag_motion = true;
      flag_velocity_control = true;
      init_sync_velocity_control(1000, 10);
      mode8 = 1;
    }
    if (mode8 == 1 && abs(NUC_yaw - NUC_yaw_setpoint) == 0 && abs(NUC_y_setpoint - NUC_y) != 0) {
      set_y();
    }
    if (flag_motion && flag_velocity_control && abs(NUC_y_setpoint - NUC_y) == 0) {
      drive_vel_motor_straight(40, 40);
    }
  }
  else if (NUC_state == 0 || NUC_state >= 4) {
    flag_motion = false;
    if (flag_velocity_control) {
      flag_stop_velocity = true;
      init_sync_velocity_control(0, 0);
    }
  }
  // Position
  else if (NUC_state == 1) {
    if (flag_velocity_control) {
      flag_stop_velocity = true;
    }
    init_sync_velocity_control(0, 0);
    flag_motion = false;
    flag_velocity_control = false;
    flag_position_control = true;
  }
  // velocity
  else if (NUC_state == 2 || NUC_state == 3) {
    flag_motion = true;
    flag_velocity_control = true;
    flag_position_control = false;
    init_sync_velocity_control(0, 0);
  }
  /* flag_start_velocity once */
  if (flag_start_velocity && NUC_state > 1) {
    if (NUC_state == 3) {
      init_sync_velocity_control(0, 0);
      turning_radius = NUC_x; w_robot = 30;
      TurnDirection = NUC_x_setpoint;
      rpm_l = ((turning_radius + (wheel_center * TurnDirection)) * w_robot) / (3 * wheel_dia);
      rpm_r = ((turning_radius - (wheel_center * TurnDirection)) * w_robot) / (3 * wheel_dia);
    }
    else if (NUC_state == 2) {
      init_sync_velocity_control(0, 0);
      motor_left_speed = motor_right_speed = 0;
    }
    /* after break encoder count not reset algo. */
    if (prev_NUC_state4) {
      encoder_count = encoder_count; prev_NUC_state4 = false;
    }
    else {
      encoder_count = encoder_count0;
    }
    flag_start_velocity = false;
    flag_velocity_cal = true;
  }

  /* Depth cam break */
  if (NUC_state == 4) {
    digitalWrite(Laser_pin, 1);
    mode8 = 0;
    if (speedl_data > 0) {
      motor_left_speed = motor_right_speed;
      drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
      while (right_motor_speed > 0) {
        if ((millis() - motion_counter4) > 1) {
          motion_counter4 = millis();
          combined_motor_speed = speed_read();
          right_motor_speed = combined_motor_speed;
          right_motor_speed = right_motor_speed * 0.1;
          motor_right_speed = motor_left_speed = right_motor_speed - 1;
          drive_vel_motor_straight(motor_left_speed, motor_right_speed);
        }
      }
    }
    else {
      flag_velocity_control = flag_stop_velocity = false;
      motor_right_speed = motor_left_speed = 0;
      drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
    }
    flag_dcc_time = 12;
  }
  else if (NUC_state == 5) {
    digitalWrite(Laser_pin, 1);
    mode8 = 0;
    if (speedl_data > 0) {
      motor_left_speed = motor_right_speed;
      drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
      while (right_motor_speed > 0) {
        if ((millis() - motion_counter4) > 3) {
          motion_counter4 = millis();
          combined_motor_speed = speed_read();
          right_motor_speed = combined_motor_speed;
          right_motor_speed = right_motor_speed * 0.1;
          motor_right_speed = motor_left_speed = right_motor_speed - 1;
          drive_vel_motor_straight(motor_left_speed, motor_right_speed);
        }
      }
    }
    else {
      flag_velocity_control = flag_stop_velocity = false;
      motor_right_speed = motor_left_speed = 0;
      drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
    }
    flag_dcc_time = 13;
  }
  else if (NUC_state == 6) {
    mode8 = 0;
    digitalWrite(Laser_pin, 1);
    if (speedl_data > 0) {
      motor_left_speed = motor_right_speed;
      drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
      while (right_motor_speed > 0 && NUC_state == 5) {
        if ((millis() - motion_counter5) > 4) {
          motion_counter5 = millis();
          combined_motor_speed = speed_read();
          right_motor_speed = combined_motor_speed;
          right_motor_speed = right_motor_speed * 0.1;
          //          motor_right_speed = motor_left_speed = motor_left_speed - 1;
          motor_right_speed = motor_left_speed = right_motor_speed - 1;
          drive_vel_motor_straight(motor_left_speed, motor_right_speed);
        }
      }
    }
    else {
      flag_velocity_control = flag_stop_velocity = false;
      motor_right_speed = motor_left_speed = 0;
      drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
    }
    flag_dcc_time = 14;
  }
  else if (NUC_state == 7) {
    digitalWrite(Laser_pin, 1);
    drive_vel_motor_straight(0, 0);
    flag_stop_velocity = false;
    flag_velocity_control = false;
    motor_right_speed = motor_left_speed = 0;
    flag_dcc_time = 15;
  }

  /* for deacc motor speed */
  if (flag_stop_velocity) {
    if (flag_dcc_time == 2) {
      if (speedl_data > 0) {
        motor_left_speed = motor_right_speed;
        drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
        while (speedl_data > 0) {
          if ((millis() - motion_counter0) > 5) {
            motion_counter0 = millis();
            motor_right_speed = motor_left_speed = motor_left_speed - 1;
            drive_vel_motor_straight(motor_left_speed, motor_right_speed);
          }
        }
      }
      flag_dcc_time = 0;
    }
    else if (flag_dcc_time == 3) {
      if (speedl_data > 0) {
        motor_left_speed = motor_right_speed;
        drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
        while (speedl_data > 0) {
          if ((millis() - motion_counter0) > 5) {
            motion_counter0 = millis();
            motor_right_speed = motor_left_speed = motor_left_speed - 1;
            drive_vel_motor_straight(motor_left_speed, motor_right_speed);
          }
        }
      }
      flag_dcc_time = 0;
    }
    else if (speedl_data > 0) {
      motor_left_speed = motor_right_speed;
      drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
      while (speedl_data > 0) {
        if ((millis() - motion_counter0) > 5) {
          motion_counter0 = millis();
          motor_right_speed = motor_left_speed = motor_left_speed - 1;
          drive_vel_motor_straight(motor_left_speed, motor_right_speed);
        }
      }
    }
    else {
      motor_right_speed = motor_left_speed = 0;
      drive_vel_motor_straight(0, 0);
      flag_velocity_control = flag_stop_velocity = false;
    }
  }

  /* position mode */
  //  if (!flag_set_angle && !flag_motion && flag_angle && !flag_velocity_control && (NUC_state == 1)) {
  if (!flag_motion && (NUC_state == 1) && (!flag_stop_velocity)) {
    digitalWrite(Laser_pin, 0);
    init_sync_pos_flag = true;
    if (prev_NUC_state == NUC_state) init_sync_pos_flag = false;

    set_angle();
  }
  /* velocity mode */
  else if (flag_motion && NUC_state == 2) { //|| NUC_state >= 7) {
    digitalWrite(Laser_pin, 0);
    /* motion start step 3 */
    if ((dis_to_Travel + 10000) < travel_read) {
      if (flag_dcc_time == 2 || flag_dcc_time > 11) {
        motor_left_speed = motor_right_speed;
        //        while (abs(motor_left_speed) != 10) {
        if ((millis() - motion_counter10) > 4) {
          motion_counter10 = millis();
          if (speedl_data > 10) {
            motor_right_speed = motor_left_speed = motor_left_speed - 1;
          }
          else if (speedl_data < -10) {
            motor_right_speed = motor_left_speed = motor_left_speed + 1;
          }
          else {
            flag_dcc_time = 3;
          }
          drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
        }
      }
      else {
        if ((millis() - motion_counter10) > 2) {
          motor_left_speed = motor_right_speed;
          motion_counter10 = millis();
          if (speedl_data > 10) {
            motor_right_speed = motor_left_speed = motor_left_speed - 1;
          }
          else if (speedl_data < -10) {
            motor_right_speed = motor_left_speed = motor_left_speed + 1;
          }
        }
        drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
      }
    }
    /* motion start step 2 */
    else if ((dis_to_Travel - 10000) < travel_read) {  //10000, 7000 15/11/22 5:56pm
      if (flag_dcc_time == 1 || flag_dcc_time > 11) {
        motor_left_speed = motor_right_speed;
        drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
        while (abs(speedl_data) != 25) {
          if ((millis() - motion_counter20) > 7) {
            motion_counter20 = millis();
            if (speedl_data > 25) {
              motor_right_speed = motor_left_speed = motor_left_speed - 1;
            }
            else if (speedl_data < 25) {
              motor_right_speed = motor_left_speed = motor_left_speed + 1;
            }
            else {
              flag_dcc_time = 2;
            }
            drive_vel_motor_straight(motor_left_speed, motor_right_speed);
          }
        }
        flag_dcc_time = 2;
      }
      else {
        while (abs(speedl_data) != 25) {
          if ((millis() - motion_counter20) > 6) {
            motion_counter20 = millis();
            if (speedl_data > 25) {
              motor_right_speed = motor_left_speed = motor_left_speed - 1;
            }
            else if (speedl_data < 25) {
              motor_right_speed = motor_left_speed = motor_left_speed + 1;
            }
            else {
              flag_dcc_time = 2;
            }
            drive_vel_motor_straight(motor_left_speed, motor_right_speed);
          }
        }
        drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
      }
    }
    /* motion start step 1 */
    else if (dis_to_Travel > travel_read) {
      if (flag_velocity_cal) {
        if ((millis() - motion_counter150) > 6) {  /* 17/11/22 4:36pm */
          motion_counter150 = millis();
          if (speedl_data < _maxSpeed) {  // speed
            motor_right_speed = motor_left_speed = motor_left_speed + 1;
          }
          else {
            flag_velocity_cal = false;
          }
        }
        drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
      }
      else {
        flag_dcc_time = 1;
        motor_right_speed = motor_left_speed = _maxSpeed;
        drive_vel_motor_straight(motor_left_speed + diff_in_speed, motor_right_speed - diff_in_speed);
      }
    }
  }
  else if (flag_motion && NUC_state == 3) {
    drive_vel_motor_straight(rpm_l, rpm_r);
  }

  /* when motor detect error */
  if (error_check() && !flag_error) {
    flag_error = true;
    error_time_count = millis();
  }
  else if (flag_error && ((millis() - error_time_count) > 3000)) {
    clear_error();
    en_motor();
    init_sync_pos_flag = true;
    flag_error = false;
    prev_NUC_state = 26;
  }
}
