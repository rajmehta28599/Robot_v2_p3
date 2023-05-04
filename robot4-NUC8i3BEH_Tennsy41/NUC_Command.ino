
void NUC_command()
{
  str.toCharArray(temp_data_array, str.length() + 1);
  byte index = 0;
  ptr = strtok(temp_data_array, ",");
  while (ptr != NULL)  {
    NUC_data[index] = ptr; index++; ptr = strtok(NULL, ",");
  }
  pre_NUC_distance = NUC_distance;
  prev_NUC_state = NUC_state;
  prev_NUC_tag_state = NUC_tag_state;
  NUC_state = atoi(NUC_data[0]);  NUC_tag_state = atoi(NUC_data[1]);
  NUC_yaw = atoi(NUC_data[2]);  NUC_yaw_setpoint = atoi(NUC_data[3]);
  if ((NUC_state == 0) && (prev_NUC_state != NUC_state)) {
    drive_vel_motor_straight(0, 0);
  }
  else if (NUC_state == 1) { // position
    if (NUC_tag_state) {
      NUC_y = atof(NUC_data[4]);  NUC_y_setpoint = atof(NUC_data[5]);
    }
    else {
      NUC_y = NUC_y_setpoint = NUC_x = NUC_x_setpoint = 0;
    }
    NUC_distance = 0;
  }
  //Setup straight motion state
  else if (NUC_state == 8) { //A
    if (nuc_mode8 == 0) {
      clear_encoder();
      nuc_mode8 = 1;
    }
    flag_start_velocity = true;
  }
  else if (NUC_state == 2) { // velocity
    if (prev_NUC_state == 4 || prev_NUC_state == 5 || prev_NUC_state == 6) {
      prev_NUC_state4 = true;
    }
    if (NUC_state != prev_NUC_state) {
      flag_start_velocity = true;
    }

    if (NUC_tag_state) {
      NUC_x = atof(NUC_data[4]); NUC_x_setpoint = atof(NUC_data[5]);
    }
    else {
      NUC_y = NUC_y_setpoint = NUC_x = NUC_x_setpoint = 0;
    }
    NUC_distance = atoi(NUC_data[6]);
  }
  else if (NUC_state == 3) { // turn
    if (NUC_state != prev_NUC_state) {
      flag_start_velocity = true;
    }
    if (NUC_tag_state) {
      NUC_x = atof(NUC_data[4]); NUC_x_setpoint = atof(NUC_data[5]); // radius, dir
    }
  }
  if (NUC_distance != pre_NUC_distance) {
    encoder_count = 0;    clear_encoder();    flag_velocity_cal = true;
  }
  /* BMI088 */
  BMI088();
  //  if (NUC_tag_state && (NUC_yaw != _BMI_YAW)) {
  /* error solver (PID)*/
  if (NUC_tag_state) {
    adddata = NUC_yaw - _yaw2;
    error_angle = NUC_yaw_setpoint - NUC_yaw;
    if ((prev_NUC_state == NUC_state) && ((NUC_state == 1) || (NUC_state == 3))) {
      NUC_to_BMI_yaw = 1;
    }
    else {
      NUC_to_BMI_yaw = 0;
    }
    //    Serial7.println(NUC_to_BMI_yaw);
  }
  else {
    error_angle = NUC_yaw_setpoint - _BMI_YAW;
    if (NUC_state == 1) {
      NUC_to_BMI_yaw = 1;
    }
  }

  /* algo for 359 to 0 angle change */
  if (abs(error_angle) > 1800) {
    if (NUC_tag_state) {
      if (NUC_yaw_setpoint > NUC_yaw) error_angle = -(3600 - error_angle);
      else error_angle = error_angle + 3600;
    }
    else if (NUC_yaw_setpoint > _BMI_YAW) error_angle = -(3600 - error_angle);
    else error_angle = error_angle + 3600;
  }
}
