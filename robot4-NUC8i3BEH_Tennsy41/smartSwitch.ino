//int right_motor_speed, left_motor_speed;

void smartSwitch()
{
  if (digitalRead(main_SW_FB))
  {
    // Serial1.print(" main switch off ");
    if (!flag_shutdown)
    {
      shut_down_count = millis();
      flag_shutdown = true;
    }
    else
    {
      if (((millis() - shut_down_count) > 10000) && NUC_state == 25)
      {
        digitalWrite(main_relay_pin, 0);
      }
      else if (NUC_state != 25) {
        shut_down_count = millis();
      }
      ErrorCode = 101;
    }
  }
  else
  {
    flag_shutdown = false;
    shut_down_count = millis();
  }
  if ((estop_FB == 0) && (Estop_FB_flag == false) && ((millis() - estop_FB_counter) > 200)) {
    //    NUC_state = 5;  // for smooth breaking and start accelerating
    quick_stop_response(6); // deaccelarate break
    /* stop */
    estop_FB_to_NUC = 1;
    ErrorCode = 121;
    if ((millis() - estop_FB_counter) > 1000) {
      /* stop */
      stop_motor();
      //      NUC_state = 5;  // for smooth breaking and start accelerating
      Estop_FB_flag = true;
      estop_FB_counter = millis();
    }
  }
  else if ((estop_FB == 1) && Estop_FB_flag) {
    /* enable motor */
    en_motor();
    estop_FB_to_NUC = 0;
    Estop_FB_flag = false;
    estop_FB_counter = millis();
  }
  else if (estop_FB == 1) {
    estop_FB_counter = millis();
    estop_FB_to_NUC = 0;
  }
}
