void BMI088()
{
  /* BMI088 */
  gyroread();
  if (isnan(yaw2)) {
    _BMI_YAW = 44444;
    ErrorCode = 209;
    SCB_AIRCR = 0x05FA0004;
  }
  else {
    _yaw = TO_DEG(yaw2) + 180 ; _yaw2 = _yaw * 10;
    _BMI_Add_YAW = _yaw2 + adddata;
    if (_BMI_Add_YAW < 0) {
      _BMI_Add_YAW = 3600 + _BMI_Add_YAW;
    }
    _BMI_YAW = _BMI_Add_YAW % 3600;
  }
}
