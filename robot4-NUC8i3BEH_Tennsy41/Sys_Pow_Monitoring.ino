void Sys_Pow_Monitoring()
{
  raw_CS_24 = adc.read(MCP3208::Channel::SINGLE_6);
  vol_CS_24 = adc.toAnalog(raw_CS_24);
  
  raw_CS_24_vref = adc.read(MCP3208::Channel::SINGLE_7);
  vol_CS_24_vref = adc.toAnalog(raw_CS_24_vref);

  raw_CS_5 = adc.read(MCP3208::Channel::SINGLE_0);
  vol_CS_5 = adc.toAnalog(raw_CS_5);

  raw_VD_24 = adc.read(MCP3208::Channel::SINGLE_4);
  vol_VD_24 = adc.toAnalog(raw_VD_24);
  //  raw_CS_19 = adc.read(MCP3208::Channel::SINGLE_2);
  //  vol_CS_19 = adc.toAnalog(raw_CS_19);

  //  raw_VD_24_af = adc.read(MCP3208::Channel::SINGLE_5);
  //  vol_VD_24_af = adc.toAnalog(raw_VD_24_af);

  //  raw_VD_NUC = adc.read(MCP3208::Channel::SINGLE_3);
  //  vol_VD_NUC = adc.toAnalog(raw_VD_NUC);

  //  raw_VD_19 = adc.read(MCP3208::Channel::SINGLE_1);
  //  vol_VD_19 = adc.toAnalog(raw_VD_19);

  bat_current = (float(vol_CS_24_vref - vol_CS_24) / 2000) * 25;
  bat_current_sum = bat_current_sum + bat_current - bat_current_filtered;
  bat_current_filtered = bat_current_sum / 200;
  //  current_19 = (float(vol_CS_24_vref - vol_CS_19) / 2000) * 25;
  //  current_19_sum = current_19_sum + current_19 - current_19_filtered;
  //  current_19_filtered = current_19_sum / 200;
  current_5 = (float(vol_CS_24_vref - vol_CS_5) / 2000) * 25;
  current_5_sum = current_5_sum + current_5 - current_5_filtered;
  current_5_filtered = current_5_sum / 200;
  //  bat_current_filtered, voltage_24_filtered, current_5_filtered
  //  if ((bat_current_filtered > 10 or current_19_filtered > 10 or current_5_filtered > 2) or (bat_current_filtered < -10 or current_19_filtered < -10 or current_5_filtered < -10)) {
  //    ErrorCode = 212;
  //  }
  if ((bat_current_filtered > 10 or current_5_filtered > 2) or (bat_current_filtered < -10 or current_5_filtered < -10)) {
    ErrorCode = 212;
  }
  voltage_24 = float(vol_VD_24) / 1000;
  voltage_24_sum = voltage_24_sum + voltage_24 - voltage_24_filtered;
  voltage_24_filtered = (voltage_24_sum / 200);
  //  voltage_24_af = float(vol_VD_24_af) / 1000;
  //  voltage_24_af_sum = voltage_24_af_sum + voltage_24_af - voltage_24_af_filtered;
  //  voltage_24_af_filtered = (voltage_24_af_sum / 200);
  //  voltage_NUC = float(vol_VD_NUC) / 1000;
  //  voltage_NUC_sum = voltage_NUC_sum + voltage_NUC - voltage_NUC_filtered;
  //  voltage_NUC_filtered = (voltage_NUC_sum / 200);
  //  voltage_19 = float(vol_VD_19) / 1000;
  //  voltage_19_sum = voltage_19_sum + voltage_19 - voltage_19_filtered;
  //  voltage_19_filtered = (voltage_19_sum / 200);

  //  if ((voltage_24_filtered > 25.3 or voltage_NUC_filtered > 19.0) or (voltage_24_filtered < 19.3 or voltage_NUC_filtered < 18.5) or (voltage_19_filtered != 19.0)) {
  //    ErrorCode = 211;
  //  }
  if (((voltage_24_filtered * 6) > 25.3) or ((voltage_24_filtered * 6) < 19.3)) {
    ErrorCode = 211;
  }
}
