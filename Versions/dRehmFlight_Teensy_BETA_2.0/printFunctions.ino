void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:1000.0"));
    Serial.print(F(" HIGH:2000.0"));
    for (int8_t i = 0; i < NUM_CHANNELS; i++) {
      Serial.print(F(" CH"));
      Serial.print(i+1);
      Serial.print(F(":"));
      Serial.print(channel_pwm[i]);
    }
    Serial.println(F(""));
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:-180.0"));
    Serial.print(F(" HIGH:180.0"));
    Serial.print(F(" thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:-300.0"));
    Serial.print(F(" HIGH:300.0"));
    Serial.print(F(" GyroX:"));
    Serial.print(imu.GyroX);
    Serial.print(F(" GyroY:"));
    Serial.print(imu.GyroY);
    Serial.print(F(" GyroZ:"));
    Serial.println(imu.GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:-2.0"));
    Serial.print(F(" HIGH:2.0"));
    Serial.print(F(" AccX:"));
    Serial.print(imu.AccX);
    Serial.print(F(" AccY:"));
    Serial.print(imu.AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(imu.AccZ);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:-300.0"));
    Serial.print(F(" HIGH:300.0"));
    Serial.print(F(" MagX:"));
    Serial.print(imu.MagX);
    Serial.print(F(" MagY:"));
    Serial.print(imu.MagY);
    Serial.print(F(" MagZ:"));
    Serial.println(imu.MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:-180.0"));
    Serial.print(F(" HIGH:180.0"));
    Serial.print(F(" roll:"));
    Serial.print(imu.roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(imu.pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(imu.yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:-1.0"));
    Serial.print(F(" HIGH:1.0"));
    Serial.print(F(" roll_PID:"));
    Serial.print(imu.roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(imu.pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.println(imu.yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:125.0"));
    Serial.print(F(" HIGH:250.0"));
    for (int i=0; i<numMotors; i++) {
      Serial.print(F(" m"));
      Serial.print(i+1);
      Serial.print(F("_command:"));
      Serial.print(m_command_scaled[i]);
    }
    Serial.println("");
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:0.0"));
    Serial.print(F(" HIGH:1.0"));
    for (int i=0; i<numServos; i++) {
      Serial.print(F(" s"));
      Serial.print(i+1);
      Serial.print(F("_command:"));
      Serial.print(s_command_scaled[i]);
    }
    Serial.println("");
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}

