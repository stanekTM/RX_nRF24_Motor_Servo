
//*********************************************************************************************************************
// Motor setup
//*********************************************************************************************************************
void motor_setup()
{
#if defined(PIN_5_6_MOTOR1)
  pinMode(pins_motor1[0], OUTPUT);
  pinMode(pins_motor1[1], OUTPUT);
  set_PWM_prescaler(pins_motor1[0], PRESCALER_TIMER0_5_6);
#endif

#if defined(PIN_9_10_MOTOR2)
  pinMode(pins_motor2[0], OUTPUT);
  pinMode(pins_motor2[1], OUTPUT);
  set_PWM_prescaler(pins_motor2[0], PRESCALER_TIMER1_9_10);
#endif

#if defined(PIN_3_11_MOTOR3)
  pinMode(pins_motor3[0], OUTPUT);
  pinMode(pins_motor3[1], OUTPUT);
  set_PWM_prescaler(pins_motor3[0], PRESCALER_TIMER2_3_11);
#endif
}

//*********************************************************************************************************************
// Motor control
//*********************************************************************************************************************
void motor_control()
{
#if defined(MOTOR1)
  int motor1_val = 0;
  
  // Forward motor 1
  if (rc_packet[0] > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motor1_val = map(rc_packet[0], MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, REACTION_MOTOR1, MAX_FORWARD_MOTOR1);
    motor1_val = constrain(motor1_val, REACTION_MOTOR1, MAX_FORWARD_MOTOR1);
    analogWrite(pins_motor1[1], motor1_val);
    digitalWrite(pins_motor1[0], LOW);
  }
  // Reverse motor 1
  else if (rc_packet[0] < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motor1_val = map(rc_packet[0], MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, REACTION_MOTOR1, MAX_REVERSE_MOTOR1);
    motor1_val = constrain(motor1_val, REACTION_MOTOR1, MAX_REVERSE_MOTOR1);
    analogWrite(pins_motor1[0], motor1_val);
    digitalWrite(pins_motor1[1], LOW);
  }
  else
  {
    analogWrite(pins_motor1[0], BRAKE_MOTOR1);
    analogWrite(pins_motor1[1], BRAKE_MOTOR1);
  }
#endif // End MOTOR1

#if defined(MOTOR2)
  int motor2_val = 0;
  
  // Forward motor 2
  if (rc_packet[1] > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motor2_val = map(rc_packet[1], MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, REACTION_MOTOR2, MAX_FORWARD_MOTOR2);
    motor2_val = constrain(motor2_val, REACTION_MOTOR2, MAX_FORWARD_MOTOR2);
    analogWrite(pins_motor2[1], motor2_val);
    digitalWrite(pins_motor2[0], LOW);
  }
  // Reverse motor 2
  else if (rc_packet[1] < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motor2_val = map(rc_packet[1], MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, REACTION_MOTOR2, MAX_REVERSE_MOTOR2);
    motor2_val = constrain(motor2_val, REACTION_MOTOR2, MAX_REVERSE_MOTOR2);
    analogWrite(pins_motor2[0], motor2_val);
    digitalWrite(pins_motor2[1], LOW);
  }
  else
  {
    analogWrite(pins_motor2[0], BRAKE_MOTOR2);
    analogWrite(pins_motor2[1], BRAKE_MOTOR2);
  }
#endif // End MOTOR2

#if defined(MOTOR3)
  int motor3_val = 0;
  
  // Forward motor 3
  if (rc_packet[1] > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motor3_val = map(rc_packet[1], MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, REACTION_MOTOR3, MAX_FORWARD_MOTOR3);
    motor3_val = constrain(motor3_val, REACTION_MOTOR3, MAX_FORWARD_MOTOR3);
    analogWrite(pins_motor3[1], motor3_val);
    digitalWrite(pins_motor3[0], LOW);
  }
  // Reverse motor 3
  else if (rc_packet[1] < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motor3_val = map(rc_packet[1], MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, REACTION_MOTOR3, MAX_REVERSE_MOTOR3);
    motor3_val = constrain(motor3_val, REACTION_MOTOR3, MAX_REVERSE_MOTOR3);
    analogWrite(pins_motor3[0], motor3_val);
    digitalWrite(pins_motor3[1], LOW);
  }
  else
  {
    analogWrite(pins_motor3[0], BRAKE_MOTOR3);
    analogWrite(pins_motor3[1], BRAKE_MOTOR3);
  }
#endif // End MOTOR3

#if defined(MIX_TANK_MOTOR2_3)
  int motor2_val = 0, motor3_val = 0;
  int calc_mix = 258;
  
  int ch1 = rc_packet[0] / 2, ch2 = rc_packet[1] / 2;
  int mix1 = ch1 - ch2 + 1500;
  int mix2 = ch1 + ch2;
  //Serial.println(mix1);
  
  // Forward motor 2
  if (mix1 > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motor2_val = map(mix1, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL - calc_mix, REACTION_MOTOR2, MAX_FORWARD_MOTOR2);
    motor2_val = constrain(motor2_val, REACTION_MOTOR2, MAX_FORWARD_MOTOR2);
    analogWrite(pins_motor2[1], motor2_val); 
    digitalWrite(pins_motor2[0], LOW);
  }
  // Reverse motor 2
  else if (mix1 < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motor2_val = map(mix1, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL + calc_mix, REACTION_MOTOR2, MAX_REVERSE_MOTOR2);
    motor2_val = constrain(motor2_val, REACTION_MOTOR2, MAX_REVERSE_MOTOR2);
    analogWrite(pins_motor2[0], motor2_val);
    digitalWrite(pins_motor2[1], LOW);
  }
  else
  {
    analogWrite(pins_motor2[0], BRAKE_MOTOR2);
    analogWrite(pins_motor2[1], BRAKE_MOTOR2);
  }
  
  // Forward motor 3
  if (mix2 > MID_CONTROL_VAL + DEAD_ZONE)
  {
    motor3_val = map(mix2, MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL - calc_mix, REACTION_MOTOR3, MAX_FORWARD_MOTOR3);
    motor3_val = constrain(motor3_val, REACTION_MOTOR3, MAX_FORWARD_MOTOR3);
    analogWrite(pins_motor3[1], motor3_val);
    digitalWrite(pins_motor3[0], LOW);
  }
  // Reverse motor 3
  else if (mix2 < MID_CONTROL_VAL - DEAD_ZONE)
  {
    motor3_val = map(mix2, MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL + calc_mix, REACTION_MOTOR3, MAX_REVERSE_MOTOR3);
    motor3_val = constrain(motor3_val, REACTION_MOTOR3, MAX_REVERSE_MOTOR3);
    analogWrite(pins_motor3[0], motor3_val);
    digitalWrite(pins_motor3[1], LOW);
  }
  else
  {
    analogWrite(pins_motor3[0], BRAKE_MOTOR3);
    analogWrite(pins_motor3[1], BRAKE_MOTOR3);
  }
#endif // End MIX_TANK_MOTOR2_3
}
 
