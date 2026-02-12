
//*********************************************************************************************************************
// Servo setup
//*********************************************************************************************************************
Servo servo[SERVO_CHANNELS]; // Class driver

void servo_setup()
{
#if defined(SERVO_12CH) || defined(SERVO_10CH_MOTOR3) || defined(SERVO_8CH_MOTOR2_3)
  for (byte i = 0; i < SERVO_CHANNELS; i++)
  {
    servo[i].attach(pins_servo[i]);
  }
#endif
}

//*********************************************************************************************************************
// Servo control
//*********************************************************************************************************************
void servo_control()
{
#if defined(SERVO_12CH) || defined(SERVO_10CH_MOTOR3) || defined(SERVO_8CH_MOTOR2_3)
  for (byte i = 0; i < SERVO_CHANNELS; i++)
  {
    servo[i].writeMicroseconds(rc_packet[i]);
  }
#endif
}

//*********************************************************************************************************************
// Fail-safe center/neutral position of servo and motor 1500
//*********************************************************************************************************************
void fail_safe()
{
  for (byte i = 0; i < rc_channels; i++)
  {
    rc_packet[i] = 1500;
  }
}
 
