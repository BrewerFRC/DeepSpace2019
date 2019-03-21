package frc.robot;

public class Constants{
    public static final int 
        REFRESH_RATE = 20,
        // PWMs 
        PWM_SLIDER_MOTOR = 0,
        PWM_SERVO_LEFT = 2,
        PWM_SERVO_RIGHT = 3,
        PWM_INTAKE_MOTOR = 4,
        PWM_FLOOR_PICKUP = 5,
        PWM_HOZ_CLIMBER = 7,
        PWM_LIFT_CLIMBER = 8,
        PWM_ARM_MOTOR = 9,
        // DIOs
        DIO_LIFT_ENCODER_A = 0,
        DIO_LIFT_ENCODER_B = 1,
        DIO_LIMIT_LIFT = 3,
        DIO_LIMIT_HOZ = 2,
        DIO_SLIDER_SIDE_LIMIT = 7,
        DIO_LOWER_LIMIT = 4,
        DIO_MAG_SWITCH = 5,
        DIO_FINGER_SWITCH = 6,
        //CAN
        CAN_DRIVE_BL = 2,
        CAN_DRIVE_FL = 0,
        CAN_DRIVE_BR = 3,
        CAN_DRIVE_FR = 1,

        CAN_LIFT_R = 5,
        CAN_LIFT_L = 4,
        
        // ANALOGS
        ANA_POT_SLIDER = 0,
        ANA_ARM_POT = 1,
        ANA_IR_SENSOR = 2,
        ANA_FLOOR_POT = 3;
}