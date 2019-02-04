package frc.robot;

public class Constants{
    public static final int 
        REFRESH_RATE = 20,
        // PWMs 
        PWM_SLIDER_MOTOR = 0,
        PWM_ARM_MOTOR = 1,
        PWM_SERVO_LEFT = 2,
        PWM_SERVO_RIGHT = 3,
        PWM_INTAKE_MOTOR = 4,

        // CANs
        CAN_DRIVE_FL = 0,
        CAN_DRIVE_FR = 1,
        CAN_DRIVE_BL = 2,
        CAN_DRIVE_BR = 3,
        CAN_ELEVATOR_L = 4,
        CAN_ELEVATOR_R = 5,
        CAN_PDP = 6, //Should not be used
        CAN_VRM = 7,

        // DIOs
        DIO_ENCODER_A = 0,
        DIO_ENCODER_B = 1,
        DIO_SLIDER_LEFT_LIMIT = 2,
        DIO_SLIDER_RIGHT_LIMIT = 3,
        DIO_LOWER_LIMIT = 4,
        DIO_UPPER_LIMIT = 5,
        //whatever the sensor te finger uses = 6
        // ANALOGS
        ANA_POT_SLIDER = 0,
        ANA_ARM_POT = 1,
        ANA_IR_SENSOR = 2;
}