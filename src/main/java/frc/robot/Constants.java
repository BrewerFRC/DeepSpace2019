package frc.robot;

public class Constants{
    public static final int 
        REFRESH_RATE = 20,
        // PWMs 
        PWM_SLIDER_MOTOR = 0,
        PWM_ARM_MOTOR = 9,
        PWM_SERVO_LEFT = 2,
        PWM_SERVO_RIGHT = 3,
        PWM_INTAKE_MOTOR = 4,
<<<<<<< HEAD

        // CANs
        CAN_DRIVE_FL = 0,
        CAN_DRIVE_FR = 1,
        CAN_DRIVE_BL = 2,
        CAN_DRIVE_BR = 3,
        CAN_ELEVATOR_L = 4,
        CAN_ELEVATOR_R = 5,
        CAN_PDP = 6, //Should not be used
        CAN_VRM = 7,

        ELEVATOR_LEFT = 0,
        ELEVATOR_RIGHT = 1,

        PWM_SLIDER = 2,
        INTAKE_ARM = 4, // TODO: Declare intake channels for constants
        LEFT_INTAKE_MOTOR = 6,
        RIGHT_INTAKE_MOTOR = 5,

        ELEVATOR_ENCODER_A = 0,
        ELEVATOR_ENCODER_B = 1,
        // DIOs
        LOWER_LIMIT = 4,
        MAG_SWITCH = 5,
        // ANALOGS
        ANA_POT_SLIDER = 0, // TODO: Define this stuff
        INTAKE_POTENTIOMETER = 1

        
        ;
=======
        // DIOs
        DIO_LIFT_ENCODER_A = 0,
        DIO_LIFT_ENCODER_B = 1,
        DIO_SLIDER_LEFT_LIMIT = 2,
        DIO_SLIDER_RIGHT_LIMIT = 3,
        DIO_LOWER_LIMIT = 4,
        DIO_MAG_SWITCH = 5,
        //whatever the sensor te finger uses = 6
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
        ANA_IR_SENSOR = 2;
>>>>>>> 656994a877073bbd041975bb6b1300050a04da2d
}