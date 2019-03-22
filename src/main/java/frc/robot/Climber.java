package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/**
 * A class to control the Climber in the 2019 season
 * 
 * @author FRC Team 4564 Brewer robotics
 * @author Samuel "Woodie" Woodward
 * @author Brent Roberts
 */
public class Climber {

    private static final Talon liftMotor =  new Talon(Constants.PWM_LIFT_CLIMBER); //Should move positive is down
    private Spark footMotor = new Spark(Constants.PWM_HOZ_CLIMBER);
    private DigitalInput liftUpperLimit =  new DigitalInput(Constants.DIO_LIMIT_LIFT);
    private DigitalInput hozLimit =  new DigitalInput(Constants.DIO_LIMIT_HOZ);
    private Encoder encoder = new Encoder(Constants.DIO_CLIMB_ENCODE_A, Constants.DIO_CLIMB_ENCODE_B, false, EncodingType.k4X); //0 is highest and descending is increasing

    private Arm  arm;
    private DriveTrain dt;

    private double COUNTS_PER_INCH  = 1;
    private double LIFT_MAX = 20;
    private double LIFT_UP_POWER = .4, LIFT_DOWN_POWER = -.3, LIFT_HOLD_POWER = .2, FOOT_POWER = .3;

    public enum climberStates {
        STOPPED, //Starting state
        HOMING, //Homing UP to 0 the encoder
        READY, //Idle and ready to climb
        LIFTING, //Moving up to either level 2 or 3 height to climb
        SLIDING, //Moving foot forward.
        RETRACTING, //Moving the foot up to a safe height
        COMPLETE //Holding powers

    }

    public climberStates state = climberStates.STOPPED;

    public Climber(Arm arm, DriveTrain dt) {
        this.arm = arm;
        this.dt = dt;
    }

    //Getter functions
    /**
     * Returns if the lift is at the upper limit.
     * 
     * @return If the lift is at the upper limit.
     */
    public boolean atTop() {
        return liftUpperLimit.get();
    }

    /**
     * Returns the raw encoder counts.
     * 
     * @return The raw encoder counts.
     */
    public double getCounts() {
        return encoder.get();
    }

    /**
     * Returns the height in inches.
     * 0 is with the lift stowed/homed.
     * 
     * @return The lift height in inches.
     */
    public double getInches() {
        return getCounts()/COUNTS_PER_INCH;
    }

    //Setter functions

    //Power set functions
    /**
     * Moves the lift up until it reaches it's limit at the same power.
     * Will hold at limit.
     */
    private void liftUp() {
        if (!atTop()) {
            liftMotor.set(LIFT_UP_POWER);
        } else {
            liftMotor.set(LIFT_HOLD_POWER);
        }
    }

    /**
     * Moves the lift down until it reaches the bottom limit.
     * Will hold at limit.
     */
    private void liftDown() {
        if (getInches() <= LIFT_MAX) {
            liftMotor.set(LIFT_HOLD_POWER);
        } else {
            liftMotor.set(LIFT_DOWN_POWER);
        }
    }

    private void footDrive() {
        if (Footlimit not pressed) {
            footMotor.set(FOOT_POWER);
        }
    }
    
    /**
     * Sets all motors to zero.
     */
    public void EmergancyStop() {
        liftMotor.set(0);
        footMotor.set(0);
    }

    //Update
}