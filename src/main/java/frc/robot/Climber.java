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
    private Encoder encoder = new Encoder(Constants.DIO_CLIMB_ENCODE_A, Constants.DIO_CLIMB_ENCODE_B, true, EncodingType.k4X); //0 is highest and descending is increasing

    private Elevator ele;
    private DriveTrain dt;

    private final double COUNTS_PER_INCH  = 227.55555; //Napkin math
    private final double LIFT_MAX = 24; //Actually 25
    private double offGround = 0;
    private final double LIFT_UP_POWER = 1.0, LIFT_DOWN_POWER = -0.4, LIFT_STOW_POWER = -.1, LIFT_HOLD_POWER = 0.2, FOOT_POWER = 1.0/*Was 1.0 */, LIFT_HOME_POWER = -.15;

    private double power = 0, footPower = 0;


    //Rough numbers for heights
    private final double LEVEL_2 = 10, LEVEL_3 = 23;
    private int targetLevel =  3;

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

    public Climber(Elevator ele, DriveTrain dt) {
        this.ele = ele;
        this.dt = dt;
        liftMotor.setInverted(true);
        footMotor.setInverted(true);
    }

    public void init() {
        state = climberStates.HOMING;
    }

    //Getter functions
    /**
     * Returns if the lift is at the upper limit.
     * 
     * @return If the lift is at the upper limit.
     */
    public boolean atLiftLimit() {
        return !liftUpperLimit.get();
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

    public boolean footLimit() {
        return !hozLimit.get();
    }


    //Setter functions

    public void home() {
        Common.debug("Lift Starting home");
        state = climberStates.HOMING;
    }

    public void liftLevel2() {
        if (state == climberStates.READY) {
            Common.debug("Lift Starting level 2 climb");
            targetLevel = 2;
            ele.moveToHeight(Robot.ELE_HIGH_STOW);
            ele.arm.movePosition(Robot.ARM_HIGH_STOW);
            dt.arcadeDrive(0, 0);
            state = climberStates.LIFTING;
        }
    }

    public void liftLevel3() {
        if (state ==  climberStates.READY) {
            Common.debug("Lift Starting level 3 climb");
            targetLevel = 3;
            ele.moveToHeight(Robot.ELE_HIGH_STOW);
            ele.arm.movePosition(Robot.ARM_HIGH_STOW);
            dt.arcadeDrive(0, 0);
            state = climberStates.LIFTING;
        }
    }

    public void emergencyStop() {
        state =  climberStates.STOPPED;
        Common.debug("Lift stopping");
    }

    //Power set functions

    private void setLiftPower(double power) {
        if (power > 0) {
            if (this.getInches() >= this.LIFT_MAX) {
                power = 0;
            }
        } else {
            if (atLiftLimit()) {
                //Common.debug("Limiting to STOW_POWER");
                power = LIFT_STOW_POWER;
            }
        }
        this.power = power;
        liftMotor.set(power);
    }

    /**
     * Moves the lift up until it reaches it's limit at the same power.
     * Will hold at limit.
     */
    public void liftUp() {
       setLiftPower(LIFT_UP_POWER);
          
    }

    public void liftHold() {
        setLiftPower(LIFT_HOLD_POWER);
    }

    public void liftStow() {
        setLiftPower(LIFT_STOW_POWER);
    }

    private void liftHomeSpeed() {
        setLiftPower(LIFT_HOME_POWER);
    }

    /**
     * Moves the lift down until it reaches the bottom limit.
     * Will hold at limit.
     */
    public void liftDown() {
        setLiftPower(LIFT_DOWN_POWER);
    }

    /**
     * Moves the foot.
     * Forward only.
     */
    public void footDrive() {
        if (!footLimit()) {
            //Common.debug("Running foot");
           footMotor.set(FOOT_POWER);
            footPower = FOOT_POWER;
        } else {
            footMotor.set(0);
            footPower = 0;
        }
    }

    public void footHold() {
        footMotor.set(0);
        footPower = 0;
    }
    
    /**
     * Sets all motors to zero.
     */
    private void stopPower() {
        setLiftPower(0);
        footHold();
    }

    
    //Update
    /**
     * Updates the class.
     */
    public void update() {
        /*STOPPED, //Starting state
        HOMING, //Homing UP to 0 the encoder
        READY, //Idle and ready to climb
        LIFTING, //Moving up to either level 2 or 3 height to climb
        SLIDING, //Moving foot forward.
        RETRACTING, //Moving the foot up to a safe height
        COMPLETE //Holding powers*/
        switch (state) {
            case STOPPED :
                stopPower();
                break;
            case HOMING :
                this.liftHomeSpeed();
                footHold();
                if (atLiftLimit()) {
                    Common.debug("Lift moving from HOMING to READY");
                    encoder.reset();
                    state =  climberStates.READY;
                }
                break;
            case READY : 
                this.liftStow();
                break;
            case LIFTING :
                liftUp();
                dt.arcadeDrive(0, 0);
                if (targetLevel == 3) {
                    if (getInches() >= LEVEL_3) {
                        Common.debug("Lift completed LEVEL 3 height, going to SLIDING");
                        state = climberStates.SLIDING;
                    }
                } else if (targetLevel == 2) {
                    if (getInches() >= LEVEL_2) {
                        Common.debug("Lift compledted Level 3 height, goint to SLIDING");
                        state = climberStates.SLIDING;
                    }
                }
                break;
            case SLIDING:
                liftHold();
                footDrive();
                dt.arcadeDrive(-.43, 0);
                if (footLimit()) {
                    Common.debug("Lift completed sliding, now retracting");
                    offGround = getInches() -7;
                    //Common.debug("Offground is: "+offGround);
                    state = climberStates.RETRACTING;
                }
                break;
            case RETRACTING :
                liftDown();
                footHold();
                dt.arcadeDrive(-.3, 0);
		        ele.arm.movePosition(0);
               /* if (getInches() <= offGround) {
                    Common.debug("Lift completed retracting, now complete at: "+getInches());
                    state = climberStates.COMPLETE;
                }*/
                if (atLiftLimit()) {
                    Common.debug("Lift completed retracting, now complete at: "+getInches());
                    state = climberStates.COMPLETE;
                }
                break;
            case COMPLETE :
                //liftStow();
                dt.arcadeDrive(-0.43, 0);
                footHold();
                setLiftPower(0);
                break;
        }

        /*if (atLiftLimit()) {
            encoder.reset();
        }*/
        debug();
    }

    public void debug() {
        Common.dashBool("lift limit", this.atLiftLimit());
        Common.dashBool("foot limit", this.footLimit());
        Common.dashNum("Lift counts", this.getCounts());
        Common.dashNum("Lift inches", this.getInches());
        Common.dashStr("Lift state", this.state.toString());
        Common.dashNum("Lift power", this.power);
        Common.dashNum("Foot power", this.footPower);
        Common.dashNum("Target Level", this.targetLevel);
    }
}
