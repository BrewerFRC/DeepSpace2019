package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to control the floor hatch retrieval system in the 2019 robotics season.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Cooper Parlee
 */

public class HatchIntake {
    private AnalogInput hatchPickupPotentiometer;
    private Spark hatchPickupMotor;
    private Elevator elevator;

    private final float MAX_POWER = 0.6f;
    private final int ARM_EXTEND = 3226;
    private final int ARM_HATCH_RETRIEVE = 1638;
    private final int ARM_STOW = 628;//Was 500
    private final int ARM_SAFE_ERROR = 25;
    private final float ELEVATOR_SAFE_HEIGHT = 15; // The number of inches necessary for the floor intake to be considered safe to clear the elevator arm. 
    private final float ARM_MIN_SAFE_DEGREE = -15;
    private final float P = 0.00275f;

    private boolean moveComplete = false;
    private float target = 0;
    private double moveTime = 0;

    public enum HatchPickupStates {
        EXTEND,
        RETRIEVE,
        STOW
    }
    private HatchPickupStates hatchPickupState = HatchPickupStates.STOW;

    public HatchIntake(Elevator elevator)
    {
        hatchPickupPotentiometer = new AnalogInput(Constants.ANA_FLOOR_POT);
        hatchPickupMotor = new Spark(Constants.PWM_FLOOR_PICKUP);
        hatchPickupMotor.setInverted(true);
        this.elevator = elevator;
    }

    public void update ()
    {
        double power = 0;
        switch (hatchPickupState){
            case EXTEND:
                target = ARM_EXTEND;
            break;
            case RETRIEVE:
                target = ARM_HATCH_RETRIEVE;
                if (Common.time() > moveTime) {
                    moveComplete = true;
                }
            break;
            case STOW:
                target = ARM_STOW;
            break;
        }
        moveComplete = false;

        double error = target - getPotentiometerRaw();

        if(Math.abs(error) <= ARM_SAFE_ERROR){
            power = 0; // If our error is within an acceptable range, don't do anything.
            moveComplete = true;
            if (hatchPickupState == HatchPickupStates.EXTEND) {

            }
        }
        else if(!isSafe()){
            power = 0;
        }
        else {
            power = error * P;
        }
        
        setMotor(power);
        //Common.dashBool("Hatch pickup iscomplete", isComplete());
        Common.dashStr("Hatch Intake State", hatchPickupState.toString());
        //Common.dashNum("Hatch Intake Error", error);
        Common.dashNum("Hatch Intake Target", target);
        SmartDashboard.putNumber("Hatch Potentiometer raw", getPotentiometerRaw());
        SmartDashboard.putNumber("Floor pickup power", power);
    }

    public void debug()
    {
        SmartDashboard.putNumber("Hatch Potentiometer raw", getPotentiometerRaw());
    }

    public boolean isSafe()
    {
        if(this.elevator.getInches() > Robot.ELE_HATCH_PICKUP-1){
            return true;
        }
        return false;
    }

    public boolean isComplete()
    {
        return moveComplete;
    }
    /**
     * Sets the hatch pickup motor power.
     * @param power sets the power of the motor constrained to MAX_POWER variable.
     */
    public void setMotor(double power)
    {
        power = Math.max(Math.min(MAX_POWER, power), -MAX_POWER);
        hatchPickupMotor.set(power);
        Common.dashNum("Hatch floor pickup power", power);
    }
    /**
     * Gets the raw value of the hatch pickup potentiometer.
     * @return the raw value of the potententiometer
     */
    public double getPotentiometerRaw()
    {
        return hatchPickupPotentiometer.getValue();
    }
    public HatchPickupStates getHatchState()
    {
        return hatchPickupState;
    }

    public void doHatchTransfer ()
    {
        if(isSafe())
        {
            moveComplete = false;
            moveTime = Common.time() + 750;
            hatchPickupState = HatchPickupStates.RETRIEVE;
        }
    }

    public void doStow ()
    {
        if(isSafe())
        {
            //moveTime = Common.time() +750;
            hatchPickupState = HatchPickupStates.STOW;
        }        
    }

    public void doPickup ()
    {
        if(isSafe())
        {
            hatchPickupState = HatchPickupStates.EXTEND;
        }
    }
}