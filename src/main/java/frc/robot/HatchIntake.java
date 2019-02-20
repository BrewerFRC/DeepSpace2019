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

    private final float MAX_POWER = 0.2f;
    private final int ARM_EXTEND = 3188;
    private final int ARM_HATCH_RETRIEVE = 1638;
    private final int ARM_STOW = 500;
    private final int ARM_SAFE_ERROR = 100;
    private final float ELEVATOR_SAFE_HEIGHT = 15; // The number of inches necessary for the floor intake to be considered safe to clear the elevator arm. 
    private final float ARM_MIN_SAFE_DEGREE = -15;
    private final float P = 0.0005f;

    private boolean moveComplete = false;
    private float target = 0;

    public enum HatchPickupStates {
        EXTEND,
        RETRIEVE,
        STOW
    }
    private HatchPickupStates hatchPickupState = HatchPickupStates.EXTEND;

    public HatchIntake(Elevator elevator)
    {
        hatchPickupPotentiometer = new AnalogInput(Constants.ANA_FLOOR_POT);
        hatchPickupMotor = new Spark(Constants.PWM_FLOOR_PICKUP);
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
        }
        else if(!isSafe()){
            power = 0;
        }
        else {
            power = error * P;
        }

        SmartDashboard.putNumber("Floor pickup power", power);
    }

    public void debug()
    {
        SmartDashboard.putNumber("Potentiometer raw", getPotentiometerRaw());
    }

    public boolean isSafe()
    {
        if(this.elevator.getInches() > Robot.ELE_LOW_STOW){
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
            hatchPickupState = HatchPickupStates.RETRIEVE;
        }
    }

    public void doStow ()
    {
        if(isSafe())
        {
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