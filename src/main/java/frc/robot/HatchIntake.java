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
    private PID pid;

    private final float MAX_POWER = 0.2f;
    private final int ARM_EXTEND = 3188;
    private final int ARM_HATCH_RETRIEVE = 1638;
    private final int ARM_STOW = 500;
    private final float P = 0.0f, I = 0.0f, D = 0.0f;

    private enum HatchPickupStates {
        EXTEND,
        RETRIEVE,
        STOW
    }
    private HatchPickupStates HatchPickupState = HatchPickupStates.STOW;

    public HatchIntake()
    {
        hatchPickupPotentiometer = new AnalogInput(Constants.ANA_FLOOR_POT);
        hatchPickupMotor = new Spark(Constants.PWM_FLOOR_PICKUP);
        pid = new PID(P, I, D, false, false, "Hatch Pickup PID");
    }

    public void update ()
    {
        pid.update();
        double target = 0;
        switch (HatchPickupState){
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
        double error = target - getPotentiometerRaw();
        double power = pid.calc(error);

        SmartDashboard.putNumber("Floor pickup power", power);
    }

    public void debug()
    {
        SmartDashboard.putNumber("Potentiometer raw", getPotentiometerRaw());
    }
    /**
     * Sets the hatch pickup motor power.
     * @param power sets the power of the motor constrained to MAX_POWER variable.
     */
    public void setMotor(double power)
    {
        power = Math.max(Math.min(MAX_POWER, power), -MAX_POWER);
        hatchPickupMotor.set(power);
        SmartDashboard.putNumber("Hatch power! ", power);
    }
    /**
     * Gets the raw value of the hatch pickup potentiometer.
     * @return the raw value of the potententiometer
     */
    public double getPotentiometerRaw()
    {
        return hatchPickupPotentiometer.getValue();
    }
}