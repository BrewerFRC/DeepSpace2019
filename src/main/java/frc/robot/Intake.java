package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * A class to control the ball intake in the 2019 robotics season.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Cooper Parlee
 */

public class Intake {
    private AnalogInput irInput = new AnalogInput(Constants.ANA_IR_SENSOR);
    private Slider slider;

    private final float maxPower = 1f;
    private final float loadingPower = 0.7f;
    private final float ejectPower = -1.0f;
    private final float softEjectPower = -0.45f;
    private final float ballLoadedInches = 5f;
    private final float loadedHoldPower = 0.25f;
    private final int loadCycles = 5;
    private final int ejectCycles = 8;

    private enum CargoStates {
        EMPTY,
        LOADING,
        LOADED,
        EJECT,
        SOFT_EJECT
    }
    private CargoStates cargoState = CargoStates.EMPTY;
    
    private double infaredPreviousReading = 0;
    private double cycles = 0;

    Spark ballIntakeMotor;
    
    public Intake (/*Slider slider*/)
    {
        this.ballIntakeMotor = new Spark(Constants.PWM_INTAKE_MOTOR); // Positive motor power is out, negative is in.
        this.ballIntakeMotor.setInverted(true);
        //this.slider = slider;
    }
    /**
     * Updates the intake and does necessary checks for appropriate state
     */
    public void update ()
    {
        switch(cargoState)
        {
            case LOADING:
                setMotor(loadingPower);

                if(getInfaredCheck()) //Waits for a load and then powers off motors.
                {
                    if(cycles < loadCycles)
                    {
                        cycles++;
                    }
                    else
                    {
                        cycles = 0;
                        cargoState = CargoStates.LOADED;
                        setMotor(loadedHoldPower);
                    }
                    
                }
            break;
            case LOADED:
                setMotor(loadedHoldPower);

                if(!getInfaredCheck()) //Waits for a load and then powers off motors.
                {
                    cargoState = CargoStates.EMPTY;
                    setMotor(0.0f);
                }
            break;
            case EMPTY:
                setMotor(0.0f);
            break;
            case EJECT:
                setMotor(ejectPower);

                if(!getInfaredCheck())
                {
                    if(cycles < ejectCycles){
                        cycles++;
                    }
                    else {
                        cargoState = CargoStates.EMPTY;
                        cycles = 0;
                    }
                }
            break;
            case SOFT_EJECT:
                setMotor(softEjectPower);

                if(!getInfaredCheck())
                {
                    if(cycles < ejectCycles){
                        cycles++;
                    }
                    else {
                        cargoState = CargoStates.EMPTY;
                        cycles = 0;
                    }
                    
                }
            break;
        }
        debug();                    
    }

    public void debug(){
        SmartDashboard.putString("Current state", getState().toString());
        SmartDashboard.putNumber("Current inches", getInfaredInches());
    }
    /**
     * Sets the motor power.
     * @param power the power you want to set the motor to.
     */
    private void setMotor(float power)
    {
        power = Math.max(Math.min(power, maxPower), -maxPower);

        ballIntakeMotor.set(power);
    }

    /**
     * Returns the current state of the ball intake.
     * @return enumerable state for ball intake.
     */
    public CargoStates getState()
    {
        return cargoState;
    }
    
    /**
     * Returns the in inches value of the infared sensor.
     * @return inches.
     */
    public double getInfaredInches()
    {
        double reading = irInput.getValue() / 4 * 0.1 + infaredPreviousReading * 0.9;
        double inches = (-20.0 / 575.0) * reading + 20;

        if (inches < 0){
          inches = 0;
        }

        infaredPreviousReading = reading;
        return inches;
    }
    /**
     * Checks if the infared sensor is within the range to be loaded with a ball.
     * @return boolean for whether or not the intake is loaded.
     */
    public boolean getInfaredCheck()
    {
        if(getInfaredInches() <= ballLoadedInches)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * Returns the raw value of the infared sensor.
     * @return the current value of the infared sensor.
     */
    public double getInfaredRaw()
    {
        return irInput.getValue();
    }
    
    /**
     * Moves the ball intake into loading mode.
     */
    public void toggleLoading (){
        if (cargoState == CargoStates.EMPTY){
            cargoState = CargoStates.LOADING;
        } 
        else if(cargoState == CargoStates.LOADING){
            cargoState = CargoStates.EMPTY;
        }
    }
    /**
     * Soft ejects the ball if within the loaded state.
     */
    public void doSoftEject(){
        if(cargoState == CargoStates.LOADED){
            cargoState = CargoStates.SOFT_EJECT;
        }
    }
    /**
     * Ejects the ball if within the loaded state.
     */
    public void doEject (){
        if(cargoState == CargoStates.LOADED){
            cargoState = CargoStates.EJECT;
        }
    }
}