package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Servo;

/**
 * A class to control a finger, the slider that finger is mounted on and associated sensors.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Sam Woodward
 * @author Brent Roberts
 */
public class Slider {

    private Servo lServo = new Servo(Constants.PWM_SERVO_LEFT);
    private Servo rServo = new Servo(Constants.PWM_SERVO_RIGHT);
    private AnalogInput pot = new AnalogInput(Constants.ANA_POT_SLIDER);
    private Spark motor = new Spark(Constants.PWM_SLIDER_MOTOR);
    private DigitalInput fingerSwitch = new DigitalInput(Constants.DIO_FINGER_SWITCH);
    private DigitalInput leftSwitch = new DigitalInput(Constants.DIO_SLIDER_LEFT_LIMIT);
    private DigitalInput rightSwitch = new DigitalInput(Constants.DIO_SLIDER_RIGHT_LIMIT); 

    public static final int
        POT_LEFT_LIMIT = 235,
        POT_RIGHT_LIMIT = 1237,
        POT_CENTER = 741;
    public static final double
        INCH_LEFT_LIMIT = -3.2,
        INCH_RIGHT_LIMIT = 3.2,
        INCH_CENTER = 0,
        FACTOR = (INCH_LEFT_LIMIT - INCH_RIGHT_LIMIT) / (POT_LEFT_LIMIT - POT_RIGHT_LIMIT),
        MOTOR_POWER = 1d,
        ALLOWANCE = 0.1d,
        P = 0.5;
    public static final boolean
        INVERT_MOTOR = true,
        INVERT_TARGET = false;

    private double targetInches = 0.0d,
        power = 0;

    public enum states {
        MOVING,
        SEARCHING
    }
    private double previousPosition = 0;
    private boolean fingerSearchright = true, isComplete = false, halfComplete = false;
    private states sliderState = states.MOVING;
    

    /**
	 * Returns the raw potentiometer reading for the slider potentiometer.
	 * 
	 * @return - the raw sensor value between 0 and 4096
	 */
    public double currentPotReading() {
        return pot.getValue();
    }


    public double potInches() {
        double inches = (POT_CENTER - currentPotReading()) * -FACTOR;
        if (INVERT_TARGET == true) {
            inches = -inches;
        }
        return inches;
    } 

    /**
     * Internal function to move to position, ignores state.
     * 
     * @param inches The target to move to
     */
    private void setTarget(double inches) {
        isComplete = false;
        if (inches > INCH_RIGHT_LIMIT) { 
            targetInches = INCH_RIGHT_LIMIT;
        } else if (inches < INCH_LEFT_LIMIT) {
            targetInches = INCH_LEFT_LIMIT;
        } else {
            targetInches = inches;
        }
    }

    /**
     * Function to move to target position that requires the slider to be in moving state.
     * 
     * @param inches The target to move to.
     */
    public void moveTo(double inches) {
        if (sliderState == states.MOVING) {
           setTarget(inches);
        }
    }

    /**
     * Returns if the finger limit Switch is pressed.
     * 
     * @return If the finger limit switch is pressed.
     */
    public boolean fingerPressed() {
        return fingerSwitch.get();
    }

    /**
     * Returns true if either of the intake limit switches are pressed.
     * 
     * @return Whether either of the front finger limit switches are pressed.
     */
    public boolean pressed() {
        return (leftSwitch.get() || rightSwitch.get());
    }



    public void fingerUp() {
        lServo.set(0.34);
        rServo.set(0.0);
    }

    public void fingerDown() {
        // was 
        lServo.set(0.03);
        //rServo.set(0.39)
        lServo.set(0.05);
        rServo.set(0.3);
    }

    private void forcePower(double power) {
        motor.set(power);
    }

    
    private void startFingerSearch(boolean right) {
        halfComplete = false;
        fingerSearchright = right;
        if (right) {
            setTarget(INCH_RIGHT_LIMIT);
        } else {
            setTarget(INCH_LEFT_LIMIT);
        }
        sliderState = states.SEARCHING;
    }

    /**
     * Starts a finger search by going right.
     */
    public void startRightFingerSearch() {
        startFingerSearch(true);
    }

    /**
     * Starts a finger search if the side limit switchs are pressed.
     */
    public void rightSearchIfPressed() {
        if (pressed()) {
            startRightFingerSearch();
        }
    }

    /**
     * Starts a finger search by going left
     */
    public void startLeftFingerSearch() {
        startFingerSearch(false);
    }

    /**
     * Starts a finger search if the side limit switchs are pressed.
     */
    public void leftSearchIfPressed() {
        if (pressed()) {
            startLeftFingerSearch();
        }
    }

    /**
     * Returns whether the slider is with allowance of it's target.
     * 
     * @return Whether the slider is within allowance of it's target.
     */
    public boolean isComplete() {
        return isComplete;
    }

    /**
     * Moves the arm, should only be called in update
     */
    private void move() {
        double error = targetInches - potInches();
        /*if (Common.between(potInches(), targetInches - ALLOWANCE, targetInches + ALLOWANCE)) {
		    power = 0;
        } else { 
            if (targetInches > potInches()) {
			    power = MOTOR_POWER;
            } else {
			    power = - MOTOR_POWER;
            }
            if (INVERT_MOTOR) {
                power = - power;
            }
        }*/
        if (Math.abs(error) < ALLOWANCE) {
            isComplete = true;
		    power = 0;
        } else {
            if (targetInches > potInches()) {
			    power = P*error;
            } else {
			    power = - (P*error);
            } 
        }
        motor.set(power);
    }

    public void update() {
        switch (sliderState) {
            case MOVING:
                move();
                break;
            case SEARCHING:
                if (this.pressed() && !fingerPressed()) {
                    fingerUp();
                }
                if (isComplete() || previousPosition == potInches()) {
                    if (halfComplete) {
                        sliderState = states.MOVING;
                        Common.debug("Finger search completed empty");
                    } else {
                        halfComplete = true;
                        if (fingerSearchright) {
                            setTarget(INCH_LEFT_LIMIT);
                        } else {
                            setTarget(INCH_RIGHT_LIMIT);
                        }
                    }
                }
                break;
        }
        // debug necessary?
        Common.dashStr("Slider state", sliderState.toString());
        Common.dashNum("Slider power", power);
        Common.dashNum("Slider Target Inches", targetInches);
        Common.dashNum("Slider Inches", potInches());
        Common.dashNum("Current Pot Reading", currentPotReading());
        previousPosition = potInches();
    }
}


