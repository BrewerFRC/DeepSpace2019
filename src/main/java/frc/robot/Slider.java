package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Servo;


public class Slider {

    private Servo lServo = new Servo(Constants.PWM_SERVO_LEFT);
    private Servo rServo = new Servo(Constants.PWM_SERVO_RIGHT);

    public static final int
    POT_LEFT_LIMIT = 200,
    POT_RIGHT_LIMIT = 3900,
    POT_CENTER = 2048;
    public static final double
    INCH_LEFT_LIMIT = -4,
    INCH_RIGHT_LIMIT = 4,
    INCH_CENTER = 0,
    FACTOR = (INCH_LEFT_LIMIT - INCH_RIGHT_LIMIT) / (POT_LEFT_LIMIT - POT_RIGHT_LIMIT),
    MOTOR_POWER = 0.4d,
    ALLOWANCE = 0.5d;
    public static final boolean
    INVERT_MOTOR = false,
    INVERT_TARGET = false;

    private double targetInches = 0.0d;
    private AnalogInput pot = new AnalogInput(Constants.ANA_POT_SLIDER);
    private Spark motor = new Spark(Constants.PWM_SLIDER_MOTOR);

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

    public void setTarget(double inches) {
        if (inches > INCH_RIGHT_LIMIT) { 
            targetInches = INCH_RIGHT_LIMIT;
        } else if (inches < INCH_LEFT_LIMIT) {
            targetInches = INCH_LEFT_LIMIT;
        } else {
            targetInches = inches;
        }
    }

    public void fingerUp() {
        lServo.set(0.39);
        rServo.set(0.0);
    }

    public void fingerDown() {
        //lServo.set(0.0);
        //rServo.set(0.39)
        lServo.set(0.03);
        rServo.set(0.35);
    }

    public void update() {
        double power;
        if (Common.between(potInches(), targetInches - ALLOWANCE, targetInches + ALLOWANCE)) {
		    power = 0;
        } else { 
            if (targetInches > potInches()) {
			    power = MOTOR_POWER;
            } else {
			    power = - MOTOR_POWER;
            }
        }
	    if (INVERT_MOTOR == true) {
		   power = -power;
        }
        //debug necessary?
        Common.dashNum("Slider power", power);
        Common.dashNum("Slider Target Inches", targetInches);
        Common.dashNum("Slider Inches", potInches());
		motor.set(power);
    }
}


