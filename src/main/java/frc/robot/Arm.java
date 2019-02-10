package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;;

/**
 * A class to control a 4 bar arm in the 2019 robotics season
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Brent Roberts
 */
public class Arm {
	private Slider slider;
	private Elevator elevator;
    private static final Spark 
                    intakeArm =  new Spark(Constants.PWM_ARM_MOTOR),
                    intake =  new Spark(Constants.PWM_INTAKE_MOTOR);
	private AnalogInput pot =  new AnalogInput(Constants.ANA_ARM_POT);
	//private DigitalInput leftSwitch;
	//private DigitalInput rightSwitch;
    private PositionByVelocityPID pid;

    public final double MIN_ELEVATOR_SAFE = 0,//Safe angles when elevator is not at top
    //The angle at which the intake is horizontal out the front.
	HORIZONTAL_POSITION = 100,//The arm's position at 0 degrees/parallel to floor.
   // MIN_POSITION = 210, MAX_POSITION = 3593, 
    MIN_ANGLE = -12, MAX_ANGLE = 160, 
    MIN_ABS_ANGLE = -45, //To be determined
    //The degrees that the power ramping takes place in at the limits
    DANGER_ZONE = 25,
    //Down powers
    MIN_DOWN_POWER = 0, MAX_DOWN_POWER = -0.1,
    //up powers
    MIN_UP_POWER = 0, MAX_UP_POWER = 0.5,
    //Max power change in accel limit
    MAX_DELTA_POWER = 0.1,
    MAX_VELOCITY = 10,
    COUNTS_PER_DEGREE = 14.89444444,
    //PARTIALLY_LOADED_DISTANCE = 10,
    //maximum IR distance a fully loaded cube can be
    //FULLY_LOADED_DISTANCE = 3,
    //How close to the targetHeight that elevator can be to complete
    ACCEPTABLE_ERROR = 2.0;

    //Ramp constants
    //final double MIDDLE_POWER = 0.7;
	//final double MAX_POWER = 1.0;
	//final double MIN_POWER = 0.0;
    //final double LOW_POWER = 0.08;
    
    private double P_POS = 0, I_POS = 0, D_POS = 0,
			P_VEL = 0.000, I_VEL = 0, D_VEL = 0.0, G = 0,
			lastPower = 0, previousReading = 0;
	
	private long intakeTime = 0;
	public boolean loading = false;
	public double velocity = 0.0, lastVelocityTarget = 0;
	public double targetVelocity = 0.0;
	public double position = 0;
	private long previousMillis = Common.time();
	private boolean previousIntakeSafe = false;
	private double previousPosition;


    public enum States {
		STOPPED, //The state that arm starts, does nothing unless the home function is run.
		HOLDING, //State of the arm holding position, both types of arm usage can be used from this state.
		MOVING, //State of arm  moving to a target position within the ACCEPTABLE_ERROR.
		JOYSTICK; //Moves the arm by a desired power returns to IDLE after setting the power once.
    }
    public States state = States.HOLDING;

    public Arm(Elevator elevator) {
		this.elevator = elevator;
        slider =  new Slider();
		pid = new PositionByVelocityPID(MIN_ANGLE, MAX_ANGLE, -MAX_VELOCITY, MAX_VELOCITY, 0.01, "intake");
		pid.setPositionScalars(P_POS, I_POS, D_POS);
		pid.setVelocityScalars(P_VEL, I_VEL, D_VEL);
		pid.setVelocityInverted(true);
		pid.setPositionInverted(true);
		pid.setTargetPosition(45);
		//Thread t = new Thread(new PotUpdate());
		//t.start();
		Common.dashNum("G", G);
	}

/**
	 * Sets the power of the intake arm.
	 * 
	 * @param power - the power
	 */
	public void setArmPower(double power) {
		double minAngle = getMinAngle();
		if (power > 0.0) {
			if (getPosition() >= MAX_ANGLE) {
				power = 0.0;
				pid.reset();
			}
		} else {
			if (getPosition() <= minAngle) { 
				power = 0.0;
				pid.reset();
			}
        }
        power += gTerm();
        power = rampPower(power);
        //intakeArm.setPower(power);
		Common.dashNum(" Power", power);
		Common.dashNum("Arm Last Power", lastPower);
		lastPower = power;
	}
	
	private void setAccelArmPower(double targetPower) {
		double power = 0; 	
		if (Math.abs(lastPower - targetPower) > MAX_DELTA_POWER) {
			if (lastPower > targetPower) {
				power = lastPower - MAX_DELTA_POWER;
			} else {
				power = lastPower + MAX_DELTA_POWER;
			}
		} else {
			power = targetPower;
		}
		setArmPower(power);
	}
	
	public double rampPower(double power) {
		
		double maxPower = 0.0;
		double minPower = 0.0;
        if (power > 0) {
			if(getPosition() >= MAX_ANGLE - DANGER_ZONE) {
            	maxPower  = Common.map(getPosition(), MAX_ANGLE-DANGER_ZONE, MAX_ANGLE, MAX_UP_POWER, MIN_UP_POWER);
				power = Math.min(power, maxPower);
			}
        } else {
			if (getPosition() <= getMinAngle()+DANGER_ZONE) {
				minPower = Common.map(getPosition(), getMinAngle()+DANGER_ZONE, getMinAngle(), MAX_DOWN_POWER, MIN_DOWN_POWER);
				power = Math.max(power, minPower);
			}
        }
		
		return power;
    }
    /**
	 * Gets the arm position in degrees.
	 * 
	 * @return - the position
	 */
	public double getPosition() {
		//return position;
		previousPosition = position;
		position = (getRawPosition() - HORIZONTAL_POSITION) / COUNTS_PER_DEGREE; //210 is the lowest potentiometer reading when arm is fully down
		return position;
	}
	
	/**
	 * Returns the raw sensor reading for the arm potentiometer.
	 * 
	 * @return - the raw sensor value between 0 and 4096
	 */
	public int getRawPosition() {
		return pot.getValue();
	}
	
	/**
	 * Gets the velocity of the arm in degrees per second.
	 * Uses a complementary function to smooth velocity.
	 * 
	 * @return -the velocity in degrees per second
	 */
	public double getVelocity() {
		velocity = (position - previousPosition)/(1/Constants.REFRESH_RATE);
		if (Double.isNaN(velocity)) {
			Common.debug("Velocity NaN"+ velocity);
			velocity = 0;
		}
		return velocity;
	}
	
	/**
	 * PID controlled move to a defined position.
	 * 
	 * @param position - the position in degrees
	 */
	public void movePosition(double position) {
		pid.setTargetPosition(position);
		state = States.MOVING;
	}
	
	/**
	 * Uses a PID to move the robot at the PID target positon.
	 */
	public void pidPosMove() {
		double pidPosCalc = pid.calc(getPosition(), getVelocity());
        Common.dashNum("pidPosCalc for intake arm", pidPosCalc);
        //Got rid of G term because baked into arm.
		setAccelArmPower(pidPosCalc);
	}
	
	/**
	 * PID controlled move at a defined velocity.
	 * 
	 * @param velocity - the velocity in degrees/second.
	 */
	public void moveVelocity(double velocity) {
		double minAngle = getMinAngle();
		double calc = 0;
		//If velocity changes direction, reset pid to speed up response.
		if ((lastVelocityTarget > 0 && velocity < 0) || (lastVelocityTarget < 0 && velocity > 0)) {
			pid.resetVelocityPID();
		}
		if (getPosition() < minAngle && velocity > 0) {
			pid.setTargetVelocity(0.0);
		}
		else {
			pid.setTargetVelocity(velocity);
		}
		if (velocity >= 0.0) {
            //Got rid of G term because it is now baked in.
			calc = pid.calcVelocity(getVelocity());
		}
		else {
			calc = pid.calcVelocity(getVelocity());
		}
		setAccelArmPower(calc);
		lastVelocityTarget = velocity;
	}
	
	/**
	 * Whether or not the elevator can move in the current arm position.
	 * 
	 * @return - safe
	 */
	public boolean elevatorSafe() {
		if (getPosition() > MIN_ELEVATOR_SAFE) {
			return true;
		}
		return false;
	}
	
	public void reset() {
		state = States.STOPPED;
		pid.reset();
	}
	
	/**
	 * Function designed for joystick control of intake arm.
	 * Only works for one cycle.
	 * 
	 * @param jInput -The velocity mapped for 1.0(max down) to -1.0(max up) to move the elevator at
	 */
	public void joystickControl(double jInput) {
		//overrules movePosition()
		if (jInput != 0) {
			double jMap = Common.map(-jInput, -1, 1, -MAX_VELOCITY, MAX_VELOCITY);
			Common.dashNum("jMap intake", jMap);
			targetVelocity = jMap;
			//Common.debug("New intake state Joystick");
			state = States.JOYSTICK;
		}
	}
	
	public double getMinAngle() {
		if (elevator.intakeSafe()) {  //Is elevator
			Common.dashBool("MIN_ANGLE", true);
			return MIN_ANGLE;
		} 
		else {
			Common.dashBool("MIN_ANGLE", false);
			return MIN_ELEVATOR_SAFE;
		}
    }
    
    public double gTerm() {
        double gTerm = G*Math.cos(Math.toRadians(getPosition()));
        return gTerm;
    }

    /**
     * Returns the slider.
     * 
     * @return The slider.
     */
    public Slider getSlider() {
        return slider;
    }
	
	public double getPositionTarget() {
		return pid.getTargetPosition();
	}


	
	/**
	 * Whether or not the elevator is within acceptable range of the position target.
	 * 
	 * @return - complete
	 */
	public boolean isComplete() {
		return Math.abs(pid.getTargetPosition() - getPosition()) < ACCEPTABLE_ERROR;
	}
	
	public void update() {
		pid.update();
		G = Common.getNum("G");
		switch(state) {
			case STOPPED:
				setAccelArmPower(0.0);
				break;
			case HOLDING:
				if (!previousIntakeSafe && elevator.intakeSafe()) {
					pid.resetVelocityPID();
				}
				previousIntakeSafe = elevator.intakeSafe();
				pidPosMove();
				
				//moveVelocity(0.0);
				break;
			case MOVING:
				if (isComplete()) {
					state = States.HOLDING;
				}
				pidPosMove();
				break;
			case JOYSTICK:
				moveVelocity(targetVelocity);
				//accleMove(speed);
				//Common.debug("new State Idle");
				if (state == States.JOYSTICK){
					state = States.HOLDING;
					pid.setTargetPosition(Math.max(MIN_ANGLE + 1, Math.min(getPosition() + velocity*0.1, MAX_ANGLE-1)));
				}
				break;
		}
	}

	public void dashboard() {
		Common.dashNum("Arm degrees", getPosition());
		Common.dashNum("Arm raw position", getRawPosition());
		Common.dashNum("Arm velocity", getVelocity());
		Common.dashNum("Arm Position Target", getPositionTarget());
		Common.dashNum("Arm Velocity Target", pid.getTargetVelocity());
	}

    /* From last year, if needed.
	public class PotUpdate implements Runnable {

		@Override
		public void run() {
			while (true) {
				double previousPosition = position;
				position = MAX_ABS_ANGLE - (getRawPosition() - 210) / COUNTS_PER_DEGREE; //210 is the lowest potentiometer reading when arm is fully down
				position = 0.05 * position + 0.95 * previousPosition;
				
				double previousVelocity = velocity;
				long millis = Common.time();
				if (millis - previousMillis > 0) {
					velocity = (position - previousPosition) / ((millis - previousMillis) / 1000.0);
					velocity = 0.97 * previousVelocity + 0.03 * velocity;
				} else {
					velocity = 0.0;
				}
				previousMillis = millis;
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {}
			}
		}
		
	}*/

}