package frc.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;

/**
 * A class to control the elevator
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Brent Roberts
 * @author Evan McCoy
 */
public class Elevator {
	//Intake intake;
	Victor elevatorLeft = new Victor(Constants.ELEVATOR_LEFT);
	Victor elevatorRight = new Victor(Constants.ELEVATOR_RIGHT);
	private Encoder encoder; 
	//true = pressed
	private DigitalInput lowerLimit = new DigitalInput(Constants.LOWER_LIMIT);
	//false = pressed
	private DigitalInput upperLimit = new DigitalInput(Constants.UPPER_LIMIT);
	//Elevator height in inches(random value
	public final double COUNTS_PER_INCH = 7414/65.5, 
			//Absolute elevator travel is 66.75 inches
			ELEVATOR_HEIGHT = 66.5,
			//The height the elevator should be positioned at to drop in the switch.
			SWITCH_HEIGHT = 25,
			//How close to the targetHeight that elevator can be to complete
			ACCEPTABLE_ERROR = 1.0,
			//The location of the upper limit switch in inches
			UPPER_LIMIT_POINT = 57.5,
			//The maximum power that the elevator can be run at upward
			MAX_UP_POWER = 1.0,
			MAX_DOWN_POWER = -0.9,
			//The minimum power that the elevator can be run at upward
			MIN_UP_POWER = 0.12,
			MIN_DOWN_POWER = -0.02,
			//The maximum power change
			MAX_DELTA_POWER = 0.1,
			//In inches per second, for velocity ramping
			MIN_VELOCITY = 0.5,
			//In inches per second, for position PID
			MAX_POS_VELOCITY = 45,
			//Maximum velocity while using the joystick
			MAX_J_VELOCITY = 45,
			//For encoder test function, minimum values to move the robot in different directions
			ENCODER_MIN_UP = 0.15, ENCODER_MIN_DOWN = -0.12,
			//For Velocity ramping
			DANGER_VEL_ZONE = 20;
	
	//Reduced speed zone at upper and lower limits in inches.
	final int DANGER_ZONE = 18;
	
	double velocity = 0.0,
			//The last power that was set
			lastPower = 0,
			//-1 is not moving, 0 or greater is moving
			moveCheck = -1,
			//The previous counts of the encoder 
			previousCounts = 0.0;
	long startTime;
	
	PositionByVelocityPID pid = new PositionByVelocityPID(0, ELEVATOR_HEIGHT, -MAX_POS_VELOCITY, MAX_POS_VELOCITY, MAX_DOWN_POWER, MAX_UP_POWER, 0, "Elevator PID");
	double velP = 0.002, velI = 0.0, velD = 0.0;
	double posP = 5, posI = 0.0, posD = 0.0;
	
	public enum States {
		STOPPED, //The state that elevator starts, does nothing unless the home function is run.
		HOMING,  //Brings elevator slowly to the bottom most position possible, sets the offset. Must be done before any use of elevator.
		HOLDING, //State of the elevator holding position, both types of elevator usage can be used from this state.
		MOVING, //State of elevator moving to a target position within the ACCEPTABLE_ERROR.
		JOYSTICK, //Moves the elevator by a desired power returns to IDLE after setting the power once.
		START; //Executes at the beginning of auto.
	}
	States state = States.STOPPED;
	
	public Elevator(/*Intake intake*/) {
		//this.intake = intake;
		/*elevatorRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		elevatorRight.configVelocityMeasurementWindow(8, 0);//defaults to 64, rolling average sample size
		//defaults to 100 Ms, the time of the sample that the current sample is compare to, changes the units
		elevatorRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 0);*/
		encoder = new Encoder(Constants.ELEVATOR_ENCODER_A, Constants.ELEVATOR_ENCODER_B, true, EncodingType.k1X);
		elevatorLeft.setInverted(true);
		pid.setVelocityScalars(velP, velI, velD);
		pid.setVelocityInverted(true);
		pid.setPositionScalars(posP, posI, posD);
		pid.setPositionInverted(true);
		
		Thread t = new Thread(new UpperLimitTask()); // This starts the new thread for the magnetic sensor.
		t.start();
	}
	
	/**
	 * A safe function to set the power of the elevator, cannot exceed MAX_POWER
	 * 
	 * @param power - power to run the elevator at, + = up and - = down
	 */
	public void setPower(double power) {
		// Check safeties and stop power if necessary
		if (/*!intake.elevatorSafe() && */power < MIN_UP_POWER) {//Don't let elevator drop if intake arm is in a unsafe position
			power = MIN_UP_POWER;
			pid.reset();
		}
		if (power > 0.0) {  //Move up
			if(getInches() >= ELEVATOR_HEIGHT) { //hard limit on expected height
				power = 0.0;
			} else if(!upperLimit.get()) {  //Have we made it to the upper limit trigger point(limit switch false = reached)
				if (getInches() < UPPER_LIMIT_POINT) {  //Make sure encoder has counted enough inches
					power = 0.0;
					Common.debug("Upper Limit fail, HOMING");
					state = States.HOMING;
				}
			} else if(getInches()>= ELEVATOR_HEIGHT-DANGER_ZONE) {
				power = Math.min(power, Common.map(ELEVATOR_HEIGHT-getInches(), 0.0, DANGER_ZONE, MIN_UP_POWER, MAX_UP_POWER));
			} else {
				power = Math.min(power, MAX_UP_POWER);
			}
		} else {  //Moving Down
			if (lowerLimit.get()) { //lower limit true when pressed
				power = 0.0;
			} else {
				if(state != States.HOMING) {
					if (getInches() <= DANGER_ZONE) {
						power = Math.max(power, Common.map(getInches(), 0.0, DANGER_ZONE, MIN_DOWN_POWER, MAX_DOWN_POWER));
					} else {
						power = Math.max(power, MAX_DOWN_POWER);
					}
				}
			}
		}
		power = encoderTest(power);
		lastPower = power;
		//TODO: Re-enable elevator motor power here
		/*elevatorRight.set(power); 
		elevatorLeft.set(power);*/
		Common.dashNum("Elevator power:", power);
	}
	/**
	 * Limits elevator acceleration for safety
	 * 
	 * @param targetPower -The goal power of the function
	 */
	public void accelPower(double targetPower) {
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
		setPower(power);
	}
	
	
	/** 
	 * Test that encoder is registering movement.  Set Power to zero and re-home if not.
	 *
	 * @param power - Current power to be tested.
	 */
	public double encoderTest(double power) {
		if (moveCheck == -1) {  //If not currently testing
			if (power > ENCODER_MIN_UP || power < ENCODER_MIN_DOWN) {  //Is power great enough to move elevator?
				moveCheck = 1;  //Start check counter 
			}
		} else {
			if (power < ENCODER_MIN_UP && power > ENCODER_MIN_DOWN) {  //Is power not great enough to move elevator?
				moveCheck = -1; //Stop checking
			}
			if (moveCheck >= 5) {  //Test encoder movement after 5 cylces
				if (state != States.HOMING) {
					if (getVelocity() == 0) {
						Common.debug("ENCODER FAULT: Velocity is still zero, State = HOMING"+"power:"+power);
						state = States.HOMING;
						power = 0.0;
					} 
					if (getEncoder() == previousCounts) {
						Common.debug("ENCODER FAULT: Count is the same as previous, state = HOMING"+getEncoder()+"power"+power);
						state = States.HOMING;
						power = 0.0;
					}
				}
				moveCheck = 1;  //Restart counter
			} else {
				moveCheck += 1;
			}
		}
		previousCounts = getEncoder();
		return power;
	}
	
	
	/**
	 * Uses a pid to move the robot at a target velocity
	 * 
	 * @param targetVelocity -The target speed for the robot to move in inches per second
	 */
	public void pidVelMove(double targetVelocity) {
		double SAFE_HEIGHT = ELEVATOR_HEIGHT - 0.25;
		if (targetVelocity >= 0.0) {
			if (getInches() >= SAFE_HEIGHT - DANGER_VEL_ZONE) {
				double rampMap = Common.map(SAFE_HEIGHT-getInches(), 0, DANGER_VEL_ZONE, MIN_VELOCITY, MAX_J_VELOCITY);
				targetVelocity = Math.min(targetVelocity, rampMap);
			}
		} else {
			if (getInches() >= DANGER_VEL_ZONE) {
				double rampMap = Common.map(getInches(), 0, DANGER_VEL_ZONE, -MIN_VELOCITY, -MAX_J_VELOCITY);
				targetVelocity = Math.max(targetVelocity, rampMap);
			}
		}
		pid.setTargetVelocity(targetVelocity);
		double pidVelCalc = pid.calcVelocity(getVelocity());
		Common.dashNum("pidVelCalc", pidVelCalc);
		accelPower(pidVelCalc);
	}
	
	/**
	 * Uses a PID to move the robot at the PID target positon.
	 */
	public void pidDisMove() {
		double pidDisCalc = pid.calc(getInches(), getVelocity());
		Common.dashNum("pidDisCalc", pidDisCalc);
		accelPower(pidDisCalc);
	}
	
	/**
	 * Whether or not the intake is safe to move at the current elevator height.
	 * 
	 * @return - True when it is safe to move the intake over the back
	 */
	public boolean intakeSafe() {
		if (getInches() >= ELEVATOR_HEIGHT-1.4) {
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Gets if the elevator is at the top
	 * 
	 * @return - true = at top
	 */
	public boolean upperLimitSafe() {
		//greater or equal to total height
		if (!upperLimit.get() &&  getInches() < UPPER_LIMIT_POINT) {
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Gets if the elevator is at the top
	 * 
	 * @return - true = at bottom
	 */
	public boolean atBottom() {
		if (lowerLimit.get()) {
			return true;
		} else {
			return false;
		}
	}
	
	/** Gets the elevator height as a percentage
	 * 
	 * @return -Elevator height as a percentage
	 */
	public double getElevatorPercent() {
		return Common.map(getInches(), 0, ELEVATOR_HEIGHT, 0, 1);
	}
	
	/**
	 * Starts homing the elevator
	 */
	public void home() {
		Common.debug("New state Homing");
		state = States.HOMING;
	}
	
	public void start() {
		state = States.START;
		startTime = Common.time();
	}
	
	/**
	 * Gets the state of the elevator.
	 * 
	 * @return -The current state of the elevator.
	 */
	public States getState() {
		return state;
	}
	/**
	 * Returns passed state in a human-readable way.
	 * @param enumerableState The current state as an enumerable.
	 * @return The current state as a string.
	 */
	public String getStateReadable(States enumerableState){
		String stateReadable;
		switch (enumerableState){
			case STOPPED:
				stateReadable = "Stopped";
				break;
			case HOMING:
				stateReadable = "Homing";
				break;
			case HOLDING:
				stateReadable = "Holding";
				break;
			case MOVING:
				stateReadable = "Moving";
				break;
			case JOYSTICK: 
				stateReadable = "Joystick";
				break;
			case START:
				stateReadable = "Start";
				break;
			default:
				stateReadable = "Other";
				break;
		}
		return stateReadable;
	}
	
	/**
	 * Resets the offset of the encoder
	 */
	public void resetEncoder() {
		//offset = elevatorRight.getSensorCollection().getPulseWidthPosition();
		encoder.reset();
	}
	
	/**
	 * Gets the raw encoder counts + the offset in counts 
	 * 
	 * @return -The current height of the elevator in counts
	 */
	public double getEncoder() {
		return encoder.get();
	}
	
	public boolean isUpperLimitTriggered(){
		return upperLimit.get();
	}
	/**
	 * Gets the current height of the elevator in inches
	 * 
	 * @return -The current height of the elevator in counts 
	 */
	public double getInches() {
		return getEncoder()/COUNTS_PER_INCH;
	}
	
	/**
	 * The velocity of the elevator.
	 * 
	 * @return - the velocity in inches per second.
	 */
	public double getVelocity() {
		return encoder.getRate()/COUNTS_PER_INCH;
	}
	
	/**
	 * Function designed for joystick control of elevator.
	 * Only works for one cycle.
	 * 
	 * @param jInput -The velocity mapped for 1.0(max down) to -1.0(max up) to move the elevator at
	 */
	public void joystickControl(double jInput) {
		//overrules moveToHeight()
		Common.dashNum("Elevator Joystick", jInput);
		if (state != States.STOPPED && state != States.HOMING){
			if (jInput != 0) {
				double jMap = Common.map(-jInput, -1, 1, -MAX_J_VELOCITY, MAX_J_VELOCITY);
				Common.dashNum("jMap", jMap);
				velocity = jMap;
				//Common.debug("New state Joystick");
				state = States.JOYSTICK;
			}
		}
	}
	
	/**
	 * Starts moving the elevator to a target height
	 * 
	 * @param targetHeight - Height in inches that the elevator to move to
	 */
	public void moveToHeight(double targetHeight) {
		if (state != States.STOPPED && state != States.HOMING && state != States.JOYSTICK) {
			pid.setTargetPosition(targetHeight);
			state = States.MOVING;
		}
	}
	
	/**
	 * Whether or not the elevator is within acceptable range of the position target.
	 * 
	 * @return - complete
	 */
	public boolean isComplete() {
		return Math.abs(pid.getTargetPosition() - getInches()) < ACCEPTABLE_ERROR;
	}
	
	/**
	 * Prints standard debug information about elevator components.
	 */
	public void debug() {
		Common.dashNum("Elevator encoder", getEncoder());
		Common.dashNum("Elevator encoder in inches", getInches());
		Common.dashBool("upper limits safe", upperLimitSafe());
		Common.dashBool("upper Limit Triggered", upperLimit.get());
		Common.dashBool("at bottom", atBottom());
		Common.dashStr("Elevator State", state.toString());
		Common.dashNum("Elevator Velocity", getVelocity());
	}
	
	/**
	 * Update function that should be the last run of all elevator functions.
	 * Exports certain values to smart dashboard and runs the state process of the elevator
	 */
	public void update() {
		pid.update();
		//Common.dashNum("Elevator encoder", getEncoder());
		Common.dashNum("Elevator encoder in inches", getInches());
		Common.dashBool("upper limits safe", upperLimitSafe());
		Common.dashBool("upper Limit Triggered", upperLimit.get());
		Common.dashBool("at bottom", atBottom());
		Common.dashStr("Elevator State", state.toString());
		Common.dashNum("Elevator Velocity", getVelocity());
		switch(state) {
		case STOPPED:
			setPower(0.0);
			break;
		case HOMING:
			if (lowerLimit.get()) {
				resetEncoder();
				setPower(0.0);
				pid.setTargetPosition(0);
				Common.debug("New state Holding");
				state = States.START;
			} else {
				setPower(-0.1);
			}
			break;
		case START:
			moveToHeight(5);
			break;
		case HOLDING:
			pidDisMove();
			break;
		case MOVING:
			if (isComplete()) {
				state = States.HOLDING;
			}
			pidDisMove();
			break;
		case JOYSTICK:
			pidVelMove(velocity);
			//accleMove(speed);
			//Common.debug("new State Idle");
			if (state == States.JOYSTICK){
				state = States.HOLDING;
				pid.setTargetPosition(Math.max(0, Math.min(ELEVATOR_HEIGHT - 0.25, getInches() + 0.13*getVelocity())));
			}
			break;
		}
	}
	
	public class UpperLimitTask implements Runnable {
		//private long previousTime;
		//private int counter = 0;
		public void run() {
			while (true) {
				//counter++;
				if(!upperLimit.get()) {  //Have we made it to the upper limit trigger point(limit switch false = reached)
					if (getInches() < UPPER_LIMIT_POINT) {  //Make sure encoder has counted enough inches
						Common.debug("Upper Limit fail, HOMING");
						state = States.HOMING;
					}
				}
				/*if (counter == 99) {
					long time = Common.time();
					System.out.println("Time: " + (time - previousTime));
					previousTime = time;
					counter = 0;
				}*/
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {}
			}
		}
	}
}