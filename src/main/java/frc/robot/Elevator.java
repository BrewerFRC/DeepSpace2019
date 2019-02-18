package frc.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * A class to control the elevator
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Brent Roberts
 * @author Evan McCoy
 */
public class Elevator {
	//Intake intake;
	WPI_VictorSPX elevatorLeft = new WPI_VictorSPX(Constants.CAN_LIFT_L);
	WPI_VictorSPX elevatorRight = new WPI_VictorSPX(Constants.CAN_LIFT_R);
	private Encoder encoder; 
	//true = pressed
	private DigitalInput lowerLimit = new DigitalInput(Constants.DIO_LOWER_LIMIT);
	//false = pressed
	private DigitalInput magSwitch = new DigitalInput(Constants.DIO_MAG_SWITCH);
	public Arm arm;
	//Elevator height in inches(random value
	public final double COUNTS_PER_INCH = 7120/62.75, 
		//Absolute elevator travel is 62.75 inches
		ELEVATOR_HEIGHT = 62.75,
		// The distance from absolute height that the elevator is considered safe to.
		SAFETY_MARGIN = 2.0,
		// The maximum height that the robot is allowed
		SAFE_HEIGHT = ELEVATOR_HEIGHT - SAFETY_MARGIN,
		//How close to the targetHeight that elevator can be to complete
		ACCEPTABLE_ERROR = 0.5, 
		//The location of the magnetic switch in inches, just below trigger point
		MAG_SWITCH_POINT = 23.65, //was 23.75 
		//The maximum power that the elevator can be run at upward
		MAX_UP_POWER = .35, // was 1
		MAX_DOWN_POWER = -0.2, // was -0.8 
		//The minimum power that the elevator can be run
		MIN_UP_POWER = 0.11,
		MIN_DOWN_POWER = -0.08,
		GRAVITY_HOLD_POWER = 0.1,
		//The maximum power change; for power curving of PID and Xbox
		MAX_DELTA_POWER = 0.1, 
		//In inches per second, for velocity ramping
		MIN_VELOCITY = 2,
		//In inches per second, for position PID
		MAX_POS_VELOCITY = 45, // Was: 45in/s
		//Maximum velocity while using the joystick
		MAX_J_VELOCITY = 20, // Was: 10 in/s
		//For encoder test function, test is only performed if power is above the minimum. 
		ENCODER_MIN_UP = 0.2, ENCODER_MIN_DOWN = -0.15,
		//The distance from the floor that the arm pivots in inches
		INCHES_FROM_FLOOR = 10.5,
		//How much safe space (in inches) to remove taking into account the bumpers
		BUMPER_OFFSET = 7.5,
		//How much space (in inches) to be used as a buffer in order to prevent collisions
		ARM_ARC_BUFFER = 2,
		//How far, in inches, the bottom of the forbar is from its respective pivot point.
		FORBAR_YEXT = 2,
		//For Velocity ramping
		DANGER_VEL_ZONE = 10,
		//The distance from the floor to the arm pivot on the elevator in inches.
		ARM_PIVOT_TO_FLOOR = 10.5,
		//How much safe space (in inches) to remove taking into account the bumpers
		BUMPER_INCHES_TO_FLOOR = 7.5,
		//Minimum Y distance above bumper or floor to hand 
		Y_HAND_SAFETY = .5,
		//Minimum Y distance above floor.
        Y_HAND_FLOOR_SAFETY = 1.0,  //was 1.5
		//How far, in inches, the bottom of the hand from the pivot of the fourbar.
		Y_HAND_EXT = 10.87,
		//The minimum angle at which the bumper can be cleared.
		MIN_BUMPER_CLEAR_ANGLE = -10, //was -5, -4.5
		MIN_BUMPER_CLEAR_HEIGHT = 4,
		//The length of the arm in inches pivot to pivot.
		ARM_LEN = 19.887,
		PIVOT_TO_BOTTOM = 5.875;
	
	//Reduced speed zone at upper and lower limits in inches.
	final int DANGER_ZONE = 10;

	//Height constants
	public final double LOW_STOW = 27,
	HIGH_STOW = 0;
	
	double velocity = 0.0,
			// Target Height which is feed to Position PID via pidDisMove()
			targetHeight = 0.0,
			//The last power that was set
			lastPower = 0.0,
			//-1 is not moving, 0 or greater is moving
			moveCheck = -1,
			//The previous counts of the encoder 
			previousCounts = 0.0;
	long startTime;
	
	PositionByVelocityPID pid = new PositionByVelocityPID(0, SAFE_HEIGHT, -MAX_POS_VELOCITY, MAX_POS_VELOCITY, MAX_DOWN_POWER, MAX_UP_POWER, 0, "Elevator PID");
	double velP = 0.005, velI = 0.0, velD = 0.0;
	double posP = 5, posI = 0.0, posD = 0.0;
	
	public enum States {
		STOPPED, //The state that elevator starts, does nothing unless the home function is run.
		HOMING,  //Brings elevator slowly to the bottom most position possible, sets the offset. Must be done before any use of elevator.
		HOLDING, //State of the elevator holding position, both types of elevator usage can be used from this state.
		MOVING, //State of elevator moving to a target position within the ACCEPTABLE_ERROR.
		JOYSTICK; //Moves the elevator by a desired power returns to IDLE after setting the power once.
		//START; //Executes at the beginning of auto.
	}
	States state = States.STOPPED;
	
	public Elevator() {
		encoder = new Encoder(Constants.DIO_LIFT_ENCODER_A, Constants.DIO_LIFT_ENCODER_B, true, EncodingType.k4X);
		elevatorLeft.setInverted(true);
		pid.setVelocityScalars(velP, velI, velD);
		pid.setVelocityInverted(true);
		pid.setPositionScalars(posP, posI, posD);
		pid.setPositionInverted(true);
		arm = new Arm(this);
		
		Thread t = new Thread(new MagSwitchTask()); // This starts the new thread for the magnetic sensor.
		t.start();
	}
	

	/**
	 *  Initialize elevator variables whenever enabling robot
	 * 
	 * @param power
	 */
	public void init() {
		targetHeight = 0;
		arm.init();
		home();
	}

	/**
	 * A safe function to set the power of the elevator, cannot exceed MAX_POWER
	 * 
	 * @param power - power to run the elevator at, + = up and - = down
	 */
	public void setPower(double power) {
		//Common.dashNum("setPower passed power", power);
		if (power > 0.0) {  //Move up
			if(getInches() >= SAFE_HEIGHT) { //hard limit on expected height
				power = 0.0;
			/*} //NOTE: encoderTest is doing this safety check. Not needed here.
				else if(!magSwitchSafe()) {  //Have we made it to the magnetic switch trigger point(limit switch false = reached)
				power = 0.0;
				Common.debug("Magnetic switch fail, HOMING");
				state = States.HOMING; */
			} else if(getInches()>= SAFE_HEIGHT-DANGER_ZONE) {
				power = Math.min(power, Common.map(SAFE_HEIGHT-getInches(), 0.0, DANGER_ZONE, MIN_UP_POWER, MAX_UP_POWER));
			} else {
				power = Math.min(power, MAX_UP_POWER);
			}
		} else {  //Moving Down
			double safeHeight = minArmSafeHeight(arm.getPosition());
			if (atBottom() || getInches() < safeHeight) { //lower limit true when pressed
				power = 0.0;
				if (state == States.HOMING) 
					Common.debug("setPower zeroed power while HOMING "+atBottom());
			} else {
				if(state != States.HOMING) {
					if (getInches() - safeHeight <= DANGER_ZONE) { // This is for the lower danger zone.
						power = Math.max(power, Common.map(getInches() - safeHeight, 0.0, DANGER_ZONE, MIN_DOWN_POWER, MAX_DOWN_POWER));
					} else {
						power = Math.max(power, MAX_DOWN_POWER);
					}
				}
			}
		}
		power = encoderTest(power); 

		lastPower = power;
		elevatorRight.set(power); 
		elevatorLeft.set(power);
		Common.dashNum("Elevator power", power);
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
			if (moveCheck >= 3) {  //Test encoder movement after 3 cylces
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
				previousCounts = getEncoder();
			} else {
				moveCheck += 1;
			}
		}
		return power;
	}
	
	
	/**
	 * Uses a pid to move the robot at a target velocity
	 * 
	 * @param targetVelocity -The target speed for the robot to move in inches per second
	 */
	public void pidVelMove(double targetVelocity) {
		 
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
	 * Uses a PID to move the robot to a targetHeight or minimum safe height for arm.
	 */
	public void pidDisMove() {
		double safeTarget = Math.max(targetHeight, minArmSafeHeight(arm.getPosition()));
		pid.setTargetPosition(safeTarget);
		double pidDisCalc = pid.calc(getInches(), getVelocity());
		Common.dashNum("pidDisCalc", pidDisCalc);
		accelPower(pidDisCalc);
	}

	/**
	 * Gets the arm object
	 * @return arm
	 */
	public Arm getArm (){
		return arm;
	}

	/**
	 * Checks for a safe angle that the arm can be moved to a height.
	 * @param height the height to check safe angle of.
	 * @return the safe angle.
	 */
	public double minArmSafeAngle(double height){
        double yElevation, yAvailable;
        if (height < MIN_BUMPER_CLEAR_HEIGHT) { //Allows hand in front of bumpers.
            yElevation = height + ARM_PIVOT_TO_FLOOR -Y_HAND_FLOOR_SAFETY;
        } else {
            yElevation = height + ARM_PIVOT_TO_FLOOR - BUMPER_INCHES_TO_FLOOR - Y_HAND_SAFETY;
        }
        yAvailable = yElevation - (Y_HAND_EXT);
        
        //Checks if the arm should be unconstrained
		//if(yAvailable < ARM_LEN + ARM_PIVOT_TO_FLOOR - BUMPER_INCHES_TO_FLOOR + Y_HAND_EXT) 
        if (Math.abs(yAvailable/ARM_LEN) < 1)
        {
            double radians = Math.asin(yAvailable/ARM_LEN);
			return -Math.toDegrees(radians);
		}
		else
		{
			return -90;
		}
	}

	/**
	 * Checks for a safe height that elevator can be moved to, for a given arm angle.
	 * @param angle the angle in degrees.
	 * @return the safe height in inches.
	 */
	public double minArmSafeHeight(double angle){
        //double constant = (-BUMPER_INCHES_TO_FLOOR + ARM_PIVOT_TO_FLOOR) + Y_HAND_SAFETY + (Y_HAND_EXT)- PIVOT_TO_BOTTOM;
        double minHeight;
        double constant = Y_HAND_EXT - ARM_PIVOT_TO_FLOOR;
        double angleReach = (-Math.sin(Math.toRadians(angle)) * ARM_LEN);
       if (angle < MIN_BUMPER_CLEAR_ANGLE) {
            //Over the bumper
            minHeight = angleReach + constant+ BUMPER_INCHES_TO_FLOOR + Y_HAND_SAFETY;
        } else {
            //Outside the bumper
            minHeight = angleReach + constant +  Y_HAND_FLOOR_SAFETY;
        }
        return Math.max(0, minHeight);
	}
	
	/**
	 * 
	 * @return
	 */
	/*
	public double maxArmSafeAngle(){
		double yElevation = getInches() + INCHES_FROM_FLOOR + BUMPER_OFFSET;
		double yAvailable = yElevation - (FORBAR_YEXT + ARM_ARC_BUFFER);

		if(yAvailable < ARM_LEN) 
		{
			return -Math.asin(yAvailable/ARM_LEN);
		}
		else
		{
			return -90;
		}
	}
	*/

	/**
	 * Whether or not the intake is safe to move at the current elevator height.
	 * 
	 * @return - True when it is safe to move the intake over the back
	 */
	/*
	public boolean intakeSafe() {
		if (getInches() >= SAFE_HEIGHT-1.4) {
			return true;
		} else {
			return false;
		}
	}
	*/

	/**
	 * Tests to see if mag switch is triggered and if so, if elevator is at safe height.
	 * 
	 * @return - True if no issue detected, False if mag is trigged and height is not safe.
	 */
	public boolean magSwitchSafe() {
		//greater or equal to total height
		if (atMagSwitch()) {
			if(getInches() > MAG_SWITCH_POINT){
				return true;
			}
			else {
				return false;
			}
		}
		return true;
	}
	
	/**
	 * Gets if the elevator is pressing limit switch at bottom
	 * 
	 * @return - true = elevator is at bottom
	 */
	public boolean atBottom() {
		if (lowerLimit.get()) {
			return true;
		} else {
			return false;
		}
	}
	
	/** Gets the elevator height as a percentage of total height
	 * 
	 * @return -Elevator height as a percentage
	 */
	public double percentHeight() {
		return Common.map(getInches(), 0, SAFE_HEIGHT, 0, 1);
	}
	
	
	/**
	 * Starts homing the elevator and ensure Arm moves up to accomodate
	 */
	public void home() {
		Common.debug("Elevator Homing and Arm moving to safe height for Home");
		state = States.HOMING;
		double safeAngle = Math.max(arm.MIN_HOMING_ANGLE, arm.getPosition());
		arm.movePosition(safeAngle);
	}
	
	/*public void start() {
		state = States.START;
		startTime = Common.time();
	}*/
	
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
		return enumerableState.toString();
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
	/**
	 * Checks if the elevator is at the magnetic switch.
	 * @return Boolean true when at magnetic switch.
	 */
	public boolean atMagSwitch(){ // Should return false when triggered
		return !magSwitch.get();
	}
	/**
	 * Gets the current height of the elevator in inches
	 * 
	 * @return - height in inches 
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
		if (state != States.STOPPED && state != States.HOMING && Robot.isTeleopAllowed()){
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
	 * Starts moving the elevator to a target height.  In turn it inititates
	 * and Arm move to a safe angle for that height.
	 * 
	 * @param targetHeight - Height in inches
	 */
	public void moveToHeight(double inches) {
		if (state != States.STOPPED && state != States.HOMING && state != States.JOYSTICK) {
			this.targetHeight = inches;  // pidDISMove() will use this target control position PID
			double safeAngle = minArmSafeAngle(inches);
			if (arm.getPosition() < safeAngle) {
				//swCommon.debug("Elevator forcing arm angle");
				arm.movePosition(safeAngle);
			}
			//pid.setTargetPosition(inches);
			state = States.MOVING;
		}
	}
	
	/**
	 * Whether or not the elevator is within acceptable range of the position target.
	 * 
	 * @return - complete
	 */
	public boolean isComplete() {
	//	return Math.abs(pid.getTargetPosition() - getInches()) < ACCEPTABLE_ERROR;
		return Math.abs(targetHeight - getInches()) < ACCEPTABLE_ERROR;
	}
	
	/**
	 * Prints standard debug information about elevator components.
	 */
	public void debug() {
		Common.dashNum("Elevator Target Height", this.targetHeight);
		Common.dashNum("Elevator Encoder", getEncoder());
		Common.dashNum("Elevator Encoder in Inches", getInches());
		//Common.dashBool("Magnetic Sensor Safe", magSwitchSafe());
		//Common.dashBool("at Mag Switch", atMagSwitch());
		Common.dashBool("At Bottom", atBottom());
		Common.dashStr("Elevator State", state.toString());
		Common.dashNum("Elevator Velocity", getVelocity());
		Common.dashNum("Position PID Target", pid.getTargetPosition());
		Common.dashNum("Velocity PID Target", pid.getTargetVelocity());
		//Common.dashNum("Get Rate", encoder.getRate());
	}
	
	/**
	 * Update function that should be the last run of all elevator functions.
	 * Exports certain values to smart dashboard and runs the state process of the elevator
	 * 
	 * CALLS ARM.UPDATE!!!!!
	 */
	public void update() {
		pid.update();
		switch(state) {
		case STOPPED:
			setPower(0.0);
			break;
		case HOMING:
			if (atBottom()) { 
				resetEncoder();
				setPower(0.0);
				pid.setTargetPosition(0);
				Common.debug("New state Holding");
				state = States.HOLDING;
			} else {
				setPower(-0.12);  //was -0.10
			}
			break;
		/*case START:
			state = States.HOMING;
			//moveToHeight(5);
			break;*/
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
				// Each cycle, JOYSTICK can set velocity, but we'll predict where elevator would coast to if joystick input stops.
				// The JOYSTICK state only lasts one cycle, but if joystick is held, it updates every cycle via joystickControl().
				state = States.MOVING;
				// Set an predicted height for elevator based on current velocity
				double predictedHeight = getInches() + 0.13*getVelocity();
				// Contstrain to safe target height, based on safety for arm angle and elevator safe max height.
				targetHeight = (Math.max(minArmSafeHeight(arm.getPosition()), Math.min(SAFE_HEIGHT, predictedHeight)));
			}
			break;
		}
		arm.update();
	}
	
	public class MagSwitchTask implements Runnable {
		//private long previousTime;
		//private int counter = 0;
		public void run() {
			while (true) {
				//counter++;
				if(atMagSwitch()) {  //Have we made it to the magnetic switch trigger point(limit switch false = reached)
					if (getInches() < MAG_SWITCH_POINT) {  //Make sure encoder has counted enough inches
						Common.debug("Magnetic switch fail, HOMING");
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