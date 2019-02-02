package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to control the intake arm and loader.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Brent Roberts
 * @author Sam Woodward
 */
public class Intake {
	private static final Spark
			intakeArm = new Spark(Constants.INTAKE_ARM),
			leftIntake = new Spark(Constants.LEFT_INTAKE_MOTOR),
			rightIntake = new Spark(Constants.RIGHT_INTAKE_MOTOR);
	private AnalogInput irInput = new AnalogInput(Constants.IR_SENSOR); //TODO: Decide on sensor
	private AnalogInput pot = new AnalogInput(Constants.INTAKE_POTENTIOMETER);
	private Solenoid hardSole = new Solenoid(Constants.PCM_CAN_ID, Constants.HARD_SOLE);
	private Solenoid openSole = new Solenoid(Constants.PCM_CAN_ID, Constants.OPEN_SOLE);
	private PositionByVelocityPID pid;
	
	public final double MAX_ELEVATOR_SAFE = 64, MIN_ELEVATOR_SAFE = 0, //Safe angles when elevator is not at top
			//The angle at which the intake is horizontal out the front.
			FRONT_HORIZONTAL = 0,
			MIN_POSITION = 210, MAX_POSITION = 3593, 
			MIN_ANGLE = -12, MAX_ANGLE = 160, 
			MAX_ABS_ANGLE = 209.0041,
			//The degrees that the power ramping takes place in at the limits
			DANGER_ZONE = 25,
			//Down powers
			MIN_DOWN_POWER = 0, MAX_DOWN_POWER = -0.1,
			//up powers
			MIN_UP_POWER = 0, MAX_UP_POWER = 0.5,
			//Max power change in accel limit
			MAX_DELTA_POWER = 0.1,
			MAX_VELOCITY = 120,
			COUNTS_PER_DEGREE = 14.89444444,
			PARTIALLY_LOADED_DISTANCE = 10,
			//maximum IR distance a fully loaded cube can be
			FULLY_LOADED_DISTANCE = 3,
			//How close to the targetHeight that elevator can be to complete
			ACCEPTABLE_ERROR = 2.0;
	
	private double P_POS = 3.0, I_POS = 0, D_POS = 700,
			P_VEL = 0.0002, I_VEL = 0, D_VEL = 0.05, G = 0.1,
			lastPower = 0, previousReading = 0;
	
	private long intakeTime = 0;
	public boolean loading = false;
	public double velocity = 0.0, lastVelocityTarget = 0;
	public double targetVelocity = 0.0;
	public double position = 64;
	private long previousMillis = Common.time();
	private boolean previousIntakeSafe = false;
	
	public enum States {
		STOPPED, //The state that elevator starts, does nothing unless the home function is run.
		HOLDING, //State of the elevator holding position, both types of elevator usage can be used from this state.
		MOVING, //State of elevator moving to a target position within the ACCEPTABLE_ERROR.
		JOYSTICK; //Moves the elevator by a desired power returns to IDLE after setting the power once.
	}
	States state = States.STOPPED;
	
	public Intake() {
		pid = new PositionByVelocityPID(MIN_ANGLE, MAX_ANGLE, -MAX_VELOCITY, MAX_VELOCITY, 0.01, "intake");
		pid.setPositionScalars(P_POS, I_POS, D_POS);
		pid.setVelocityScalars(P_VEL, I_VEL, D_VEL);
		pid.setVelocityInverted(true);
		pid.setPositionInverted(true);
		pid.setTargetPosition(60);
		intakeArm.setInverted(true);
		leftIntake.setInverted(true);
		Thread t = new Thread(new PotUpdate());
		t.start();
		SmartDashboard.putNumber("G", G);
	}
	
	/**
	 * Sets the power of the intake arm.
	 * 
	 * @param power - the power
	 */
	public void setArmPower(double power) {
		double maxAngle = getMaxAngle();
		if (power > 0.0) {
			if (getPosition() >= maxAngle) {
				power = 0.0;
				pid.reset();
			}
		} else {
			if (getPosition() <= MIN_ANGLE) { 
				power = 0.0;
				pid.reset();
			}
		}
		power = rampPower(power);
		intakeArm.set(power);
		Common.dashNum("Intake arm Power", power);
		Common.dashNum("Intake Last Power", lastPower);
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
		
		final double MIDDLE_POWER = 0.7;
		final double MAX_POWER = 1.0;
		final double MIN_POWER = 0.0;
		final double LOW_POWER = 0.08;
		double maxPower = 0.0;
		double minPower = 0.0;
		if (Robot.getElevator().intakeSafe())
			if (getPosition() < 90) {
				if (power > 0.0) {
					maxPower = Common.map(getPosition(), MIN_ANGLE, 90, MAX_POWER, MIDDLE_POWER);
					power = Math.min(power, maxPower);
				} else {
					minPower = Common.map(getPosition(), MIN_ANGLE, 90, -MIN_POWER, -MIDDLE_POWER);
					power = Math.max(power, minPower);
				}
			} else {
				if (power > 0.0 ) {
					maxPower = Common.map(getPosition(), 90, MAX_ANGLE, MIDDLE_POWER, MIN_POWER);
					power = Math.min(power, maxPower);
				} else {
					minPower = Common.map(getPosition(), 90, MAX_ANGLE, -MIDDLE_POWER, -MAX_POWER);
					power = Math.max(power, minPower);
				}
		} else {
			if ( power > 0.0) {
				maxPower = Common.map(getPosition(), MIN_ANGLE, MAX_ELEVATOR_SAFE, MAX_POWER, LOW_POWER);
				//Common.dashNum("Max Power Map", maxPower);
				power = Math.min(power, maxPower);
			} else {
				minPower = Common.map(getPosition(), MIN_ANGLE, MAX_ELEVATOR_SAFE, -LOW_POWER, -MIDDLE_POWER);
				power = Math.max(power, minPower);
			}
		}
		return power;
	}
	
	/**
	 * Sets the power of the left intake motor.
	 * 
	 * @param power - the power
	 */
	public void setLeftIntakePower(double power) {
		if (power >= 0.0 && isFullyLoaded()) {
			//was 0.0
			power = 0.25;
		}
		leftIntake.set(power);
	}
	
	/**
	 * Sets the power of the right intake motor.
	 * 
	 * @param power - the power
	 */
	public void setRightIntakePower(double power) {
		if (power >= 0.0 && isFullyLoaded()) {
			//was 0.0
			power = 0.25;
		}
		rightIntake.set(power);
	}
	
	/**
	 * Sets the power of both intake motors.
	 * 
	 * @param power - the power
	 */
	public void setIntakePower(double power) {
		setRightIntakePower(power);
		setLeftIntakePower(power);
		
	}
	
	/**
	 * Returns the distance of a cube from the infrared sensor.
	 * 
	 * @return - the distance in inches
	 */
	public double getCubeDistance() {
	  double reading = irInput.getValue() / 4 * 0.1 + previousReading * 0.9;
	  double inches = (-20.0/575.0)*reading+20;
	  if (inches < 0){
	    inches = 0;
	  }
	  previousReading = reading;
	  return inches;
	
	}
	
	/**
	 * Whether or not there is a cube partially or fully loaded.
	 * 
	 * @return cube loaded
	 */
	public boolean isPartiallyLoaded() {
		return (getCubeDistance() < PARTIALLY_LOADED_DISTANCE);
	}
	
	/**
	 * Detects fully loaded cube
	 * 
	 * @return -true will cube is loaded fully
	 */
	public boolean isFullyLoaded() {
		return (getCubeDistance() < FULLY_LOADED_DISTANCE);
	}
	
	/**
	 * Controls the intake power when loading a cube.
	 * 
	 * @param power - the power if a cube is not loaded
	 * @return - whether the cube is loaded
	 */
	public boolean loadCube(double power) {
		if (isPartiallyLoaded()) {
			if (Common.time() > intakeTime) {
				setIntakePower(0.0);
				intakeTime = 0;
				return true;
			} else {
				setIntakePower(power);
				return false;
			}
		}
		else {
			intakeTime = Common.time()+250;
			setIntakePower(power);
			return false;
		}
	}
	
	public void hardArm() {
		//Common.debug("Hard");
		hardSole.set(true);
		openSole.set(false);
	}
	
	public void softArm() {
		//Common.debug("soft");
		hardSole.set(false);
		openSole.set(false);
	}
	
	public void openArm() {
		//Common.debug("open");
		hardSole.set(false);
		openSole.set(true);
	}
	
	/**
	 * Gets the arm position in degrees.
	 * 
	 * @return - the position
	 */
	public double getPosition() {
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
		setAccelArmPower(pidPosCalc + G * Math.cos(Math.max(0, getPosition())));
	}
	
	/**
	 * PID controlled move at a defined velocity.
	 * 
	 * @param velocity - the velocity in degrees/second.
	 */
	public void moveVelocity(double velocity) {
		double maxAngle = getMaxAngle();
		double calc = 0;
		//If velocity changes direction, reset pid to speed up response.
		if ((lastVelocityTarget > 0 && velocity < 0) || (lastVelocityTarget < 0 && velocity > 0)) {
			pid.resetVelocityPID();
		}
		if (getPosition() > maxAngle && velocity > 0) {
			pid.setTargetVelocity(0.0);
		}
		else {
			pid.setTargetVelocity(velocity);
		}
		if (velocity >= 0.0) {
			calc = pid.calcVelocity(getVelocity()) + G * Math.cos(Math.max(0, getPosition()));
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
		if (getPosition() < MAX_ELEVATOR_SAFE) {
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
	
	public double getMaxAngle() {
		if (Robot.getElevator().intakeSafe()) {
			if (Robot.instance().isOperatorControl()) {
				return MAX_ELEVATOR_SAFE;
			}
			else {
				Common.dashBool("MAX_ANGLE", true);
				return MAX_ANGLE;
			}
		} 
		else {
			Common.dashBool("MAX_ANGLE", false);
			return MAX_ELEVATOR_SAFE;
		}
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
		G = SmartDashboard.getNumber("G", G);
		switch(state) {
			case STOPPED:
				setAccelArmPower(0.0);
				break;
			case HOLDING:
				if (!previousIntakeSafe && Robot.getElevator().intakeSafe()) {
					pid.resetVelocityPID();
				}
				previousIntakeSafe = Robot.getElevator().intakeSafe();
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
					pid.setTargetPosition(Math.max(MIN_ANGLE + 1, Math.min(getPosition() + velocity*0.1, getMaxAngle()-1)));
				}
				break;
		}
	}

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
		
	}

}
