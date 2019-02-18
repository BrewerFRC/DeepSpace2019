/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;


public class Robot extends TimedRobot {
	private Xbox driver;
	private Xbox operator;
	private Heading heading;
	private DriveTrain dt;
	private DigitalInput headingbutton;
	private Slider slider;
	private Arm arm;
	private Elevator elevator;
	private Intake intake;

	private enum States {
		EMPTY,
		//TO_STOW,
		STOW_UP,
		STOW_DOWN,
		HOMING,
		HATCH_PICKUP,
		HATCH_GRAB,
		HATCH_SEARCH,
		HAS_HATCH,
		HATCH_PLACE,
		CARGO_PICKUP,
		HAS_CARGO
	}
	private States state = States.HOMING;
	
	//Needs to be set true when elevator or arm is being used by a human
	private boolean userMove = false;

	//Position constants
	//Ball is 13 inches abover elevator roughly
	private final double ELE_LOW_CARGO = 5, ELE_MID_CARGO=32, ELE_HIGH_CARGO=60, ARM_HIGH_CARGO = 50, ELE_LOW_HATCH = 26.5,
	ELE_MID_HATCH= 20, ELE_HIGH_HATCH= 50, ARM_LOW_PLACE=-47, ARM_HIGH_PLACE =41,
	ARM_HIGH_STOW = 60, ELE_LOW_STOW = 25, ARM_LOW_STOW = -66, ELE_HIGH_STOW = 3.3,
	ARM_CARGO_PICKUP = -5, ELE_CARGO_PICKUP = 2;
	//Angle should be around 40 to place

	//Distance to add/subtract to make place/pickup smooth
	private final double ELE_DIFF = 0;

	//Distances for pi
	//private final double GRAB_DIST = -1, STOW_SAFE = -1;


	//Whether or not to stow up.
	//True is up, false is down.
	public boolean stowUp = true;

	static boolean teleopAllowed = true;

	public Robot() {
		//m_robotDrive.setExpiration(0.1);
	}

	@Override
	public void robotInit() {
		driver = new Xbox(0);
		operator =  new Xbox(1);
		heading = new Heading();
		heading.reset();
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(240, 180);
		elevator = new Elevator();
		arm = elevator.arm;
		slider = arm.getSlider();
		intake = arm.intake;
		dt = new DriveTrain(elevator);
		/*heading = new Heading();
		heading.reset();
		headingbutton = new DigitalInput(5);*/

	}

	@Override
	public void disabledPeriodic() {
		if (operator.when("start") || driver.when("start")) {
			slider.toggleHasHatch();
		}
		debug();
		arm.dashboard();
	}

	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("Ir inches", intake.getInfaredInches());
		//activePeriodic();
	}

	@Override
	public void teleopInit() {
		elevator.init();
		
		
		//heading.reset();
		//heading.setHeadingHold(true);
	}

	@Override
	public void teleopPeriodic() {
		//activePeriodic();
		testPeriodic();
	}


	public void testPeriodic(){
		arm.movePosition(20);
		arm.update();
		arm.dashboard();
		elevator.update();
	}

	public void activePeriodic() {
		/* Driver:
		 * Left bumper is hard intake run
		 * Right bumper is hatch pickup
		 * left joystick is drive
		 * Right joystick is arm
		 * start is hasHatch (and back is hasCargo not implemented)
		 * B is spit cargo
		 * A is cargo intake state
		 * 
		 * Operator:
		 * Left bumper is Arm angle for cargo
		 * Right bumper is place hatch
		 * Left Joystick is drive
		 * Right Joystick is elevator
		 * Up arrow is up stow
		 * Down arrow is down stow
		 * A is low position(for placing[defaults to cargo])
		 * B is middle position
		 * Y is top position
		 */
		userMove = false;


		double forward = joystickY(GenericHID.Hand.kLeft); 
		double turn = joystickX(GenericHID.Hand.kLeft);
		dt.accelDrive(forward, turn);

		if (driver.getPressed("leftBumper")) {
			intake.toggleLoading(); //in
		}

		if (driver.when("b")) {
			intake.doEject(); //out
		}

		if (operator.when("dPadUp")) {
			stowUp = true;
		}
		
		if (operator.when("dPadDown")) {
			stowUp = false;
		}

		if (operator.when("start") || driver.when("start")) {
			slider.toggleHasHatch();
		}

		if (isTeleopAllowed()) {
			//Driver
			if (driver.when("rightBumper")) { //Hatch pickup
				arm.movePosition(this.ARM_LOW_STOW);;
				elevator.moveToHeight(this.ELE_LOW_HATCH);
				//arm.startAlign();//unknown function to start slider movement
				state = States.HATCH_PICKUP;
			}

			if (driver.when("a")) { //Cargo pickup
				Common.debug("Cargo pickup");
				arm.movePosition(ARM_CARGO_PICKUP);
				elevator.moveToHeight(ELE_CARGO_PICKUP);
				state =  States.CARGO_PICKUP;
			}
/*
			//Joystick Arm
			double armMove = driver.deadzone(driver.getY(GenericHID.Hand.kRight));
			if (Math.abs(armMove) > 0) {
				userMove = true;
				arm.joystickControl(armMove);
			}
*/
			//Operator
			if (operator.when("rightBumper")) { //Place hatch
				if (arm.getPosition() < 0) {
					arm.movePosition(ARM_LOW_PLACE);
					//elevator.doPlace(-1);
				} else {
					arm.movePosition(ARM_HIGH_PLACE);
					//elevator.doPlace(1);
				}
				state = States.HATCH_PLACE;
			}

			if (operator.when("leftBumper")) {
				arm.movePosition(55);//For upper level?
			}

			//Set points
			if (slider.hasHatch()) {
				if (operator.when("a")) {
					elevator.moveToHeight(ELE_LOW_HATCH);
				}
				if (operator.when("b")) {
					elevator.moveToHeight(ELE_MID_HATCH);
				}
				if (operator.when("y")) {
					elevator.moveToHeight(ELE_HIGH_HATCH);
				}
			} else {
				if (operator.when("a")) {
					elevator.moveToHeight(ELE_LOW_CARGO);
				}
				if (operator.when("b")) {
					elevator.moveToHeight(ELE_MID_CARGO);
				}
				if (operator.when("y")) {
					elevator.moveToHeight(ELE_HIGH_CARGO);
				}
			}

			//Joystick elevator
			double operatorRight = operator.getY(GenericHID.Hand.kRight);
			if (Math.abs(operatorRight) > .2) {
				userMove = true;
				elevator.joystickControl(operatorRight);
			}

		}
		
		// Updates
		elevator.update();
		elevator.debug();
		//intake.update();
		arm.dashboard();
		debug();
		Common.dashStr("Robot state",state.toString());

		//Still neccasary?
		if (elevator.getState() == Elevator.States.HOMING) {
			arm.movePosition(this.ARM_HIGH_STOW);
			state = States.HOMING;
		}
		update();
	}
	/**
	 * Debugs data to smart dashboard
	 **/
	public void debug() {
		Common.dashBool("userMove", this.userMove);
		/*SmartDashboard.putNumber("Degrees NavX", heading.getNavXAngle());
		SmartDashboard.putNumber("Target angle", heading.getTargetAngle());
		SmartDashboard.putNumber("PID", heading.turnRate());
		if(Math.abs(driver.getY(GenericHID.Hand.kLeft)) > 0.15){
			elevator.joystickControl(driver.getY(GenericHID.Hand.kLeft));
		}

		elevator.update();

		elevator.debug();
		//SmartDashboard.putNumber("PID", heading.turnRate());
		*/
	}

	/**
	 * Returns if it is okay for the armevator to move via command.
	 * 
	 * @return Whether it is okay to move the armevator.
	 */
	public static boolean isTeleopAllowed(){
		return teleopAllowed;
	}
	
	public void update() {
		if (state == States.HOMING || state == States.HATCH_GRAB || state == States.HATCH_SEARCH || state == States.HATCH_PLACE) {
			teleopAllowed =  false;
		} else {
			teleopAllowed = true;
		}
		switch(state) {
		case HOMING:
			if (elevator.getState() != Elevator.States.HOMING) {
				teleopAllowed = true;
				Common.debug("Robot State going from HOMING to STOW_UP");
				state = States.STOW_UP;
			}
			break;
		case EMPTY:
			slider.moveTo(0);
			if (intake.getInfaredCheck()) {
				Common.debug("Robot State going from EMPTY to HAS_CARGO");
				state = States.HAS_CARGO;
			} else if (slider.hasHatch()) {
				Common.debug("Robot State going from EMPTY to HAS_HATCH");
				state = States.HAS_HATCH;
			}
			break;
		case HATCH_PICKUP:
			if (slider.pressed()) {//Replace with pressure?
				Common.debug("Robot State going from HATCH_PICKUP to HATCH_GRAB");
				state = States.HATCH_GRAB;
			}
			arm.movePosition(this.ARM_LOW_PLACE);
			//elevator.doPlace(-1); //Down
			slider.fingerDown();
			//arm.fingerSearch();//Starts fingerSearch
			if (userMove) {
				Common.debug("Robot State going from HATCH_PICKUP to EMPTY");
				state = States.EMPTY;
			}
			break;
		case HATCH_GRAB:
			if (slider.hasHatch()) {
				elevator.moveToHeight(elevator.getInches()+5);
				if (elevator.isComplete()) {
					Common.debug("Robot State going from HATCH_GRAB to STOW_DOWN");
					state = States.STOW_DOWN;
				}
			}
			if (slider.pressed()) { //Arm is pressed
				slider.startRightFingerSearch();
				Common.debug("Robot State going from HATCH_GRAB to HATCH_SEARCH");
				state = States.HATCH_SEARCH;
			}
			break;
		case HATCH_SEARCH:
			if (slider.hasHatch()) {
				elevator.moveToHeight(elevator.getInches()+5);
				if (elevator.isComplete()) {
					Common.debug("Robot State going from HATCH_SEARCH to STOW_DOWN");
					state = States.STOW_DOWN;
				}
			}
			if (slider.getSliderState() == Slider.states.MOVING) {
				Common.debug("Robot State going from HATCH_SEARCH to HATCH_PICKUP");
				state = States.HATCH_PICKUP;
				stowDown();
				//arm.startAlign();//unknown function to start slider movement
			}
			break;
		case HAS_HATCH: 
			if (!slider.hasHatch()) {
				Common.debug("Robot State going from HAS_HATCH to EMPTY");
				state = States.EMPTY;
			}
			break;
		case HATCH_PLACE:
			int t = 0;
			if (slider.pressed()) {
				slider.fingerDown();
				t++;
			}
			if (t >= 2) {
				//state = States.TO_STOW;
				//TODO: better exit
				if (arm.getPosition() > 0) {
					stowUp = true;
				} else {
					stowUp = false;
				}
				if (arm.getPosition() > 0) {
					Common.debug("Robot State going from HATCH_PLACE to STOW_UP");
					state = States.STOW_UP;
				} else {
					Common.debug("Robot State going from HATCH_PLACE to STOW_DOWN");
					state = States.STOW_DOWN;
				}
			}
			break;
		case CARGO_PICKUP:
			slider.moveTo(0);
			intake.toggleLoading();;
			if (userMove) {
				Common.debug("Robot State going from CARGO_PICKUP to EMPTY");
				state = States.EMPTY;
			}
			if (intake.getInfaredCheck()/* || driver.when("a")*/) {
				Common.debug("Robot State going from CARGO_PICKUP to STOW_UP");
				state = States.STOW_UP;
			}
			break;
		case HAS_CARGO:	
			slider.moveTo(0);
			if (!intake.getInfaredCheck()) {
				Common.debug("Robot State going from HAS_CARGO to EMPTY");
				state = States.EMPTY;
			}
			break;
		/*case  TO_STOW:
			Common.dashBool("Stow Up", stowUp);
			if (stowUp || intake.getInfaredCheck()/* && pi.getDistance > STOW_SAFE) {
				stowUp();
			} else /*if (pi.getDistance >STOW_SAFE) {
				stowDown();
			}
			if (arm.isComplete() && elevator.isComplete()) {
				slider.fingerUp();
				if (slider.hasHatch()) {
					Common.debug("Robot State going from TO_STOW to HAS_HATCH");
					state = States.HAS_HATCH;
				} else if (intake.getInfaredCheck()) {
					Common.debug("Robot State going from TO_STOW to HAS_CARGO");
					state = States.HAS_CARGO;
				} else {
					Common.debug("Robot State going from TO_STOW to EMPTY");
					state = States.EMPTY;
				}
			}
			break;*/
		case STOW_UP:
			stowUp();
			if (arm.isComplete() && elevator.isComplete()) {
				slider.fingerUp();
				if (slider.hasHatch()) {
					Common.debug("Robot State going from STOW_UP to HAS_HATCH");
					state = States.HAS_HATCH;
				} else if (intake.getInfaredCheck()) {
					Common.debug("Robot State going from STOW_UP to HAS_CARGO");
					state = States.HAS_CARGO;
				} else {
					Common.debug("Robot State going from STOW_UP to EMPTY");
					state = States.EMPTY;
				}
			}
			break;
		case STOW_DOWN:
			stowDown();
			if (arm.isComplete() && elevator.isComplete()) {
				slider.fingerUp();
				if (slider.hasHatch()) {
					Common.debug("Robot State going from STOW_DOWN to HAS_HATCH");
					state = States.HAS_HATCH;
				} else if (intake.getInfaredCheck()) {
					Common.debug("Robot State going from STOW_DOWN to HAS_CARGO");
					state = States.HAS_CARGO;
				} else {
					Common.debug("Robot State going from STOW_DOWN to EMPTY");
					state = States.EMPTY;
				}
			}
			break;
		}
	}

	/**
	 * Returns state of robot.
	 * 
	 * @return state of robot.
	 */
	public States getState() {
		return this.state;
	}
	
	/**
	 * Returns if it is safe to move the elevator/arm.
	 * 
	 * @return if it is safe to move the armevator.
	 */
	public boolean safeToMove() {
		boolean safe = true;
		if (state == States.HOMING || state == States.HATCH_GRAB || state == States.HATCH_SEARCH || state ==States.HATCH_PLACE) {
			safe =  false;
		}
		return safe;
	}

	/**
     * Gets the highest joystick x value from the defined hand.
     * 
     * @param hand the hand to get the value from.
     * @return double the value.
     */
    public double joystickX(GenericHID.Hand hand) {
    	if (hand == GenericHID.Hand.kLeft) {
    		return (Math.abs(driver.getX(hand)) > Math.abs(operator.getX(hand))) ? driver.getX(hand) : operator.getX(hand);
    	}
    	return (Math.abs(driver.getX(hand)) > Math.abs(operator.getX(hand))) ? driver.getX(hand) : operator.getX(hand);
    }
    
    /**
     * Gets the highest joystick y value from the defined hand.
     * 
     * @param hand the hand to get the value from.
     * @return double the value.
     */
    public double joystickY(GenericHID.Hand hand) {
    	if (hand == GenericHID.Hand.kLeft) {
    		return (Math.abs(driver.getY(hand)) > Math.abs(operator.getY(hand))) ? driver.getY(hand) : operator.getY(hand);
    	}
    	return (Math.abs(driver.getY(hand)) > Math.abs(operator.getY(hand))) ? driver.getY(hand) : operator.getY(hand);
	}
	
	/**
	 * Stows the robot so the arm is up.
	 */
	public void stowUp() {
		elevator.moveToHeight(ELE_HIGH_STOW);
		arm.movePosition(ARM_HIGH_STOW);
	}

	/**
	 * Stows the robot so the arm is down.
	 */
	public void stowDown() {
		elevator.moveToHeight(ELE_LOW_STOW);
		arm.movePosition(ARM_LOW_STOW);
	}
}