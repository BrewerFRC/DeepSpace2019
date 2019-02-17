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
		TO_STOW,
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
	private final double ELE_LOW_CARGO = -1, ELE_MID_CARGO=-1, ELE_HIGH_CARGO=-1, ELE_LOW_HATCH=-1,
	ELE_MID_HATCH= 20, ELE_HIGH_HATCH= 47, ARM_LOW_PLACE=-1, ARM_HIGH_PLACE =-1,
	ARM_HIGH_STOW = 60, ELE_LOW_STOW = 0, ARM_LOW_STOW =0;
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
		/*
		*  TODO: Added some way to mark a hatch to system
		*
		*/
		debug();
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
		activePeriodic();
	}

	public void activePeriodic() {
		/* Driver:
		 * Left bumper is hard intake run
		 * Right bumper is hatch pickup
		 * left joystick is drive
		 * Right joystick is arm
		 * start is hasHatch and back is hasCargo
		 * B is spit cargo
		 * A is cargo intake state
		 * 
		 * Operator:
		 * Left bumper is Arm aim for bumper
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


		double forward = joystickX(GenericHID.Hand.kLeft); 
		double turn = joystickY(GenericHID.Hand.kLeft);
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

		if (operator.when("start")) {
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
				arm.movePosition(0);
				elevator.moveToHeight(ELE_LOW_STOW); //Is this the same position?
				state =  States.CARGO_PICKUP;
			}

			//Joystick Arm
			double armMove = driver.deadzone(driver.getX(GenericHID.Hand.kRight));
			if (Math.abs(armMove) > 0) {
				userMove = true;
				arm.joystickControl(armMove);
			}

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

			//Joystick elevator
			double operatorRight = driver.getY(GenericHID.Hand.kRight);
			if (Math.abs(operatorRight) > .2) {
				userMove = true;
				elevator.joystickControl(operatorRight);
			}

		}
		
		// Updates
		elevator.update();
		elevator.debug();
		intake.update();
		arm.dashboard();
		debug();

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
				state = States.TO_STOW;
			}
			break;
		case EMPTY:
			slider.moveTo(0);
			if (intake.getInfaredCheck()) {
				state = States.HAS_CARGO;
			} else if (slider.hasHatch()) {
				state = States.HAS_HATCH;
			}
			break;
		case HATCH_PICKUP:
			if (slider.pressed()) {//Replace with pressure?
				state = States.HATCH_GRAB;
			}
			arm.movePosition(this.ARM_LOW_PLACE);
			//elevator.doPlace(-1); //Down
			slider.fingerDown();
			//arm.fingerSearch();//Starts fingerSearch
			if (userMove) {
				state = States.EMPTY;
			}
			break;
		case HATCH_GRAB:
			if (slider.hasHatch()) {
				state = States.TO_STOW;
			}
			if (slider.pressed()) { //Arm is pressed
				slider.startRightFingerSearch();
				state = States.HATCH_SEARCH;
			}
			break;
		case HATCH_SEARCH:
			if (slider.hasHatch()) {
				state = States.TO_STOW;
			}
			if (slider.getSliderState() == Slider.states.MOVING) {
				state = States.HATCH_PICKUP;
				stowDown();
				//arm.startAlign();//unknown function to start slider movement
			}
			break;
		case HAS_HATCH: 
			if (!slider.hasHatch()) {
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
				if (stowUp) {
					
				}
				state = States.EMPTY;
			}
			break;
		case CARGO_PICKUP:
			slider.moveTo(0);
			intake.toggleLoading();;
			if (userMove) {
				state = States.EMPTY;
			}
			if (intake.getInfaredCheck()) {
				state = States.TO_STOW;
			}
			break;
		case HAS_CARGO:	
			slider.moveTo(0);
			if (!intake.getInfaredCheck()) {
				state = States.EMPTY;
			}
			break;
		case  TO_STOW:
			if (stowUp/* && pi.getDistance > STOW_SAFE*/) {
				stowUp();
			} else /*if (pi.getDistance >STOW_SAFE)*/ {
				stowDown();
			}
			if (arm.isComplete() && elevator.isComplete()) {
				if (elevator.getInches() == ELE_LOW_STOW || elevator.getInches() == 0) {
					if (arm.getPosition() == ARM_HIGH_STOW || arm.getPosition() == ARM_LOW_STOW) {
						if (slider.hasHatch()) {
							state = States.HAS_HATCH;
						} else if (intake.getInfaredCheck()) {
							state = States.HAS_CARGO;
						} else {
							state = States.EMPTY;
						}
					}
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
		elevator.moveToHeight(0);
		arm.movePosition(60);
	}

	/**
	 * Stows the robot so the arm is down.
	 */
	public void stowDown() {
		elevator.moveToHeight(ELE_LOW_STOW);
		arm.movePosition(ARM_LOW_STOW);
	}
}