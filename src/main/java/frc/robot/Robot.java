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


public class Robot extends TimedRobot {
	private Xbox driver;
	private Xbox operator;
	private Heading heading;
	private DriveTrain dt;
	private DigitalInput headingbutton;
	//private Slider slider;
	private Arm arm;
	private Elevator elevator;

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
	private final double ELE_LOW_CARGO=14.5, ELE_MID_CARGO=-1, ELE_HIGH_CARGO=-1, ELE_LOW_HATCH=-1,
	ELE_MID_HATCH=-1, ELE_HIGH_HATCH=-1, ARM_LOW_PLACE=-1, ARM_HIGH_PLACE =-1;

	//Distance to add/subtract to make place/pickup smooth
	private final double ELE_DIFF = 0;

	//Distances for pi
	private final double GRAB_DIST = -1, STOW_SAFE = -1;

	public boolean hasHatch = false;

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
		//dt = new DriveTrain();
		
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
		activePeriodic();
	}

	@Override
	public void teleopInit() {
		elevator.home();
		arm.init();
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
		**/
		userMove = false;


		double forward = joystickX(GenericHID.Hand.kLeft); 
		double turn = joystickY(GenericHID.Hand.kRight);
		dt.accelDrive(forward, turn);

		if (driver.getPressed("leftBumper")) {
			arm.runIntake(); //in
		}

		if (driver.when("b")) {
			arm.runIntake(); //out
		}

		if (isTeleopAllowed()) {
			//Driver
			if (driver.when("rightBumper")) {
				arm.doStowDown();
				elevator.moveToHeight(this.ELE_LOW_HATCH);
				arm.startAlign();//unknown function to start slider movement
				state = States.HATCH_PICKUP;
			}

			if (driver.when("a")) {
				arm.doHorizontal();
				elevator.doStowUp(); //Is this the same position?
				state =  States.CARGO_PICKUP;
			}

			//Joystick
			double armMove = driver.deadzone(driver.getX(GenericHID.Hand.kRight));
			if (Math.abs(armMove) > 0) {
				userMove = true;
				arm.joystickControl(armMove);
			}

			//Operator
			if (operator.when("rightBumper")) {
				if (arm.getPosition() < arm.horizental) {
					arm.movePosition(ARM_LOW_PLACE);
					elevator.doPlace(-1);
				} else {
					arm.movePosition(ARM_HIGH_PLACE);
					elevator.doPlace(1);
				}
				state = States.HATCH_PLACE;
			}

			
			
			/*
			*  Need to change userMove when elevator or arm is being used by human
			*/
		}
		

		debug();
		slider.update();
		if (elevator.state == elevator.States.HOMING) {
			arm.doStowUp();
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
		/*
			TODO: center slider with cargo
			TODO: checks to see if things are in position
			TODO: TO_STOW
			TODO: think about more safeties
		*/
		if (state == States.HOMING || state == States.HATCH_GRAB || state == States.HATCH_SEARCH || state ==States.HATCH_PLACE) {
			teleopAllowed =  false;
		} else {
			teleopAllowed = true;
		}
		switch(state) {
		case HOMING:
			if (elevator.getState() != elevator.States.HOMING) {
				safe = true;
				state = States.TO_STOW;
			}
			break;
		case EMPTY:
			if (arm.hasCargo()) {
				state = States.HAS_CARGO;
			} else if (arm.hasHatch()) {
				state = States.HAS_HATCH;
			}
			break;
		case HATCH_PICKUP:
			if (HatchVision.getDistance() <= this.GRAB_DIST) {
				state = States.HATCH_GRAB;
				//Assumes arm will stop
				arm.positionLowPlace();
				elevator.doPlace(-1); //Down
				arm.fingerSearch();//Starts fingerSearch
			}
			if (userMove) {
				state = States.EMPTY;
			}
			break;
		case HATCH_GRAB:
			if (hasHatch) {
				state = States.TO_STOW;
			}
			if (arm.isPressure()) { //Arm is pressed
				if (!arm.fingerPressd()) {//Finger unpressed
					arm.raiseFinger();
					hasHatch = true;
					state = States.HAS_HATCH;
				} else {
					state = States.HATCH_SEARCH;
				}
			}
			break;
		case HATCH_SEARCH:
			if (arm.isPressure && !arm.pressed) {
				arm.raiseFinger();
				hasHatch = true;
				state = States.TO_STOW;
			}
			//TODO: Needs safety?
			if (!arm.slider.fingerSearching || arm.isPressure) {
				state = States.HATCH_PICKUP;
				arm.doStowDown();
				elevator.moveToHeight(this.ELE_LOW_HATCH);
				arm.startAlign();//unknown function to start slider movement
			}
			break;
		case HAS_HATCH: 
			if (!hasHatch) {
				state = States.EMPTY;
			}
			break;
		case HATCH_PLACE:
			int t = 0;
			if (arm.isPressure) {
				arm.slider.dropFinger();
				t++;
			}
			if (t >= 2) {
				state = States.TO_STOW;
			}
			break;
		case CARGO_PICKUP:
			arm.runIntake();
			if (userMove) {
				state = States.EMPTY;
			}
			if (arm.hasCargo()) {
				state = States.TO_STOW;
			}
			break;
		case HAS_CARGO:
			if (!arm.hasCargo) {
				state = States.EMPTY;
			}
			break;
		case  TO_STOW:
			if (stowUp && pi.getDistance > STOW_SAFE) {
				elevator.doStowUp(); //TODO: make sure this is the correct function.
				arm.doStowUp();
			} else if (pi.getDistance >STOW_SAFE) {
				elevator.doStowDown();
				arm.doStowUp();
			}
			if (arm.isComplete() && elevator.isComplete()) {
				if (elevator.Target == elevator.stowDown || elevator.Target == elevator.stowDown) {
					if (arm.Target == arm.stowUp || arm.target == arm.stowDown) {
						if (hasHatch) {
							state = States.HAS_HATCH;
						} else if (arm.hasCargo) {
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
	public static boolean isTeleopAllowed(){
		return teleopAllowed;
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
}