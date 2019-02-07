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

public class Robot extends TimedRobot {
	private Xbox driver;
	private Xbox operator;
	private Heading heading;
	private BasicDrive drivetrain;
	private DigitalInput headingbutton;
	private Slider slider;
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
	private final double ELE_LOW_CARGO=-1, ELE_MID_CARGO=-1, ELE_HIGH_CARGO=-1, ELE_LOW_HATCH=-1,
	ELE_MID_HATCH=-1, ELE_HIGH_HATCH=-1, ARM_LOW_PLACE=-1, ARM_HIGH_PLACE =-1;

	//Distance to add/subtract to make place/pickup smooth
	private final double ELE_DIFF = 0;

	//Distances for pi
	private final double GRAB_DIST = -1, STOW_SAFE = -1;

	public boolean hasHatch = false;

	//Whether or not to stow up.
	//True is up, false is down.
	public boolean stowUp = true;

	public Robot() {
		//m_robotDrive.setExpiration(0.1);
	}

	@Override
	public void robotInit() {
		driver = new Xbox(0);
		operator =  new Xbox(1);
		heading = new Heading();
		heading.reset();
		headingbutton = new DigitalInput(5);
		slider = new Slider();

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
		heading.reset();
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
		 * start is haHatch and back is hasCargo
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
		
		if (safeToMove()) {
			if (driver.when("rightBumper")) {
				arm.doStowDown();
				elevator.setPosition(this.ELE_LOW_HATCH);
				arm.startAlign();//unknown function to start slider movement
				State = States.HATCH_PICKUP;
			}

			if (operator.when("rightBumper")) {
				if (arm.getPostion < arm.horizental) {
					arm.setPosition(ARM_LOW_PLACE);
					elevator.doPlace(-1);
				} else {
					arm.setPosition(ARM_HIGH_PLACE);
					elevator.doPlace(1);
				}
				state = States.HATCH_PLACE;
			}

			if (driver.when("rightBumper")) {
				arm.doHorizontal();
				elevator.doStowUp(); //Is this the same position?
				state =  States.CARGO_PICKUP;
			}
			
			/*
			*  Need to change userMove when elevator or arm is being used by human
			*/
		}
		

		debug();
		slider.update();
		if (elevator.getState = HOMING) {
			arm.doStowUp();
			state = States.HOMING;
		}
		update();
	}

	/**
	 * Debugs data to smart dashboard
	 **/
	public void debug() {
		SmartDashboard.putNumber("Degrees NavX", heading.getNavXAngle());
		SmartDashboard.putNumber("Target angle", heading.getTargetAngle());
		SmartDashboard.putNumber("PID", heading.turnRate());
	}
	
	public void update() {
		/*
			TODO: checks to see if things are in position
			TODO: TO_STOW
			TODO: think about more safeties
		*/
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
				elevator.setPosition(this.ELE_LOW_HATCH);
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

}