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
	Xbox driver;
	Heading heading;
	DriveTrain dt;
	DigitalInput headingbutton;
	Slider slider;
	Elevator elevator;

	static boolean teleopAllowed = true;

	public Robot() {
		//m_robotDrive.setExpiration(0.1);
	}

	@Override
	public void robotInit() {
		//dt = new DriveTrain();
		elevator = new Elevator();
		/*heading = new Heading();
		heading.reset();
		headingbutton = new DigitalInput(5);*/

		driver = new Xbox(0);
		//slider = new Slider();
	}

	@Override
	public void autonomousPeriodic() {
		
	}

	@Override
	public void teleopInit() {
		elevator.home();
		//heading.reset();
		//heading.setHeadingHold(true);
	}
	@Override
	public void teleopPeriodic() {
		//double leftJoystickX = driver.getX(GenericHID.Hand.kLeft);
		//double leftJoystickY = -driver.getY(GenericHID.Hand.kLeft);

		if(driver.getStartButtonPressed()){
			elevator.resetEncoder();
		}

		if(Math.abs(driver.getY(GenericHID.Hand.kLeft)) > 0.15){
			elevator.joystickControl(driver.getY(GenericHID.Hand.kLeft));
		}

		elevator.update();

		elevator.debug();
		//SmartDashboard.putNumber("PID", heading.turnRate());
	}

	public static boolean isTeleopAllowed(){
		return teleopAllowed;
	}
	
	
}