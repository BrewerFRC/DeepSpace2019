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
	Xbox driver;
	Heading heading;
	BasicDrive drivetrain;
	DigitalInput headingbutton;
	Slider slider;

	public Robot() {
		//m_robotDrive.setExpiration(0.1);
	}

	@Override
	public void robotInit() {
		driver = new Xbox(0);
		heading = new Heading();
		heading.reset();
		headingbutton = new DigitalInput(5);
		slider = new Slider();
	}

	@Override
	public void autonomousPeriodic() {
		
	}

	@Override
	public void teleopInit() {
		heading.reset();
		heading.setHeadingHold(true);
	}
	@Override
	public void teleopPeriodic() {
		//drivetrain.throttledAccelDrive(driver.getY(Hand.kLeft), driver.getX(Hand.kLeft));

		SmartDashboard.putNumber("Degrees NavX", heading.getNavXAngle());
		SmartDashboard.putNumber("Target angle", heading.getTargetAngle());
		SmartDashboard.putNumber("PID", heading.turnRate());
		slider.update();

		if (headingbutton.get()){
			heading.zeroTarget();
		}
	}
	
}