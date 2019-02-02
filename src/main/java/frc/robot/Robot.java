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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot {
	Xbox driver;
	Heading heading;
	DriveTrain dt;
	DigitalInput headingbutton;
	Slider slider;
	Elevator elevator;

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

		dt = new DriveTrain();
	}

	@Override
	public void autonomousPeriodic() {
		
	}

	@Override
	public void teleopInit() {
		//heading.reset();
		//heading.setHeadingHold(true);
	}
	@Override
	public void teleopPeriodic() {
		double leftJoystickX = driver.getX(GenericHID.Hand.kLeft);
		double leftJoystickY = -driver.getY(GenericHID.Hand.kLeft);
		dt.accelDrive(leftJoystickY, leftJoystickX);

		SmartDashboard.putNumber("Xbox Left; X", leftJoystickX);
		SmartDashboard.putNumber("Xbox Left; Y", leftJoystickY);
		SmartDashboard.putNumber("PID", heading.turnRate());
	}

	
	
}