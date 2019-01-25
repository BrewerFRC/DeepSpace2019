package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;;

public class BasicDrive {
    private static Spark
        frontL = new Spark(Constants.DRIVE_FL),
        frontR = new Spark(Constants.DRIVE_FR),
        backL = new Spark(Constants.DRIVE_BL),
        backR = new Spark(Constants.DRIVE_BR);

    private static final SpeedControllerGroup left = new SpeedControllerGroup(frontL, backL);
    private static final SpeedControllerGroup right = new SpeedControllerGroup(frontR, backR);

    private static final double maxSpeed = 0.8f;

    public BasicDrive(){

    }

    public void accelDrive (double forward, double rad){
        /*double m_speed  = Math.max(Math.min(forward, maxSpeed), -maxSpeed);
        double m_leftpower = 
        left.set(speed);*/
    }

}