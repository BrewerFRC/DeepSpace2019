package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

public class Heading {
    private AHRS ahrs;
    private ADXRS450_Gyro gyroOld;
    public Heading(){
        ahrs = new AHRS(SPI.Port.kMXP); //NavX
        ahrs.reset();
    }
    public double GetNav () {
        return ahrs.getAngle();
    }
    public void ResetGyros (){
        ahrs.reset();
    }
}