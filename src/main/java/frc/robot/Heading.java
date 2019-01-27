package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class Heading {
    private AHRS ahrs; // This thingy drifted ~3 degrees per 10 min

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

    public float GetCompass (){
        return ahrs.getCompassHeading();
    }
}