package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class Heading {
    private static final double P = 0.025, I = 0, D = 1.5;

    private AHRS ahrs; // This thingy drifted ~3 degrees per 10 min

    private PID headingPID;

    public Heading(){
        ahrs = new AHRS(SPI.Port.kMXP); //NavX
        resetGyros();

        headingPID = new PID(P, I, D, true, false, "NavX PID");
        headingPID.setOutputLimits(-1, 1);
        headingPID.setTarget(0.0);
    }

    public double getNav () {
        return ahrs.getAngle();
    }

    public void resetGyros (){
        ahrs.reset();
    }

    public float getCompass (){
        return ahrs.getCompassHeading();
    }

    public void setPID(float p, float i, float d){
        headingPID.setP(p);
        headingPID.setI(i);
        headingPID.setD(d);
    }

    public double getTargetAngle (){
        return headingPID.getTarget();
    }
    public void setTargetAngle (double target){
        headingPID.setTarget(target);
    }

}