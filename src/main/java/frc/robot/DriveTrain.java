package frc.robot;

//import edu.wpi.first.wpilibj.CounterBase.EncodingType;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Handles actions related to robot motion.
 * Includes motor controller, encoder, and gyro instances.
 * Created January 2018
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Brent Roberts
 */
public class DriveTrain extends DifferentialDrive {
	private static DriveTrain instance;
	private static Elevator elevator;

	public final double	
		DRIVEACCELMIN = 0.05,
		DRIVEACCELMAX = 0.1,
		DRIVEMIN = 0.35,	 //Min power needed to move drive motors
		TURNACCEL = 0.04,
		TURNMAX = 1.0;
	
	//private static final double DISTANCE_PER_PULSE_L = 0.0098195208, DISTANCE_PER_PULSE_R = 0.0098293515;
	private static final WPI_TalonSRX 
			frontL = new WPI_TalonSRX(Constants.CAN_DRIVE_FL),
			frontR = new WPI_TalonSRX(Constants.CAN_DRIVE_FR),
			backL = new WPI_TalonSRX(Constants.CAN_DRIVE_BL),
			backR = new WPI_TalonSRX(Constants.CAN_DRIVE_BR); 
	private static final SpeedControllerGroup left = new SpeedControllerGroup(frontL, backL);
	private static final SpeedControllerGroup right = new SpeedControllerGroup(frontR, backR);
	
	//private Encoder encoderL, encoderR;
	//private PID pidL, pidR;
	//private Heading heading;
	//private Solenoid shifter;
	private double driveSpeed = 0, turnSpeed = 0;
	//private double tankLeft = 0, tankRight = 0;
	
	/**
	 * Creates an instance of DriveTrain.
	 * Motor controller and encoder channels are determined in Constants.
	 */
	
	public DriveTrain(Elevator elevator) {
		super(left, right);

		this.elevator = elevator;
		left.setInverted(false);
		right.setInverted(false);
		
		//pidL = new PID(0.005, 0, 0, false, true, "velL");
		//pidR = new PID(0.005, 0, 0, false, true, "velR");
		
		instance = this;
	}
	
	
	/**
	 * Returns an instance of DriveTrain which is bound to the motor controllers.
	 * Only this instance will be functional.
	 * 
	 * @return the DriveTrain instance.
	 */
	public static DriveTrain instance() {
		return instance;
	}
	
	/**
	 * Gradually accelerate to a specified drive value.
	 * 
	 * @param target - the target drive value from -1 to 1
	 * @return double - the allowed drive value for this cycle.
	 */
	public double driveAccelCurve(double target) {
		//If the magnitude of current is less than the minimum
		if (Math.abs(driveSpeed) < DRIVEMIN) {
			//Move to the lesser value of the minimum or the target, including desired direction.
			if (target > 0) {
				driveSpeed = Math.min(DRIVEMIN, target);
			}
			else {
				driveSpeed = Math.max(-DRIVEMIN, target);
			}
		}

		double accel = Common.map(elevator.percentHeight(), 0, 1, DRIVEACCELMAX, DRIVEACCELMIN);
		//If the magnitude of current is greater than the minimum
		//If the difference is greater than the allowed acceleration
		if (Math.abs(driveSpeed - target) > accel) {
			//Accelerate in the correct direction
            if (driveSpeed > target) {
                driveSpeed = driveSpeed - accel;
            } else {
                driveSpeed = driveSpeed + accel;
            }
        }
		//If the difference is less than the allowed acceleration, reach target
		else {
            driveSpeed = target;
        }
		return driveSpeed;
	 }
	 
	 /**
	  * Gradually accelerate to a specified turn value.
	  * 
	  * @param target - the target turn value from -1 to 1
	  * @return double - the allowed turn value at this cycle.
	  */
	 public double turnAccelCurve(double target) {
		 if (Math.abs(turnSpeed - target) > TURNACCEL) {
			if (turnSpeed > target) {
				turnSpeed = turnSpeed - TURNACCEL;
			} else {
				turnSpeed = turnSpeed + TURNACCEL;
			}
		} else {
			turnSpeed = target;
		}
		// Constrain to TURNMAX speed
		if (turnSpeed >= 0) {
			turnSpeed = Math.min(TURNMAX, turnSpeed);
		} else {
			turnSpeed = Math.max(-TURNMAX, turnSpeed);
		}
	    return turnSpeed;
	}
	
	//turn should be inverted on testbed -Brent
	/**
	 * Arcade drive with an acceleration curve.
	 * 
	 * @param drive - the forward/backward value from -1 to 1.
	 * @param turn - the turn value from -1 to 1.
	 */
	public void accelDrive(double drive, double turn) {
		drive = driveAccelCurve(drive);
		turn = turnAccelCurve(turn);
		arcadeDrive(drive, -turn);

		SmartDashboard.putNumber("Drive power", drive);
	}
	
	/**
	 * Shifts the drivetrain gearbox to high gear.
	 */
	/*
	public void shiftHigh() {
		shifter.set(false);
	}
	*/
	
	/**
	 * Shifts the drivetrain gearbox to low gear.
	 */
	/*
	public void shiftLow() {
		shifter.set(true);
	}
	*/
	
	/**
	 * Whether or not the drivetrain is in low gear.
	 * 
	 * @return - is low
	 */
	/*
	public boolean isShiftedLow() {
		return shifter.get();
	}
	*/
	
	/**
	 * Resets the counts of the left and right encoders.
	 */
	/*
	public void resetEncoders() {
		encoderL.reset();
		encoderR.reset();
		pidL.reset();
		pidR.reset();
	}
	*/
	
	/**
	 * Update PID tuning values from the SmartDashboard.
	 */
	/*
	public void updatePIDs() {
		pidL.update();
		pidR.update();
	}
	*/

	
	/**
	 * Get raw counts for the left encoder.
	 * 
	 * @return the counts
	 */
	/*
	public int getLeftCounts() {
		return encoderL.get();
	}
	*/
	
	/**
	 * Get the scaled distance of the left encoder.
	 * 
	 * @return the distance in inches
	 */
	/*
	public double getLeftDist() {
		return encoderL.getDistance();
	}
	*/
	
	/**
	 * Get the scaled velocity of the left encoder.
	 * 
	 * @return the velocity in inches/second
	 */
	/*
	public double getLeftVelocity() {
		return encoderL.getRate();
	}
	*/
	
	/**
	 * Get raw counts for the right encoder.
	 * 
	 * @return the counts
	 */
	/*
	public int getRightCounts() {
		return encoderR.get();
	}
	*/
	
	/**
	 * Get the scaled distance of the right encoder.
	 * 
	 * @return double - the distance in inches
	 */
	/*
	public double getRightDist() {
		return encoderR.getDistance();
	}
	*/
	
	/**
	 * Get the scaled velocity of the right encoder.
	 * 
	 * @return double - the velocity in inches/second
	 */
	/*
	public double getRightVelocity() {
		return encoderR.getRate();
	}
	*/

	/**
	 * Get the averaged counts between the two encoders.
	 * 
	 * @return int - the average counts
	 */
	/*
	public int getAverageCounts() {
		return (encoderL.get() + encoderR.get()) / 2;
	}
	*/
	
	/**
	 * Get the averaged scaled distance between the two encoders.
	 * 
	 * @return double - the average distance in inches
	 */
	/*
	public double getAverageDist() {
		return (encoderL.getDistance() + encoderR.getDistance()) / 2;
	}
	*/
	
	/**
	 * Get the averaged scaled velocity between the two encoders.
	 * 
	 * @return double - the average velocity in inches/second
	 */
	/*
	public double getAverageVelocity() {
		return (encoderL.getRate() + encoderR.getRate()) / 2;
	}
	*/
	
	/**
	 * An instance of Heading, a gyro utility and PID controller.
	 * 
	 * @return Heading - the heading instance.
	 */
	/*
	public Heading getHeading() {
		return this.heading;
	}
	*/
	

	/**
	 * An implementation of tank drive that updates current speed values used in acceleration curve methods.
	 * Does not set motors.
	 */
	/*
	@Override
	public void tankDrive(double left, double right) {
		tankLeft = left;
		tankRight = right;
	}
	*/
	
	/**
	 * Acceleration control for tank drive. Does not set motors.
	 * 
	 * @param left - the target left power.
	 * @param right - the target right power.
	 */
	/*
	public void accelTankDrive(double left, double right) {
		tankLeft = accelSide(tankLeft, left);
		tankRight = accelSide(tankRight, right);
		
		//System.out.println(tankLeft + ":" + tankRight);
	}
	*/

	/**
	 * Applies the current DriveTrain tankLeft and tankRight motor powers.
	 * Uses heading hold PID if heading hold is enabled with {@link Heading#setHeadingHold(boolean)}
	 */
	/*
	public void applyTankDrive() {
		if (heading.isHeadingHold()) {
			double turn = heading.turnRate() / 2;
			super.tankDrive(tankLeft + turn, tankRight - turn);
		}
		else {
			super.tankDrive(tankLeft, tankRight);
		}
	}
	*/

	/**
	 * A utility method that determines the change in current power, given a desired target and allowed power curve.
	 * 
	 * @param current the current motor power.
	 * @param target the target motor power.
	 * @return the new motor power.
	 */
	/*
	private double accelSide(double current, double target) {
		double TANKACCEL = getDriveAccel();
		//If the magnitude of current is less than the minimum
		if (Math.abs(current) < TANKMIN) {
			//Move to the lesser value of the minimum or the target, including desired direction.
			if (target > 0) {
				current = Math.min(TANKMIN, target);
			}
			else {
				current = Math.max(-TANKMIN, target);
			}
		}
		//If the magnitude of current is greater than the minimum
		//If the difference is greater than the allowed acceleration
		if (Math.abs(current - target) > TANKACCEL) {
			//Accelerate in the correct direction
            if (current > target) {
                current = current - TANKACCEL;
            } else {
                current = current + TANKACCEL;
            }
        }
		//If the difference is less than the allowed acceleration, reach target
		else {
            current = target;
        }
		return current;
	} 
	*/
}
