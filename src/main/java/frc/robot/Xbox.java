package frc.robot;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Represents an Xbox controller interface.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Jacob Cote
 * @author Brent Roberts
 */
public class Xbox extends XboxController {
	private Map<String, Supplier<Boolean>> functionMap = new HashMap<String, Supplier<Boolean>>();
	private Map<String, Boolean> whenMap = new HashMap<String, Boolean>();
	private Map<String, Boolean> fallingMap = new HashMap<String, Boolean>();
	
	/**
	 * Instantiates the controller on the specified port.
	 * 
	 * @param port the port of the controller.
	 */
	public Xbox(int port) {
		super(port);
		setupFunctions();
	}
	
	/**
	 * The controllable deadzone for the controller.
	 * 
	 * @param input the value of the interface in use.
	 * @param deadzone the absolute value of the deadzone.
	 * @return the input value with deadzone applied.
	 */
	public double deadzone(double input, double deadzone) {
		if (Math.abs(input) < deadzone) {
			return(0);
		} else {
			return(input);
		}
	}
	
	/**
	 * The universal deadzone for the controller.
	 * Uses a deadzone of .2.
	 * 
	 * @param input the value of the interface in use.
	 * @return the input value with deadzone applied.
	 */
	public double deadzone(double input) {
		return deadzone(input, 0.2);
	}
	
	/**
	 * Gets the value of the right trigger with deadzone.
	 * 
	 * @return the value of the right trigger from the deadzone to 1.0.
	 */
	public double getRightTrigger() {
		return deadzone(getTriggerAxis(GenericHID.Hand.kRight));
	}
	
	/**
	 * Gets the value of the left trigger with deadzone.
	 * 
	 * @return the value of the left trigger from the deadzone to 1.0.
	 */
	public double getLeftTrigger() {
		return deadzone(getTriggerAxis(GenericHID.Hand.kLeft));
	}
	
	/**
	 * Returns whether or not the specified button is pressed.
	 * 
	 * @param button the button to check.
	 * @return whether or not the button is pressed.
	 */
	public boolean getPressed(String button) {
		if (functionMap.containsKey(button)) {
			return functionMap.get(button).get();
		}
		return false;
	}
	
	/**
	 * Returns the rising edge of a button press.
	 * 
	 * @param button the button to check rising edge for.
	 * @return whether or not a rising edge was detected.
	 */
	public boolean when(String button) {
		//TODO: Debounce buttons
		if (!whenMap.containsKey(button)) {
			return false;
		}
		
		if (getPressed(button)) {
			if (!whenMap.get(button)) {
				whenMap.put(button, true);
				return true;
			}
		}
		else {
			whenMap.put(button, false);
		}
		return false;
	}
	
	/**
	 * Returns the falling edge of an button.
	 * 
	 * @param button the button to check the falling edge for.
	 * @return whether or not an falling edge was detected.
	 */
	public boolean falling(String button) {
		if (!fallingMap.containsKey(button)) {
			Common.debug("falling map does not contain"+button);
			return false;
		}
		if (fallingMap.get(button)) {
			if (!getPressed(button)) {
				fallingMap.put(button, false);
				return true;
				}
			else {
				fallingMap.put(button, true);
				return false;
				}
			}
		else {
			if (getPressed(button)) {
				fallingMap.put(button, true);
				return false;
			} else {
				fallingMap.put(button, false);
				return false;
			}
		}
	}
	
	/**
	 * Maps superclass button functions to strings and sets up built-in deadzones.
	 */
	private void setupFunctions() {
		//Changed to private because I didn't see an reason to be public -Brent 10/11/18
		functionMap.put("a", this::getAButton);
		whenMap.put("a", false);
		fallingMap.put("a", false);
		
		functionMap.put("b", this::getBButton);
		whenMap.put("b", false);
		fallingMap.put("b", false);
		
		functionMap.put("x", this::getXButton);
		whenMap.put("x", false);
		fallingMap.put("x", false);
		
		functionMap.put("y", this::getYButton);
		whenMap.put("y", false);
		fallingMap.put("y", false);
		
		functionMap.put("start", this::getStartButton);
		whenMap.put("start", false);
		fallingMap.put("start", false);
		
		functionMap.put("back", this::getBackButton);
		whenMap.put("back", false);
		fallingMap.put("back", false);
		
		functionMap.put("dPadUp", () -> {
			return (this.getPOV() == -1) ? false : Math.abs(0 - this.getPOV()) < 45 || Math.abs(360 - this.getPOV()) < 45;
		});
		whenMap.put("dPadUp", false);
		fallingMap.put("dPadUp", false);
		
		functionMap.put("dPadRight", () -> {
			return (this.getPOV() == -1) ? false : Math.abs(90 - this.getPOV()) < 45;
		});
		whenMap.put("dPadRight", false);
		fallingMap.put("dPadRight", false);
		
		functionMap.put("dPadDown", () -> {
			return (this.getPOV() == -1) ? false : Math.abs(180 - this.getPOV()) < 45;
		});
		whenMap.put("dPadDown", false);
		fallingMap.put("dPadDown", false);
		
		functionMap.put("dPadLeft", () -> {
			return (this.getPOV() == -1) ? false : Math.abs(270 - this.getPOV()) < 45;
		});
		whenMap.put("dPadLeft", false);
		fallingMap.put("dPadLeft", false);
		
		functionMap.put("leftBumper", () -> {
			return this.getBumper(GenericHID.Hand.kLeft);
		});
		whenMap.put("leftBumper", false);
		fallingMap.put("leftBumper", false);
		
		functionMap.put("rightBumper", () -> {
			return this.getBumper(GenericHID.Hand.kRight);
		});
		whenMap.put("rightBumper", false);
		fallingMap.put("rightBumper", false);
		
		functionMap.put("leftTrigger", () -> {
			return deadzone(this.getLeftTrigger()) > 0;
		});
		whenMap.put("leftTrigger", false);
		fallingMap.put("leftTrigger", false);
		
		functionMap.put("rightTrigger", () -> {
			return deadzone(this.getRightTrigger()) > 0;
		});
		whenMap.put("rightTrigger", false);
		fallingMap.put("rightTrigger", false);
		
		functionMap.put("rightThumb", () -> {
			return this.getRawButton(10);
		});
		whenMap.put("rightThumb", false);
		fallingMap.put("rightThumb", false);

		functionMap.put("leftThumb", () -> {
			return this.getRawButton(9);
		});
		whenMap.put("leftThumb", false);
		fallingMap.put("leftThumb", false);
	}
}
