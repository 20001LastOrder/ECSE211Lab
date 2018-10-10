package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

/**
 * Class to perform the color navigation
 * @author Percy Chen 260727955
 * @author Anssam Ghezala 260720743
 *
 */

public class LightLocalization {
	private static final int SLEEP_TIME = 50;
	private static final int ROTATION_SPEED = 50;
	private static final int CORRECT_ANGLE_FACTOR = 265; 
	private static final double GO_BACK_DISTANCE = 0.7;
	private static final double CHANGE_IN_PERCENT = -35; //change percentage for the color sensor to detect black line
	private static final double CHANGE_THRESHOLD = 30;  //Change threshold for the color sensor to detect black line
	private static final double SENSOR_DISTANCE = 14.5; // distance from sensor to the rotation center

	private Odometer odo;
	private Navigation nav;
	private SampleProvider colorSample;
	private double lastBrightnessLevel;
	private double currBrightnessLevel;
	private float[] RGBValues;                         //use to fetch RGB values from color sensor

	/**
	 * Constructor for lightLocalization
	 * 
	 * @param provider: color sensor sample provider
	 * @param nav: robot navigation
	 * @throws OdometerExceptions: Exception of odometer
	 */
	public LightLocalization(SampleProvider provider, Navigation nav) throws OdometerExceptions {
		this.colorSample = provider;
		this.odo = Odometer.getOdometer();
		RGBValues = new float[colorSample.sampleSize()];
		this.nav = nav;
	}

	/**
	 * Perform light sensor localization (both position and rotation)
	 * 
	 * @throws OdometerExceptions: Exception for the odometer
	 */
	public void performLocalization() throws OdometerExceptions {

		// turn 40 degree relative to current rotation (likely to be 0)
		nav.turnTo(45, odo.getXYT()[2]);

		// first go forward until reaches a black line
		while (!isBlackLineTriggered()) {
			nav.forward();
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		nav.stop();

		// Then go back until the center of the robot rotation is in the position to
		// perform color
		// sensor localization
		double[] position = odo.getXYT();
		nav.travelTo(position[0] - GO_BACK_DISTANCE, position[1] - GO_BACK_DISTANCE);

		// go back to 0 rotation to prepare for the localization
		nav.turnTo(0, odo.getXYT()[2]);

		// prepare input for the localization, rotate to detect 4 black lines and record
		// the angle
		double thetaX1, thetaX2, thetaY1, thetaY2;
		thetaX1 = getRorationOnBlackLine();
		thetaY1 = getRorationOnBlackLine();
		thetaX2 = getRorationOnBlackLine();
		thetaY2 = getRorationOnBlackLine();
		nav.turnTo(thetaY2, odo.getXYT()[2]);

		// start using formula to calculate deltaX and deltaY to the origin, as well as
		// a more
		// precise angle
		double thetaY = (thetaY2 - thetaY1) / 2;
		double thetaX = (thetaX2 - thetaX1) / 2;
		double deltaX = -SENSOR_DISTANCE * Math.cos(Math.toRadians((thetaY)));
		double deltaY = -SENSOR_DISTANCE * Math.cos(Math.toRadians((thetaX)));
		double deltaTheta = 90 + deltaY - (thetaY2 - 180);

		// correct to 0 rotation
		nav.rotate(deltaTheta - CORRECT_ANGLE_FACTOR);
		odo.setTheta(0);

		// go to origin and turn to 0 rotation
		odo.setXYT(deltaX / Lab4.TILE_SIZE, deltaY / Lab4.TILE_SIZE, 0);
		nav.travelTo(0, 0);
		nav.turnTo(0, odo.getXYT()[2]);
	}

	/**
	 * Method to get the rotation of the robot when reach a black line
	 * 
	 * @return the angle from the odometer
	 */
	public double getRorationOnBlackLine() {
		while (!isBlackLineTriggered()) {
			nav.setSpeed(ROTATION_SPEED);
			nav.rotate(false);
		}

		return odo.getXYT()[2];
	}

	/**
	 * Check if a black line is detected by the color sensor
	 * 
	 * @return weather the balck line is detected: true/false
	 */
	private boolean isBlackLineTriggered() {
		// Trigger correction (When do I have information to correct?)
		colorSample.fetchSample(RGBValues, 0);
		currBrightnessLevel = (RGBValues[0] + RGBValues[1] + RGBValues[2]) * 100; //scale the value up by 100

		if (lastBrightnessLevel == -1) {
			lastBrightnessLevel = currBrightnessLevel;
		}

		// two conditions for blackline: abs threhold or changes in percentage
		double changesPercent = 100 * (currBrightnessLevel - lastBrightnessLevel) / lastBrightnessLevel;
		
		//draw the current brightness level on lcd for test purpose
		LCD.drawString("Co: " + currBrightnessLevel, 0, 4);
		if (changesPercent < CHANGE_IN_PERCENT || currBrightnessLevel < CHANGE_THRESHOLD) {
			Sound.beep();
			return true;
		} else {
			return false;
		}
	}
}
