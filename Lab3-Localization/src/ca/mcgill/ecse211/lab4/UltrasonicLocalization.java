package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Class to perform the navigation
 * @author Percy Chen 260727955
 * @author Anssam Ghezala 260720743
 *
 */
public class UltrasonicLocalization {
	public enum UltrasonicLocalizationMode{FALLING_EDGE, RISING_EDGE}
	public static final int WALL_DIS = 50;                 //Furtherest distance to see a wall
	public static final int WIDTH = 3;                     //acceptable range
	private static final int FILTER_OUT = 20;               //filter out amount of distance that to far
	private static final int RISING_EDGE_SMALL_ANGLE = 135; //angle for performing localization when a  < b
															//135 value got from doing multiple trials
	private static final int RISING_EDGE_LARGE_ANGLE = 315; //angle for performing localization when a  > b
															//315 value got from doing multiple trials	
	private static final int FALLING_EDGE_SMALL_ANGLE = 135; //angle for performing localization when a  < b
															//135 value got from doing multiple trials
	private static final int FALLING_EDGE_LARGE_ANGLE = 180; //angle for performing localization when a  > b
	private static final int OUT_WALL_DISTANCE = 100;														//180 value got from doing multiple trials
	
	private UltrasonicLocalizationMode mode;
	private SampleProvider ultrasonicSample;
	private float[] distanceData;
	private float lastDistance;
	private Odometer odo;
	private Navigation nav;
	private int filterControl;                             //control variable for recording how long the 
	
	/** Construction for localization class
	 * @param mode: Rising of Falling edge mode
	 * @param sample: US sample provider 
	 * @param nav: navigation class
	 * @throws OdometerExceptions
	 */
	public UltrasonicLocalization(UltrasonicLocalizationMode mode, SampleProvider sample,
							Navigation nav) throws OdometerExceptions {
		this.mode = mode;
		this.ultrasonicSample = sample;
		distanceData = new float[sample.sampleSize()];
		filterControl = 0;
		this.nav = nav;
		odo = Odometer.getOdometer();
	}
	
	public void performLocalization() {
		nav.setAcc(300);
		nav.setSpeed(100);
		
		if(mode == UltrasonicLocalizationMode.RISING_EDGE) {
			//fisrt rotate counterclockwise
			rotateToRisingEdge(false);
			
			double angleA = odo.getXYT()[2];  ///get the angle a
			
			rotateToRisingEdge(true);
			
			double angleB = odo.getXYT()[2];  ///get the angle b
			
			double averageAngle = (angleA+angleB)/2; //get average of angles
			double deltaTheta; //wanted angle
			
			if(angleA<angleB) { // set values of deltaTheta depending on whether a < b
				deltaTheta = angleB + (RISING_EDGE_SMALL_ANGLE - averageAngle); 
			} else {
				deltaTheta = angleB + (RISING_EDGE_LARGE_ANGLE - averageAngle); //315 value got from doing multiple trials
		    }
			
			nav.rotate(deltaTheta); //rotate to wanted angle
		}else {
			rotateToFallingEdge(false);
			
			double angleA = odo.getXYT()[2];  ///get the angle a
			
			rotateToFallingEdge(true);
			
			double angleB = odo.getXYT()[2];  ///get the angle b
			
			double averageAngle = (angleA+angleB)/2;
			double deltaTheta;
			
			if(angleA<angleB) {// set values of deltaTheta depending on whether a < b
				deltaTheta = angleB + (FALLING_EDGE_SMALL_ANGLE - averageAngle); //135 value got from doing multiple trials
			} else {
				deltaTheta = angleB + (FALLING_EDGE_LARGE_ANGLE - averageAngle); //180 value got from doing multiple trials
		    }
			
			nav.rotate(deltaTheta);//rotate to wanted angle
		}
		
		odo.setTheta(0); //set angle theta of odometer back to 0 (once it's at the wanted angle)
	}
	
	/** Implements Falling Edge rotation
	 * @param counterClockwise: which way to rotate (clockwise if false, counter clockwise if true) 
	 */
	private void rotateToFallingEdge(boolean counterClockwise) {
		//the robot will facing the wall at the beginning, so rotate until
		//its not facing a wall
		while(getDistanceData() < WALL_DIS + WIDTH) {
			nav.rotate(counterClockwise);
		}
		Sound.beep();
		//then rotate the robot to falling a rising edge (facing the wall)
		while(getDistanceData() > WALL_DIS+WIDTH) {
			nav.rotate(counterClockwise);
		}
	}
	
	private void rotateToRisingEdge(boolean counterClockwise) {
		//the robot will not facing the wall at the beginning
		lastDistance = OUT_WALL_DISTANCE;
		//rotate until facing the wall
		while(getDistanceData() > WALL_DIS+WIDTH) {
			nav.rotate(counterClockwise);
		}
		Sound.beep();
		//then rotate the robot to find a rising edge
		while(getDistanceData() < WALL_DIS + WIDTH) {
			nav.rotate(counterClockwise);

		}
	}
	
	
	/** filters sampled distances for better reading
	 * @return distanceToReturn: distance read from sensor
	 */
	public float getDistanceData() {
		ultrasonicSample.fetchSample(distanceData, 0);
		float distance = distanceData[0]*100;    //change the unit from m to cm
		float distanceToReturn;
		if (distance >= WALL_DIS + WIDTH && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
			distanceToReturn = lastDistance;
		} else if (distance >= WALL_DIS + WIDTH) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			distanceToReturn = distance;
			lastDistance = distance;
		}else {
			// distance too low: reset filter and leave
			// distance alone.
			filterControl = 0;
			distanceToReturn = distance;
			lastDistance = distance;
		}
	    LCD.drawString("D: " +distanceToReturn, 0, 3);
		return distanceToReturn;
	}
}
