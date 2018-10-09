package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalization {
	public enum UltrasonicLocalizationMode{FALLING_EDGE, RISING_EDGE}
	public static int WALL_DIS = 50;
	public static int WIDTH = 3;
	private static final int FILTER_OUT = 40;               //filter out amount of distance that to far
	private int filterControl;                             //control variable for recording how long the 

	
	private UltrasonicLocalizationMode mode;
	private SampleProvider ultrasonicSample;
	private float[] distanceData;
	private float lastDistance;
	private Odometer odo;
	private Navigation nav;
	
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
			
			double averageAngle = (angleA+angleB)/2;
			double deltaTheta;
			
			if(angleA<angleB) {
				deltaTheta = angleB + (135 - averageAngle);
			} else {
				deltaTheta = angleB + (315 - averageAngle);
		    }
			
			nav.rotate(deltaTheta);
		}else {
			rotateToFallingEdge(false);
			
			double angleA = odo.getXYT()[2];  ///get the angle a
			
			rotateToFallingEdge(true);
			
			double angleB = odo.getXYT()[2];  ///get the angle b
			
			double averageAngle = (angleA+angleB)/2;
			double deltaTheta;
			
			if(angleA<angleB) {
				deltaTheta = angleB + (135 - averageAngle);
			} else {
				deltaTheta = angleB + (180 - averageAngle);
		    }
			
			nav.rotate(deltaTheta);
		}
		
		odo.setTheta(0);
	}
	
	private void rotateToFallingEdge(boolean counterClockwise) {
		//the robot will facing the wall at the beginning, so rotate util
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
		lastDistance = 100;
		while(getDistanceData() > WALL_DIS+WIDTH) {
			nav.rotate(counterClockwise);
		}
		Sound.beep();
		//then rotate the robot to find a rising edge
		while(getDistanceData() < WALL_DIS + WIDTH) {
			nav.rotate(counterClockwise);

		}
	}
	
	public float getDistanceData() {
		ultrasonicSample.fetchSample(distanceData, 0);
		float distance = distanceData[0]*100;
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
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			distanceToReturn = distance;
			lastDistance = distance;
		}
	    LCD.drawString("D: " +distanceToReturn, 0, 3);
		return distanceToReturn;
	}
}
