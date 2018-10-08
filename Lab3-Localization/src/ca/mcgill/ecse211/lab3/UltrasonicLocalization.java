package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalization {
	public static UltrasonicLocalization instance;
	public enum UltrasonicLocalizationMode{FALLING_EDGE, RISING_EDGE}
	public static int WALL_DIS = 50;
	public static int WIDTH = 3;
	private static final int FILTER_OUT = 40;               //filter out amount of distance that to far
	private int filterControl;                             //control variable for recording how long the 

	
	private UltrasonicLocalizationMode mode;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private SampleProvider ultrasonicSample;
	private float[] distanceData;
	private float lastDistance;
	private Odometer odo;
	
	
	public UltrasonicLocalization(UltrasonicLocalizationMode mode, SampleProvider sample,
							EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
		this.mode = mode;
		this.ultrasonicSample = sample;
		distanceData = new float[sample.sampleSize()];
		filterControl = 0;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		odo = Odometer.getOdometer();
		instance = this;
	}
	
	public void performLocalization() {
		leftMotor.setAcceleration(300);
		rightMotor.setAcceleration(300);
		leftMotor.setSpeed(Lab4.FORWARD_SPEED);
		rightMotor.setSpeed(Lab4.FORWARD_SPEED);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
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
			
			leftMotor.rotate(Lab4.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaTheta), true);
			rightMotor.rotate(-Lab4.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaTheta), false);
		}else {
			rotateToFallingEdge(false);
			
			double angleA = odo.getXYT()[2];  ///get the angle a
			
			rotateToFallingEdge(true);
			
			double angleB = odo.getXYT()[2];  ///get the angle b
			
			double averageAngle = (angleA+angleB)/2;
			double deltaTheta;
			
			if(angleA<angleB) {
				deltaTheta = angleB + (0 - averageAngle);
			} else {
				deltaTheta = angleB + (180 - averageAngle);
		    }
			
			leftMotor.rotate(Lab4.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaTheta), true);
			rightMotor.rotate(-Lab4.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaTheta), false);
		}
		
		odo.setTheta(0);
	}
	
	private void rotateToFallingEdge(boolean counterClockwise) {
		//the robot will facing the wall at the beginning, so rotate util
		//its not facing a wall
		while(getDistanceData() < WALL_DIS + WIDTH) {
			if(counterClockwise) {
				rightMotor.forward();
				leftMotor.backward();
			}else {
				rightMotor.backward();
				leftMotor.forward();
			}
		}
		Sound.beep();
		//then rotate the robot to find a rising edge (facing the wall)
		while(getDistanceData() > WALL_DIS) {
			if(counterClockwise) {
				rightMotor.forward();
				leftMotor.backward();
			}else {
				rightMotor.backward();
				leftMotor.forward();
			}
		}
		rightMotor.stop(true);
		leftMotor.stop(true);
		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void rotateToRisingEdge(boolean counterClockwise) {
		//the robot will facing the wall at the beginning, so rotate util
		//its not facing a wall
		lastDistance = 100;
		while(getDistanceData() > WALL_DIS) {
			if(counterClockwise) {
				rightMotor.forward();
				leftMotor.backward();
			}else {
				rightMotor.backward();
				leftMotor.forward();
			}
		}
		Sound.beep();
		//then rotate the robot to find a rising edge (facing the wall)
		while(getDistanceData() < WALL_DIS + WIDTH) {
			if(counterClockwise) {
				rightMotor.forward();
				leftMotor.backward();
			}else {
				rightMotor.backward();
				leftMotor.forward();
			}
		}
		rightMotor.stop(true);
		leftMotor.stop(true);
		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
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
		return distanceToReturn;
	}
}
