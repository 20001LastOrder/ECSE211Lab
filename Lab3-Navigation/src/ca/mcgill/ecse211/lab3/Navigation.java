package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static double DEGREE_TO_RADIAN_FACTOR = 180/Math.PI;
	private static boolean isNavigating;
		
	public static void driveTo(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
						int x, int y) throws OdometerExceptions {
		if(isNavigating) {
			return;
		}
		isNavigating = true;
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(300);
		}
		
		double[] currentPosition = Odometer.getOdometer().getXYT();
		double deltaX = x-currentPosition[0];
		double deltaY = y-currentPosition[1];

		double distance = Math.sqrt(Math.pow(deltaX, 2)+Math.pow(deltaY, 2));
		
		double rotation = Math.atan2(deltaX, deltaY);
		
		rotateTo(leftMotor, rightMotor, rotation * DEGREE_TO_RADIAN_FACTOR, currentPosition[2]);
		
		leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);

	    leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * Lab3.TILE_SIZE), true);
	    rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * Lab3.TILE_SIZE), false);
	    isNavigating = false;
	}
	
	public static boolean isNavigating() {
		return isNavigating;
	}
	
	public static void rotateTo(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			double rotation, double theta) {
		double angle = rotation - theta;
		int leftSpeedFactor = 1;
		int rightSpeedFactor = -1;
		
		if(angle > 180) {
			angle = 360 - angle;
			leftSpeedFactor = -1;
			rightSpeedFactor = 1;
		}
		
		leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(leftSpeedFactor*convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, angle), true);
	    rightMotor.rotate(rightSpeedFactor*convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, angle), false);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
