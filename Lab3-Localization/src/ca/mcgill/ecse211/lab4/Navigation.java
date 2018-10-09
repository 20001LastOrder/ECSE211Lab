package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * Class to perform the navigation
 * @author Percy Chen 260727955
 * @author Anssam Ghezala 260720743
 *
 */
public class Navigation{
	//CONSTANT VALUES
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 300;
	private static double DEGREE_TO_RADIAN_FACTOR = 180 / Math.PI;
	private static final int MOTOR_ACC = 300;
	private static final double SQUARE = 2;
	private static final double MAX_ANGLE = 360;
	
	//check if the robot is current navigating
	private static boolean isNavigating;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;


	/**
	 * Constructor for the navigations class
	 * @param leftMotor: abstraction for the left motor of the robot
	 * @param rightMotor: abstraction for the right motor of the robot
	 * @param poller: ultrasonic poller to get the data from ultrasonic sensor, null if not use
	 * @param destinations: destinations for the robot to reach
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	/**
	 * Drive the robot to required destination
	 * @param x: x value of the destination
	 * @param y: y value of the destination
	 * @param poller: poller: ultrasonic poller to get the data from ultrasonic sensor, null if not use
	 * @throws OdometerExceptions: exception related to the Odometer
	 */
	public void travelTo(double x, double y)throws OdometerExceptions {
		if (isNavigating) {
			return;
		}
		isNavigating = true;
		// set up the motor
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(MOTOR_ACC);
		}
		double[] currentPosition = Odometer.getOdometer().getXYT(); //get current position of the robot
		
		//motion[0] = rotation to the destination, motion[1] = distance to the destination
		double[] motions = calculateMotion(x, y, currentPosition); 
		
		//rotate to current position
		turnTo(motions[0] * DEGREE_TO_RADIAN_FACTOR, currentPosition[2]);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, motions[1] * Lab4.TILE_SIZE), true);
		rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, motions[1] * Lab4.TILE_SIZE), false);
		
		
		isNavigating = false;
	}
	
	/**
	 * check if the motor is navigating
	 * @return: true if isNavigation, otherwise false
	 */
	public boolean isNavigating() {
		return isNavigating;
	}
	
	/**
	 * calculate the data needed to perform navigation
	 * @param x: x value of the destination
	 * @param y: y value of the destination
	 * @param currentPosition: current position of the motor
	 * @return: double[] with 0th to be the rotation and 1st to be the distance
	 * @throws OdometerExceptions: OdometerExceptions: exception related to the Odometer
	 */
	private double[] calculateMotion(double x, double y, double[] currentPosition) throws OdometerExceptions {
		double deltaX = x - currentPosition[0];
		double deltaY = y - currentPosition[1];

		//find the distance to the point
		double distance = Math.sqrt(Math.pow(deltaX, SQUARE) + Math.pow(deltaY, SQUARE));
		double rotation = Math.atan2(deltaX, deltaY);

		double[] motion =  {rotation, distance};
		return motion;
	}

	/**
	 * rotate to the destination
	 * @param rotation: rotation to the destination
	 * @param theta: current rotation of the robot
	 */
	public void turnTo(double rotation, double theta) {
		double angle = rotation - theta;
		//speed factor for change sign
		//default rotate to the right
		int leftSpeedFactor = 1;
		int rightSpeedFactor = -1;
		if (angle < 0) {
			angle = angle + MAX_ANGLE;
		}
		//if the angle is larger then 180, rotate to the left instead
		if (angle > MAX_ANGLE/2) {
			angle = MAX_ANGLE - angle;
			leftSpeedFactor = -1;
			rightSpeedFactor = 1;
		}

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(leftSpeedFactor * convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, angle), true);
		rightMotor.rotate(rightSpeedFactor * convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, angle), false);
	}
	
	/**
	 * Rotate theta degree
	 * @param theta
	 */
	public synchronized void rotate(double theta) {
		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
		rightMotor.rotate(-1*convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
	}
	
	/**
	 * rotate the robot
	 * @param counterClockwise
	 */
	public synchronized void rotate(boolean counterClockwise) {
		if(counterClockwise) {
			leftMotor.backward();
			rightMotor.forward();
		}else {
			leftMotor.forward();
			rightMotor.backward();
		}	
	}

	public void setSpeed(int speed) {
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
	}
	
	public void setAcc(int acc) {
		leftMotor.setAcceleration(acc);
		rightMotor.setAcceleration(acc);
	}
	
	public void forward() {
		leftMotor.forward();
		rightMotor.forward();
	}
	
	public void stop() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	
	/**
	 * convert the angle and distance to the distance covered by motor
	 * @param radius: radius of the motor
	 * @param distance: distance of the motor to go
	 * @return: distance covered by the motor
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) (((MAX_ANGLE/2)* distance) / (Math.PI * radius));
	}

	/**
	 * convert the angle of the robot to turn to the distance covered by motor
	 * @param radius: radius of the motor
	 * @param width: width of the robot
	 * @param angle angle to turn by the robot
	 * @return distance covered by the motor
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / MAX_ANGLE);
	}
}
