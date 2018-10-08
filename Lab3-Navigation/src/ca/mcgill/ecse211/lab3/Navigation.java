package ca.mcgill.ecse211.lab3;

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
public class Navigation extends Thread {
	//CONSTANT VALUES
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 300;
	private static double DEGREE_TO_RADIAN_FACTOR = 180 / Math.PI;
	private static final int MOTOR_ACC = 300;
	private static final int ROTATE_RIGHT_ANGLE = 90;
	private static final int OBSTACLE_AVOID_DIS = 15;
	private static final int ROTATE_SPEED_ADD = 100;
	private static final int DANGEROUS_DIS = 20;
	private static final int THREAD_SLEEP_TIME = 10;
	private static final double OBSTACLE_ALLOWED_ERR_X = 0.30;
	private static final double OBSTACLE_ALLOWED_ERR_Y = 0.30;
	private static final double SQUARE = 2;
	private static final double MAX_ANGLE = 360;
	// sense we only use the medium motor here, put it directly in this class
	private static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	
	//check if the robot is current navigating
	private static boolean isNavigating;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private UltrasonicPoller poller;
	private int[][] destinations;

	//rotation for the ultrasonic sensor
	private int sensorRotation;

	/**
	 * Constructor for the navigations class
	 * @param leftMotor: abstraction for the left motor of the robot
	 * @param rightMotor: abstraction for the right motor of the robot
	 * @param poller: ultrasonic poller to get the data from ultrasonic sensor, null if not use
	 * @param destinations: destinations for the robot to reach
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, UltrasonicPoller poller,
			int[][] destinations) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.poller = poller;
		this.destinations = destinations;
		sensorRotation = 0;
	}
	
	/**
	 *  main method for the navigation thread to run
	 */
	public void run() {
		try {
			// drive the robot to destinations one by one
			Thread.sleep(1000);
			for (int[] destinate : destinations) {
				travelTo(destinate[0], destinate[1], poller);
			}
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Drive the robot to required destination
	 * @param x: x value of the destination
	 * @param y: y value of the destination
	 * @param poller: poller: ultrasonic poller to get the data from ultrasonic sensor, null if not use
	 * @throws OdometerExceptions: exception related to the Odometer
	 */
	public void travelTo(int x, int y, UltrasonicPoller poller)throws OdometerExceptions {
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
		
		//if no poller passed (no use of ultrasonic sensor), just wait for to the destination
		if(poller == null) {
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, motions[1] * Lab3.TILE_SIZE), true);

			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, motions[1] * Lab3.TILE_SIZE), false);
		}else {
			
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, motions[1] * Lab3.TILE_SIZE), true);
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, motions[1] * Lab3.TILE_SIZE), true);
			//immediate return to detect the obstacles
			avoidObstacle(x, y);
		}
		
		
		isNavigating = false;
	}

	/**
	 * Continuing check the data from the ultrasonic sensor to avoid the obstacles
	 * @param x: x value of destination to reach
	 * @param y: y value of the destination to reach
	 * @throws OdometerExceptions: exception related to the Odometer
	 */
	private void avoidObstacle(int x, int y) throws OdometerExceptions {
		while (leftMotor.isMoving() && rightMotor.isMoving()) {
			double[] position = Odometer.getOdometer().getXYT();
			double curX = position[0];
			double curY = position[1];
			double distance = this.calculateMotion(x, y, position)[1];
			//when the sensor detects the obstacle that is on the way to the destination, do corrections
			if (poller.getDistance() < DANGEROUS_DIS && poller.getDistance() < distance * Lab3.TILE_SIZE) {
				
				sensorRotation = 90;
				//rotate sensor to facing the obstacle
				sensorMotor.rotate(sensorRotation);
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				
				// rotate to the right
				leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ROTATE_RIGHT_ANGLE), true);
				rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, ROTATE_RIGHT_ANGLE), false);

				//forward some distance to not touch the obstacle
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);
//				double[] lastDis = Odometer.getOdometer().getXYT();
//				leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, OBSTACLE_AVOID_DIS), true);
//				rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, OBSTACLE_AVOID_DIS), false);
//				double[] curDis = Odometer.getOdometer().getXYT();
//				boolean isAvoiding = true;
//				boolean checkX = true;
//				double deltaX = Math.abs(curDis[0] - lastDis[0]);
//				double deltaY = Math.abs(curDis[1] - lastDis[1]);
//				if(deltaX < deltaY) {
//					checkX = false;
//				}
				//go around the obstacle to avoid it, and wait for the robot to cycle the obstacle
				//until it reaches the back of the obstacle
				//use do-while to make sure it at least move it's position
				int time = 0;
				do {
					if (poller.getDistance() > DANGEROUS_DIS) {
						leftMotor.setSpeed(FORWARD_SPEED - ROTATE_SPEED_ADD);
						rightMotor.setSpeed(FORWARD_SPEED + ROTATE_SPEED_ADD);
					} else {
						leftMotor.setSpeed(FORWARD_SPEED);
						rightMotor.setSpeed(FORWARD_SPEED);
					}
					leftMotor.forward();
					rightMotor.forward();
					time += THREAD_SLEEP_TIME;
					try {
						Thread.sleep(THREAD_SLEEP_TIME);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				} while (time < 5000);
				
				//rotate back the sensor to the original position
				sensorMotor.rotate(-sensorRotation);
				sensorRotation = 0;

				//finish this navigation, restart a new navigation to the destination
				isNavigating = false;
				travelTo(x, y, poller);
			}
		}
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
	private double[] calculateMotion(int x, int y, double[] currentPosition) throws OdometerExceptions {
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
		leftMotor.rotate(leftSpeedFactor * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, angle), true);
		rightMotor.rotate(rightSpeedFactor * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, angle), false);
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
