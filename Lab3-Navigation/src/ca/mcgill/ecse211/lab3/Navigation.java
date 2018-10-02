package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Navigation extends Thread {
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 300;
	private static double DEGREE_TO_RADIAN_FACTOR = 180 / Math.PI;
	private static boolean isNavigating;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private UltrasonicPoller poller;
	private static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	private int sensorRotation;
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, UltrasonicPoller poller) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.poller = poller;
		sensorRotation = 0;
	}

	public void run() {
		try {
			if(poller == null) {
				driveTo(leftMotor, rightMotor, 0, 2);
				driveTo(leftMotor, rightMotor, 1, 1);
				driveTo(leftMotor, rightMotor, 2, 2);
				driveTo(leftMotor, rightMotor, 2, 1);
				driveTo(leftMotor, rightMotor, 0, 0);
			} else {
			driveTo(leftMotor, rightMotor, 0, 2,poller);
			driveTo(leftMotor, rightMotor, 1, 1,poller);
			driveTo(leftMotor, rightMotor, 2, 2,poller);
			driveTo(leftMotor, rightMotor, 2, 1,poller);
			driveTo(leftMotor, rightMotor, 0, 0,poller);
			}
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void driveTo(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int x, int y)
			throws OdometerExceptions {
		if (isNavigating) {
			return;
		}
		isNavigating = true;
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(300);
		}

		double[] currentPosition = Odometer.getOdometer().getXYT();
		double deltaX = x - currentPosition[0];
		double deltaY = y - currentPosition[1];

		double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		double rotation = Math.atan2(deltaX, deltaY);

		rotateTo(leftMotor, rightMotor, rotation * DEGREE_TO_RADIAN_FACTOR, currentPosition[2]);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * Lab3.TILE_SIZE), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * Lab3.TILE_SIZE), false);
		isNavigating = false;
	}

	public void driveTo(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int x, int y,
			UltrasonicPoller poller) throws OdometerExceptions {
		
		
		if (isNavigating) {
			return;
		}
		isNavigating = true;
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(300);
		}
		
		double[] currentPosition = Odometer.getOdometer().getXYT();
		double deltaX = x - currentPosition[0];
		double deltaY = y - currentPosition[1];
		double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

		double rotation = Math.atan2(deltaX, deltaY);
		
		rotateTo(leftMotor, rightMotor,rotation * DEGREE_TO_RADIAN_FACTOR, currentPosition[2]);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * Lab3.TILE_SIZE), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance * Lab3.TILE_SIZE), true);
		
		while (leftMotor.isMoving() && rightMotor.isMoving()) {

			if (poller.getDistance() < 20 && poller.getDistance()<distance* Lab3.TILE_SIZE) {
				double curX = Odometer.getOdometer().getXYT()[0];
				double curY = Odometer.getOdometer().getXYT()[1];
				int leftFactor = 1;
				int rightFactor = -1;
				//reset sensor rotation
				if(sensorRotation == -45) {
					leftFactor = -1;
					rightFactor = 1;
				}
				sensorRotation = leftFactor* 90;
				sensorMotor.rotate(sensorRotation);
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);

				leftMotor.rotate(leftFactor * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);
				rightMotor.rotate(rightFactor * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);

				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);

				leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 15), true);
				rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 15), false);
				
				do {
					if(poller.getDistance() > 20) {
						leftMotor.setSpeed(FORWARD_SPEED-(leftFactor*100));
						rightMotor.setSpeed(FORWARD_SPEED-(rightFactor*100));
					} else {
						leftMotor.setSpeed(FORWARD_SPEED);
						rightMotor.setSpeed(FORWARD_SPEED);
					}
					leftMotor.forward();
					rightMotor.forward();
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}while(Math.abs(curX - Odometer.getOdometer().getXYT()[0]) > 0.30 &&
						Math.abs(curY - Odometer.getOdometer().getXYT()[0]) > 0.30); 
				
			
				
				/**				leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);
				rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
				leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 10), true);
				rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 10), false);
				while(poller.getDistance() <15) {
					leftMotor.forward();
					rightMotor.forward();
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				} **/
				sensorMotor.rotate(-sensorRotation);
				
				isNavigating = false;
				driveTo(leftMotor, rightMotor, x, y, poller);
				sensorRotation = 0;
				//sensorMotor.rotate(sensorRotation);
			}
		}
		isNavigating = false;
	}

	public static boolean isNavigating() {
		return isNavigating;
	}

	public void rotateTo(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double rotation,
			double theta) {
		double angle = rotation - theta;
		int leftSpeedFactor = 1;
		int rightSpeedFactor = -1;
		if (angle < 0) {
			angle = angle + 360;
		}
		if (angle > 180) {
			angle = 360 - angle;
			leftSpeedFactor = -1;
			rightSpeedFactor = 1;
		}

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(leftSpeedFactor * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, angle), true);
		rightMotor.rotate(rightSpeedFactor * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, angle), false);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
