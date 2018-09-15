package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int GAIN = 100;
	private static final int MAX_GAIN = 150;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// TODO: process a movement based on the us distance passed in (P style)
		int diff = bandCenter - distance;

		// if the distance is invalid (too far) nor the diff is within
		// the bandwidth, keep the current speed
		if (this.distance >= 255 || Math.abs(diff) < this.bandWidth) {
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		} else {
			int gain = getGain(diff);
			int rightSpeed = MOTOR_SPEED + gain;
			int leftSpeed = MOTOR_SPEED - gain;
			// too far to the wall
			WallFollowingLab.leftMotor.setSpeed(leftSpeed);
			WallFollowingLab.rightMotor.setSpeed(leftSpeed);
		}
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	public int getGain(int diff) {
		// consider different formulas
		int sign = diff >= 0? 1 : -1;
		int thisGain = GAIN*diff;
		return (Math.abs(thisGain) < MAX_GAIN)? thisGain : sign*MAX_GAIN;
	}
}
