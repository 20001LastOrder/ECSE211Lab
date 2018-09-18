package ca.mcgill.ecse211.wallfollowing;

public class BangBangController implements UltrasonicController {
	private static final int UNUSUAL_DISTANCE = 100;
	
	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int filterControl;
	static final int FILTER_OUT = 25;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.filterControl = 0;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		if (distance >= UNUSUAL_DISTANCE && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
			forward();
			return;
		} else if (distance >= UNUSUAL_DISTANCE) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// process a movement based on the us distance passed in (BANG-BANG style)
		int diff = distance-bandCenter;

		// if the diff is within
		// the bandwidth, keep the current speed
		if ( Math.abs(diff) < this.bandwidth) {
			forward();
		} else if (diff > 0) {
			turnLeft();
		} else if (diff < 0) {
			if(diff < -(bandCenter - WallFollowingLab.DANGER_DISTANCE)) {
				//if too close, go backwards
				backward();
			}else {
				turnRight();
			}
		}
	}

	public void forward() {
		WallFollowingLab.leftMotor.setSpeed(150);
		WallFollowingLab.rightMotor.setSpeed(150
				);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	public void backward() {
		WallFollowingLab.leftMotor.setSpeed(motorLow);
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.backward();
		WallFollowingLab.rightMotor.backward();
	}
	
	public void turnRight() {
		// to close to the wall
		WallFollowingLab.leftMotor.setSpeed(500); //change direction to aviod cllision
		WallFollowingLab.rightMotor.setSpeed(motorLow);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	public void turnLeft() {
		// too far to the wall
		WallFollowingLab.leftMotor.setSpeed(200);
		WallFollowingLab.rightMotor.setSpeed(500);	
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
