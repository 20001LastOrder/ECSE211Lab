package ca.mcgill.ecse211.wallfollowing;

/**
 * BangBangController
 * @author AnssamGhezala
 * @author PercyChen 
 * 
 */
public class BangBangController implements UltrasonicController {
	private static final int UNUSUAL_DISTANCE = 100;
	private static final int NOMAL_MOTOR_SPEED = 150;
	
	private static final int BACKWARD_NORMAL_SPEED = 350;
	private static final int RIGHT_TURN_HIGH = 500;
	private static final int LEFT_TURN_HIGH = 300;
	private static final int LEFT_TURN_LOW = 100;
	
	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh; //never use it because need different values for different turns (see constants above)
	private int distance;
	private int filterControl;
	static final int FILTER_OUT = 34;

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
	
	/**
	 * processing sample data from sensor
	 * @param distance : distance recieved from sensor
	 */
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
			// offtrack or corner turn left
			turnLeft();
		} else if (diff < 0) {
			if(diff < -(bandCenter - WallFollowingLab.DANGER_DISTANCE)) {
				//if too close, go backwards
				backward();
			}else {
				//off track or corner turn right
				turnRight();
			}
		}
	}

	/**
	 * 
	 * move forward
	 */
	public void forward() {
		WallFollowingLab.leftMotor.setSpeed(NOMAL_MOTOR_SPEED);
		WallFollowingLab.rightMotor.setSpeed(NOMAL_MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	/**
	 * 
	 * turn backward
	 */
	public void backward() {
		WallFollowingLab.leftMotor.setSpeed(motorLow);
		WallFollowingLab.rightMotor.setSpeed(BACKWARD_NORMAL_SPEED);
		WallFollowingLab.leftMotor.backward();
		WallFollowingLab.rightMotor.backward();
	}
	
	/**
	 * turn the robot to the right
	 *
	 */
	public void turnRight() {
		// to close to the wall
		WallFollowingLab.leftMotor.setSpeed(RIGHT_TURN_HIGH);  
		WallFollowingLab.rightMotor.setSpeed(motorLow);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	/**
	 * turn the robot to the left
	 * 
	 */
	public void turnLeft() {
		// too far to the wall
		WallFollowingLab.leftMotor.setSpeed(LEFT_TURN_LOW); 
		WallFollowingLab.rightMotor.setSpeed(LEFT_TURN_HIGH);	 
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	@Override
	/**
	 * read sampled distance from sensor
	 * 
	 */
	public int readUSDistance() {
		return this.distance;
	}
}
