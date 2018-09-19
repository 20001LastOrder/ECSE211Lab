package ca.mcgill.ecse211.wallfollowing;

public class PController implements UltrasonicController {

	/* Static Constants */
	private static final int MOTOR_SPEED = 200;             //speed for normal operation
	
	private static final int LEFT_MAX = 350;                //max speed for left wheel
	private static final int LEFT_MIN = 100;                //min speed for left wheel
	
	private static final int RIGHT_MAX = 500;               //max speed for right wheel
	private static final int RIGHT_MIN = 100;               //min speed for right wheel
	
	private static final int BACKWARD_REDUCED_SPEED = 100;  //speed for the right wheel when go back
	
	private static final int RIGHT_GAIN = 25;               //gain for correction on the right turn
	private static final int LEFT_GAIN = 12;                //gain for correction on the left turn
	private static final int MAX_ALLOW_ERR = 50;
	
	private static final int FILTER_OUT = 40;               //filter out amount of distance that to far

	private static final int UNUSUAL_DISTANCE = 255;        //distance that unusual far away     
	/* Instance Constants */
	private final int bandCenter;
	private final int bandWidth;
	
	/* Instance Variables */
	private int distance;
	private int filterControl;                             //control variable for recording how long the 
                                                           //distance that too far has been detected
	
	/**
	 * Constructor
	 * @param bandCenter: desired distance
	 * @param bandWidth: acceptable error
	 */
	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	/**
	 * process distance data from the sensor
	 * @param distance: distance detected
	 */
	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
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
		} else if(distance >= bandCenter+bandWidth){
			this.distance = distance;
		}else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// TODO: process a movement based on the us distance passed in (P style)
		int diff = distance - bandCenter;
		if(diff > MAX_ALLOW_ERR) {
			diff = MAX_ALLOW_ERR;
		}
		// if the distance is invalid (too far) nor the diff is within
		// the bandwidth, keep the current speed
		if (Math.abs(diff) < this.bandWidth) {
			forward();
		} else if (diff > 0) {
			turnLeft(diff);
		} else if (diff < 0) {
			if(diff < -(bandCenter - WallFollowingLab.DANGER_DISTANCE)) {
				//if too close, go backwards
				backward();
			}else {
				turnRight(diff);
			}
		}
	}

	/**
	 * move the robot along the way
	 */
	public void forward() {
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	/**
	 * move the robot backward (adjust the robot to facing right at the same time)
	 */
	public void backward() {
		//go backward different speed to let backward to help turning as well
		WallFollowingLab.leftMotor.setSpeed(BACKWARD_REDUCED_SPEED);
		WallFollowingLab.rightMotor.setSpeed(350);
		WallFollowingLab.leftMotor.backward();
		WallFollowingLab.rightMotor.backward();
	}
	
	/**
	 * turn the robot to the right
	 * @param diff: diff between desired distance and actual distance, used to calculate the gain
	 */
	public void turnRight(int diff) {
		// to close to the wall
		int gain = getGain(diff,RIGHT_GAIN);
		int leftSpeed = ((MOTOR_SPEED - gain) > LEFT_MAX)? LEFT_MAX : MOTOR_SPEED - gain;
		int rightSpeed = ((MOTOR_SPEED + gain) < RIGHT_MIN)? RIGHT_MIN : MOTOR_SPEED + gain;
		WallFollowingLab.leftMotor.setSpeed(leftSpeed); //change direction to aviod cllision
		WallFollowingLab.rightMotor.setSpeed(rightSpeed);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	/**
	 * turn the robot to the left
	 * @param diff: diff between desired distance and actual distance, used to calculate the gain
	 */
	public void turnLeft(int diff) {
		// too far to the wall
		int gain = getGain(diff,LEFT_GAIN);
		int leftSpeed = ((MOTOR_SPEED - gain) < LEFT_MIN )? LEFT_MIN:MOTOR_SPEED - gain;
		int rightSpeed = ((MOTOR_SPEED + gain) > RIGHT_MAX || (MOTOR_SPEED + gain) < 0)? RIGHT_MAX:MOTOR_SPEED + gain;
		WallFollowingLab.leftMotor.setSpeed(leftSpeed);
		WallFollowingLab.rightMotor.setSpeed(rightSpeed);	
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	/**
	 * get the current distance received from censor
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}

	/**
	 * calculate the gain
	 * @param diff: diff between desired distance and actual distance
	 * @param gain: the gain constant used to calculate the actual gain
	 * @return int: the calculated gain of this different at the provided constant
	 */
	public int getGain(int diff, int gain) {
		// consider different formulas
		int thisGain = gain*diff;
		return thisGain;
	}
}
