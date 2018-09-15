package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int filterControl;

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
	  
	  if(distance >= 255 && filterControl < FILTER_OUT) {
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
    
	  // process a movement based on the us distance passed in (BANG-BANG style)
	  int diff = bandCenter - distance;
	  
	  //if the distance is invalid (too far) nor the diff is within
	  //the bandwidth, keep the current speed
	  if(this.distance >= 255 || Math.abs(diff) < this.bandwidth) {
		  WallFollowingLab.leftMotor.setSpeed(motorHigh);
		  WallFollowingLab.rightMotor.setSpeed(motorHigh);
	  }else if(diff > 0) {
		  // too far to the wall
		  WallFollowingLab.leftMotor.setSpeed(motorLow);
		  WallFollowingLab.rightMotor.setSpeed(motorHigh);
	  }else if(diff < 0) {
		  //to close to the wall
		  WallFollowingLab.leftMotor.setSpeed(motorHigh);
		  WallFollowingLab.rightMotor.setSpeed(motorLow);
	  }
	  WallFollowingLab.leftMotor.forward();
	  WallFollowingLab.rightMotor.forward();
  }

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
