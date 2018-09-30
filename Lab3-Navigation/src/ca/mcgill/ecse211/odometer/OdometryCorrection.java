/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class correct the value for odometer
 * @author Percy Chen
 * @author Anssam Ghezala
 *
 */

public class OdometryCorrection implements Runnable {
  private static final double CHANGE_IN_PERSENT = -35 ;
  private static final long CORRECTION_PERIOD = 10;
  private static final double THETA_ERR = 13;
  private static final double SENSOR_OFFSET = 1.7;
  private static final double DIS_ERR = 12;
  private static final double TILE_SIZE = 30.48;
  
  //angel threhold for moving along y direction
  private static final double Y_POSITIVE_UPPER_ANGLE = 30;
  private static final double Y_POSITIVE_LOWER_ANGLE = 330;

//angel threhold for moving along x direction
  private static final double X_POSITIVE_UPPER_ANGLE = 60;
  private static final double X_POSITIVE_LOWER_ANGLE = 120;
  
  private Odometer odometer;
  
  //color sensor related
  private SampleProvider colorSample;
  private double lastBrightnessLevel;
  private double currBrightnessLevel;
  private float[] RGBValues;
  private boolean triggerBlackLine;
  
  private boolean isFirstXCorrection;
  private boolean isFirstYCorrection;
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(SampleProvider colorSample) throws OdometerExceptions {
	this.colorSample = colorSample;
    this.odometer = Odometer.getOdometer();
    RGBValues = new float[colorSample.sampleSize()];
    
    //set first correction to true
    isFirstXCorrection = true;
    isFirstYCorrection = true;
    
    lastBrightnessLevel = -1;
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // Trigger correction (When do I have information to correct?)
      colorSample.fetchSample(RGBValues, 0);
      currBrightnessLevel = RGBValues[0] + RGBValues[1] + RGBValues[2];
      
      if(lastBrightnessLevel == -1) {
    	  lastBrightnessLevel = currBrightnessLevel;
      }
      
      //two conditions for blackline: abs threhold or changes in percentage
      double changesPercent = 100 * (currBrightnessLevel - lastBrightnessLevel) / lastBrightnessLevel;
      
      if(changesPercent < CHANGE_IN_PERSENT) {
    	  triggerBlackLine = true;
      }else {
    	  triggerBlackLine = false;
      }
      
      // Calculate new (accurate) robot position
      if(triggerBlackLine) {
    	  Sound.beep();
    	  doCorrection();
      }
      
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
  
  /**
   * do correction of the robot
   */
  private void doCorrection() {
	  double[] position = odometer.getXYT();
	  double theta = position[2];
	  //get current x/y direction of the robot
	  char robotDir = getRobotDir(theta);
	  double correctedX = -1, correctedY = -1;
	  
	  //correct x/y value based on the direction
	  if(robotDir == 'x') {
		  correctedX = correctX(position[0], theta);
		  position[0] = correctedX < 0? position[0]:correctedX;
		  
	      // Update odometer with new calculated (and more accurate) vales
		  odometer.setX(position[0]);
	  }else if(robotDir == 'y') {
		  correctedY = correctY(position[1], theta);
		  position[1] = correctedY < 0? position[1]:correctedY;
		  
	      // Update odometer with new calculated (and more accurate) vales
		  odometer.setY(position[1]);
	  }
  }
  
  /**
   * get the current x/y direction of the robot
   * @param theta: current rotation in radian
   * @return char: 'x' / 'y'
   */
  private char getRobotDir(double theta){
	  int angle = 90;
	  
	  //try 0, 90, 180, 270 to find the nearest angle for theta
	  for(int i = 0; i < 4; i++) {
		  int thisAngle = i * angle;
		  double err = Math.abs(theta - thisAngle);
		  if(err > 350) {
			  err = err - 360;
		  }
		  
		  boolean within = err<THETA_ERR; 
		  if(i % 2==0 && within) {
			  return 'y';
		  }else if(within){
			  return 'x';
		  }
	  }
	  //no suitble dir, must have an err, return empty
	  return ' ';
  }
  
  /**
   * get the more accurate  x coordinate of the robot
   * @param curreX: current(potentially bad) value of x
   * @param theta: current rotation in radian
   * @return double: corrected x value
   */
  private double correctX(double currX, double theta) {
	  if(isFirstXCorrection) {
		  //first detect the line, change the coordinate from robot coordinate the real coordinate
		  isFirstXCorrection = false;
		  return 0;
	  }else {
		  for(int i = 0; i < 3; i++) {
			  if(Math.abs(currX - i * TILE_SIZE) < DIS_ERR) {
				  //try all potential coordinates of the line, and find the closest one
				  if(theta > X_POSITIVE_LOWER_ANGLE && theta < X_POSITIVE_UPPER_ANGLE) {
					  return TILE_SIZE*i - SENSOR_OFFSET;
				  }else {
					  return TILE_SIZE*i + SENSOR_OFFSET;
				  }
			  }
		  }
		  //no suitable value, not change
		  return -1;
	  }
  }
  
  /**
   * get the more accurate Y coordinate of the robot
   * @param curreY: current(potentially bad) value of x
   * @param theta: current rotation in radian
   * @return double: corrected Y value
   */
  private double correctY(double currY, double theta) {
	  if(isFirstYCorrection) {
		  //first detect the line, change the coordinate from robot coordinate the real coordinate
		  isFirstYCorrection = false;
		  return 0;
	  }else {
		  //try all potential coordinates of the line, and find the closest one
		  for(int i = 0; i < 3; i++) {
			  if(Math.abs(currY - i * TILE_SIZE) < DIS_ERR){
				  if(theta < Y_POSITIVE_UPPER_ANGLE || theta > Y_POSITIVE_LOWER_ANGLE) {
					  return TILE_SIZE*i - SENSOR_OFFSET; 
				  }else {
					  return TILE_SIZE*i + SENSOR_OFFSET; 
				  }
			  }
		  }
		  //no suitable value, not change
		  return -1;
  	}
  }
  
}
