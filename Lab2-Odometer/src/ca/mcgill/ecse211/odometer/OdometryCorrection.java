/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final double CHANGE_IN_PERSENT = -35 ;
  private static final long CORRECTION_PERIOD = 10;
  private static final double THETA_ERR = 13;
  private static final double SENSOR_OFFSET = 1.7;
  private static final double DIS_ERR = 12;
  private static final double TILE_SIZE = 30.48;
  
  //Test use, delelete
  public double[] correctPositions;
  public static OdometryCorrection instance;
  
  private Odometer odometer;
  
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
    
    // test deleteme
    instance = this;
    correctPositions = new double[3];
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
  
  private void doCorrection() {
	  double[] position = odometer.getXYT();
	  double theta = position[2];
	  char robotDir = getRobotDir(theta);
	  double correctedX = -1, correctedY = -1;
	  
	  if(robotDir == 'x') {
		  correctedX = correctX(position[0], theta);
		  position[0] = correctedX < 0? position[0]:correctedX;
		  correctPositions = position;
		  odometer.setX(position[0]);
	  }else if(robotDir == 'y') {
		  correctedY = correctY(position[1], theta);
		  position[1] = correctedY < 0? position[1]:correctedY;
		  correctPositions = position;
		  odometer.setY(position[1]);
	  }
      // Update odometer with new calculated (and more accurate) vales
  }
  
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
  
  private double correctX(double currX, double theta) {
	  if(isFirstXCorrection) {
		  isFirstXCorrection = false;
		  return 0;
	  }else {
		  for(int i = 0; i < 3; i++) {
			  if(Math.abs(currX - i * TILE_SIZE) < DIS_ERR) {
				  if(theta > 60 && theta < 120) {
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
  
  private double correctY(double currY, double theta) {
	  if(isFirstYCorrection) {
		  isFirstYCorrection = false;
		  return 0;
	  }else {
		  for(int i = 0; i < 3; i++) {
			  if(Math.abs(currY - i * TILE_SIZE) < DIS_ERR){
				  if(theta < 30 || theta > 330) {
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
