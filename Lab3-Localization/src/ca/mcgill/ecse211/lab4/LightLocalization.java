package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class LightLocalization {
	  private static final double CHANGE_IN_PERSENT = -35 ;
	  private static final long CORRECTION_PERIOD = 10;
	  private static final double SENSOR_DISTANCE = 14.5;
	  
	  private Odometer odo;
	  private Navigation nav;
	  private SampleProvider colorSample;
	  private double lastBrightnessLevel;
	  private double currBrightnessLevel;
	  private float[] RGBValues;
	  
	  public LightLocalization(SampleProvider provider, Navigation nav) throws OdometerExceptions {
		  this.colorSample = provider;
		  this.odo = Odometer.getOdometer();
		  RGBValues = new float[colorSample.sampleSize()];
		  this.nav = nav;
	  }
	  public void performLocalization() throws OdometerExceptions {
		  nav.turnTo(45, odo.getXYT()[2]);
		  while(!isBlackLineTriggered()) {
			  nav.forward();
			  try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		  }
		  nav.stop();
		  double[] position = odo.getXYT();
		  nav.travelTo(position[0]-0.7, position[1]-0.7);
		  nav.turnTo(0, odo.getXYT()[2]);
		  
		  double thetaX1, thetaX2, thetaY1, thetaY2;
		  nav.setSpeed(50);
		  thetaX1 = getRorationOnBlackLine();
		  thetaY1 = getRorationOnBlackLine();
		  thetaX2 = getRorationOnBlackLine();
		  thetaY2 = getRorationOnBlackLine();
		  nav.turnTo(thetaY2, odo.getXYT()[2]);

		  //start correction
		  double thetaY = (thetaY2-thetaY1)/2;
		  double thetaX = (thetaX2-thetaX1)/2;
		  double deltaX = -SENSOR_DISTANCE * Math.cos(Math.toRadians((thetaY)));
		  double deltaY = -SENSOR_DISTANCE * Math.cos(Math.toRadians((thetaX)));
		  double deltaTheta = 90 + deltaY - (thetaY2 - 180);
		  
		  //correct to 0 rotation 
		  nav.rotate(deltaTheta - 265);
		  odo.setTheta(0);

		  //go to oringin and turn to 0 rotation
		  odo.setXYT(deltaX / Lab4.TILE_SIZE, deltaY / Lab4.TILE_SIZE, 0);
		  nav.travelTo(0, 0);
		  nav.turnTo(0, odo.getXYT()[2]);
	  }
	  
	  public double getRorationOnBlackLine() {
		  while(!isBlackLineTriggered()) {
			  nav.setSpeed(100);
			  nav.rotate(false);
		  }
		  return odo.getXYT()[2];
	  }
	  
	  private boolean isBlackLineTriggered() {
		  // Trigger correction (When do I have information to correct?)
	      colorSample.fetchSample(RGBValues, 0);
	      currBrightnessLevel = (RGBValues[0] + RGBValues[1] + RGBValues[2])*100;
	      
	      if(lastBrightnessLevel == -1) {
	    	  lastBrightnessLevel = currBrightnessLevel;
	      }
	      
	      //two conditions for blackline: abs threhold or changes in percentage
	      double changesPercent = 100 * (currBrightnessLevel - lastBrightnessLevel) / lastBrightnessLevel;
	      LCD.drawString("Co: " +currBrightnessLevel,0, 4);
	      LCD.drawString("Co: " +changesPercent,0, 5);
	      if(changesPercent < CHANGE_IN_PERSENT || currBrightnessLevel < 30) {
	    	  Sound.beep();
	    	  return true;
	      }else {
	    	  return false;
	      }
	  }
}
