package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.Display;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab3 {
	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	int buttonChoice;

	public static final double TILE_SIZE = 30.48;
	public static final double WHEEL_RAD = 2.15;
	public static final double TRACK = 14.43;

	public static void main(String args[]) {
		// Odometer related objects
		try {
			int buttonChoice;
			Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			Display odometryDisplay = new Display(lcd); // No need to change

		    do {
		        // clear the display
		        lcd.clear();

		        // ask the user whether the motors should drive in a square or float
		        lcd.drawString("< Left | Right >", 0, 0);
		        lcd.drawString("       |        ", 0, 1);
		        lcd.drawString(" Demo1 | Demo2  ", 0, 2);
		        lcd.drawString("       |        ", 0, 3);
		        lcd.drawString("       |        ", 0, 4);

		        buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		      } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		    
		    if (buttonChoice == Button.ID_LEFT) {

		        // Display changes in position as wheels are (manually) moved
		        
		        Thread odoThread = new Thread(odometer);
		        odoThread.start();
		        Thread odoDisplayThread = new Thread(odometryDisplay);
		        odoDisplayThread.start();

		     // spawn a new Thread to avoid SquareDriver.drive() from blocking
		        (new Thread() {
		          public void run() {
		        	  try {
						Navigation.driveTo(leftMotor, rightMotor, 0, 2);
						Navigation.driveTo(leftMotor, rightMotor, 1, 1);
						Navigation.driveTo(leftMotor, rightMotor, 2, 2);
						Navigation.driveTo(leftMotor, rightMotor, 2, 1);
						Navigation.driveTo(leftMotor, rightMotor, 1, 0);

					} catch (OdometerExceptions e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
		          }
		        }).start();
		      } else {
		        // Start odometer and display threads
		        Thread odoThread = new Thread(odometer);
		        odoThread.start();
		        Thread odoDisplayThread = new Thread(odometryDisplay);
		        odoDisplayThread.start();

		        // spawn a new Thread to avoid SquareDriver.drive() from blocking
		        (new Thread() {
		          public void run() {
		          }
		        }).start();
		      }

		      while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		      System.exit(0);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
