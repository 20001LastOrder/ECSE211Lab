package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;

import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Main class for the navigation and obstacle avoiding lab
 * @author Percy Chen 260727855
 * @author 
 *
 */
public class Lab3 {
	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final int[][] destinations = { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
	int buttonChoice;

	public static final double TILE_SIZE = 30.48;
	public static final double WHEEL_RAD = 2.15;
	public static final double TRACK = 13.67;

	public static void main(String args[]) {
		// Odometer related objects
		try {
			int buttonChoice;
			Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

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
				Display odometryDisplay = new Display(lcd); // No need to change

				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();

				// spawn a new Thread to avoid SquareDriver.drive() from blocking
				Navigation nav = new Navigation(leftMotor, rightMotor, null, destinations);
				Thread navThread = new Thread(nav);
				navThread.start();
			} else {
				@SuppressWarnings("resource")
				SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
				SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
																			// this instance
				float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
																		// returned
				// start poller thread
				UltrasonicPoller poller = new UltrasonicPoller(usDistance, usData);
				Display odometryDisplay = new Display(lcd, poller); // No need to change
				// Start odometer and display threads
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				Thread pollerThread = new Thread(poller);
				pollerThread.start();

				// spawn a new Thread to avoid SquareDriver.drive() from blocking
				Navigation nav = new Navigation(leftMotor, rightMotor, poller, destinations);
				Thread navThread = new Thread(nav);
				navThread.start();
			}

			while (Button.waitForAnyPress() != Button.ID_ESCAPE)
				;
			System.exit(0);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
