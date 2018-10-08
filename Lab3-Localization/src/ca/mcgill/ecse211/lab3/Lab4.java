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
public class Lab4 {
	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	int buttonChoice;

	public static final int FORWARD_SPEED = 100;
	public static final double TILE_SIZE = 30.48;
	public static final double WHEEL_RAD = 2.15;
	public static final double TRACK = 13.67;

	public static void main(String args[]) {
		// Odometer related objects
		try {
			int buttonChoice;
			Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			@SuppressWarnings("resource") // Because we don't bother to close this resource
			SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
			SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
			Display odometryDisplay = new Display(lcd);        
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			do {
				// clear the display
				lcd.clear();

				// ask the user whether the motors should drive in a square or float
				lcd.drawString("< Left | Right >", 0, 0);
				lcd.drawString("       |        ", 0, 1);
				lcd.drawString(" Rising| Falling", 0, 2);
				lcd.drawString("     Edge       ", 0, 3);
				lcd.drawString("       |        ", 0, 4);

				buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

			if (buttonChoice == Button.ID_LEFT) {
				
				UltrasonicLocalization usLocal = new UltrasonicLocalization(
													UltrasonicLocalization.UltrasonicLocalizationMode.RISING_EDGE,
												  usDistance, leftMotor, rightMotor);
				Thread lcdThread = new Thread(odometryDisplay);
				lcdThread.start();
				usLocal.performLocalization();
			}else if(buttonChoice == Button.ID_RIGHT) {
				UltrasonicLocalization usLocal = new UltrasonicLocalization(
						UltrasonicLocalization.UltrasonicLocalizationMode.FALLING_EDGE,
					  usDistance, leftMotor, rightMotor);
				Thread lcdThread = new Thread(odometryDisplay);
				lcdThread.start();
				usLocal.performLocalization();
			}

			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * convert the angle and distance to the distance covered by motor
	 * @param radius: radius of the motor
	 * @param distance: distance of the motor to go
	 * @return: distance covered by the motor
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) (((360/2)* distance) / (Math.PI * radius));
	}

	/**
	 * convert the angle of the robot to turn to the distance covered by motor
	 * @param radius: radius of the motor
	 * @param width: width of the robot
	 * @param angle angle to turn by the robot
	 * @return distance covered by the motor
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360);
	}
}
