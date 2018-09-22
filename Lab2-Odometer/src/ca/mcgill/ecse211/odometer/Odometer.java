/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton
  private static double DEGREE_TO_RADIAN_FACTOR;
  
  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  private double[] position;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
    position = new double[3];
    DEGREE_TO_RADIAN_FACTOR = 180/Math.PI;
  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();
      leftMotor.resetTachoCount();
      rightMotor.resetTachoCount();
      
      // Calculate new robot position based on tachometer counts
      double lastPosition[] = odo.getXYT();
      
      //calculate displacement of two wheels;
      double leftDisp= rotationToDistance(leftMotorTachoCount);
	  double rightDisp = rotationToDistance(rightMotorTachoCount);
	  
	  //calculate delta rotation based on displacement of two wheels
      position[2] = robotRotation(leftDisp, rightDisp);
      double deltaDegree = position[2] * DEGREE_TO_RADIAN_FACTOR;
      position[2] = deltaDegree;
      
      //find new rotation
      double newRotation = lastPosition[2]+deltaDegree;
      double newDegree = newRotation / DEGREE_TO_RADIAN_FACTOR;
      //calculate overall displacement by the average of left and right
      double displacement = (leftDisp + rightDisp) / 2;
     
      //calculate x/y displacement
      position[0] = deltaX(displacement, newDegree);
      position[1] = deltaY(displacement, newDegree);
      
      // Update odometer values with new calculated values
      odo.update(position[0], position[1], position[2]);
      
      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }
  
  private double rotationToDistance(int degree) {
	  return (degree * WHEEL_RAD / DEGREE_TO_RADIAN_FACTOR);
  }

  private double robotRotation(double l, double r) {
	  return (l-r)/TRACK;
  }
  
  private double deltaX(double d, double rotation) {
	  return d*Math.sin(rotation);
  }
  
  private double deltaY(double d, double rotation) {
	  return d*Math.cos(rotation);
  }
  
}
