package ca.mcgill.ecse211.wallfollowing;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
  
  static final int FILTER_OUT = 20;
}
