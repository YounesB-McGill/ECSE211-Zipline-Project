package ca.mcgill.ecse211.lab5;

/** Interface for Ultrasonic Controller */ 

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
  
}
