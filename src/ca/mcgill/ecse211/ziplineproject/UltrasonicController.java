package ca.mcgill.ecse211.ziplineproject;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
  
}
