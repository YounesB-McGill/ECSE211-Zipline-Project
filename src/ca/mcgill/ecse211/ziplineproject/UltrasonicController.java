package ca.mcgill.ecse211.ziplineproject;

/**
 * Reads and acts upon data collected from the UltrasonicSensor,
 * mainly for localization and obstacle detection and avoidance.
 * 
 * @author DPM Instructors
 */
public interface UltrasonicController {
    
    /**
     * Process a movement based on the ultrasonic distance passed in
     * @param {int} distance - The distance detected by the UltrasonicSensor, in cm.
     */
    public void processUSData(int distance);

    /**
     * @return The distance between the UltrasonicSensor and an obstacle
     */
    public int readUSDistance();

}
