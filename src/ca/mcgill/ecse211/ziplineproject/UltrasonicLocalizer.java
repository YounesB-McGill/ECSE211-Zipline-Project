package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.Sound;

/**
 * This class contains the ultrasonic localization logic.
 * 
 * @author Mahad Khan
 *
 */
public class UltrasonicLocalizer {
    /**The ultrasonic sensor*/private static EV3UltrasonicSensor usSensor = Main.usSensor;
    /**The left motor*/private static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
    /**The right motor*/private static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
    /**The speed at which the motors rotate while turning in-place, in degrees/second*/
    private static final int ROTATION_SPEED = Main.ROTATE_SPEED;
    /**Threshold for distance between the US sensor and detected wall*/
    private static final int DISTANCE_THRESHOLD = 60;
    /**Constant used for noise filtering*/private static final int NOISE_FILTER = 20;
    /**Local version of Odometer object*/private static Odometer odometer = Main.odometer;
    /**The type of US localization used in this instance*/private static LocalizationType localizationType;
    /**The distance between the US sensor and an obstacle, in cm*/private static int distance;
    /**Number of times a bad US reading occured, used for filtering US data*/private static int count;
    /**Array containing US data readings*/private static float[] usData = new float[] {0};
    
    /**When <b><code>true</code></b>, print debugging information to the console*/
    private static boolean printToConsole = Main.printToConsole;
    
    /**The type of US localization to use, either rising edge or falling edge*/
    public static enum LocalizationType { RISING_EDGE, FALLING_EDGE };

    public UltrasonicLocalizer(LocalizationType type){
        localizationType = type;
    }
    
    /**
     * Do the ultrasonic localization
     */
    public static void doLocalization(){
        if(printToConsole) System.out.println("Entered doLocalization()");
        if(localizationType.equals(LocalizationType.RISING_EDGE)) risingEdge();
        else fallingEdge();
    }

    /**
     * Perform falling edge ultrasonic localization, which works by rotating in-place until a wall is seen
     */
    public static void fallingEdge() {
        Navigation.setAcceleration(125);
        double alpha = 0;
        double beta = 0;
        double deltaTheta = 0;

        
        while (getData() < DISTANCE_THRESHOLD) {
            rotateClockwise();
        }

        while (getData() > DISTANCE_THRESHOLD - NOISE_FILTER) {
            rotateClockwise();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);

        alpha = odometer.getThetaInDegrees();
        Sound.beep();

        while (getData() < DISTANCE_THRESHOLD) {
            rotateCounterClockwise();
        }

        while (getData() > DISTANCE_THRESHOLD - NOISE_FILTER) {
            rotateCounterClockwise();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        beta = odometer.getThetaInDegrees();
        Sound.beep();
//      leftMotor.stop(true); //TODO
//      rightMotor.stop(true);

        deltaTheta = headingToBe(alpha, beta);
        Sound.beepSequence();

        odometer.setTheta(odometer.getTheta() + Math.toRadians(deltaTheta));
        
        if(printToConsole) System.out.println("alpha: "+alpha+", beta: "+beta+", delta: "+deltaTheta); 

    }

    /**
     * Perform rising edge ultrasonic localization, which works by rotating in-place until no wall is seen
     */
    public static void risingEdge() {
        double alpha = 0;
        double beta = 0;
        double deltaTheta = 0;
        while (getData() > NOISE_FILTER) {
            rotateCounterClockwise();
        }
        Sound.buzz();

        while (getData() < DISTANCE_THRESHOLD) {
            rotateCounterClockwise();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        alpha = odometer.getThetaInDegrees();
        Sound.beep();

        while (getData() > NOISE_FILTER) {
            rotateClockwise();
        }
        Sound.buzz();

        while (getData() < DISTANCE_THRESHOLD) {
            rotateClockwise();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        beta = odometer.getThetaInDegrees();
        Sound.beep();

        deltaTheta = headingToBe(alpha, beta);
        Sound.beepSequence();
        leftMotor.stop(true);
        rightMotor.stop(true);
        odometer.setTheta(odometer.getTheta() + Math.toRadians(deltaTheta));
        
        if(printToConsole) System.out.println("alpha: "+alpha+", beta: "+beta+", delta: "+deltaTheta); 

    }

    /**
     * 
     * @return Distance from the US sensor in cm
     */
    public static int getData(){
        int dist;
        try { Thread.sleep(50); } catch (InterruptedException e) {}
        
        // there will be a delay here
        usSensor.getDistanceMode().fetchSample(usData, 0);
        dist = (int) (usData[0]*100);
                                                                            
        if(dist>DISTANCE_THRESHOLD && count<=3){ //filter for false positives or negatives
            count++; 
            return distance;
        }
        else if(dist>DISTANCE_THRESHOLD && count>3){
            return DISTANCE_THRESHOLD;
        }
        else{
            count=0;
            distance=dist;
            return dist;
        }
    }
    
    /**
     * Make robot rotate clockwise
     */
    public static void rotateClockwise() {
        leftMotor.setAcceleration(Main.FWD_ACC);
        rightMotor.setAcceleration(Main.FWD_ACC);
        leftMotor.setSpeed(ROTATION_SPEED);
        rightMotor.setSpeed(ROTATION_SPEED);
        leftMotor.backward();
        rightMotor.forward();
    }

    /**
     * Make robot rotate counterclockwise
     */
    public static void rotateCounterClockwise() {
        leftMotor.setAcceleration(Main.FWD_ACC);
        rightMotor.setAcceleration(Main.FWD_ACC);
        leftMotor.setSpeed(ROTATION_SPEED);
        rightMotor.setSpeed(ROTATION_SPEED);
        leftMotor.forward();
        rightMotor.backward();
    }

    /**
     * Calculate the difference in headings in order to determine the robot orientation
     * 
     * @param alpha The first angle that was detected, in degrees
     * @param beta The second detected angle, on the other wall, in degrees 
     * @return The actual difference between the two angles, in degrees
     */
    public static double headingToBe(double alpha, double beta) {
        double deltaTheta = 0;
        if (alpha < beta) {
            deltaTheta = 45 - ((alpha + beta) / 2);
        } else {
            deltaTheta = 225 - ((alpha + beta) / 2);
        }
        return deltaTheta;
    }

}
