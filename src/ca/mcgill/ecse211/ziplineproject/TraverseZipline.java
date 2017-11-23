package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class contains the zipline traversal and dismount logic.
 * 
 * @author Younes Boubekeur
 */
public class TraverseZipline {

    private static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
    private static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
    private static EV3LargeRegulatedMotor traverseMotor = Main.traverseMotor;

    private Odometer odometer = Main.odometer;
    private Navigation navigation = Main.navigation;
    private int xc;
    private int yc;

    private static final int FORWARD_SPEED = 375;
    private static final int TRAVERSE_SPEED = Main.TRAVERSE_SPEED;
    private static final int SLOWDOWN_SPEED = 50;
    /**The length of the Zipline*/
    private static final double ZIPLINE_DISTANCE = 4.3*Main.TILE*1.9; // 4 Tiles and a bit more

    public static void traverseZipline() {
        // left and right motor still need to run
        setSpeed(FORWARD_SPEED);
        forward();

        // traverse motor starts to rotate
        traverseMotor.setSpeed(TRAVERSE_SPEED);
        traverseMotor.rotate(Navigation.convertDistance(Main.WHEEL_RADIUS, ZIPLINE_DISTANCE), false);
        
        stopMotors();
    }

    public void run() { // TODO Remove this

        // left and right motor still need to run
        setSpeed(FORWARD_SPEED);
        forward();

        // traverse motor starts to rotate
        traverseMotor.setSpeed(TRAVERSE_SPEED);
        traverseMotor.forward();

        // navigation.moveTo(xc,yc);

        // because motors runs at the same speed, I wonder whether we can get
        // distances traveled by odometer.
        // If not, we have to change a lot in the odometer class
        /*
         * while (odometer.getX() < 5) { ;// do nothing }
         * 
         * // slow down setSpeed(SLOWDOWN_SPEED); forward();
         * traverseMotor.setSpeed(SLOWDOWN_SPEED); traverseMotor.forward();
         * 
         * while (odometer.getX() < 7) { ;// do nothing }
         * 
         * //stop leftMotor.stop(true); rightMotor.stop(true);
         * traverseMotor.stop(true);
         */

    }

    public static void setSpeed(float speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    public static void setLeftSpeed(float speed) {
        leftMotor.setSpeed(speed);
    }

    public static void setRightSpeed(float speed) {
        rightMotor.setSpeed(speed);
    }

    public static void forwardLeft() {
        leftMotor.forward();
    }

    public static void forwardRight() {
        rightMotor.forward();
    }

    public static void forward() {
        forwardLeft();
        forwardRight();
    }
    
    /**
     * Stop the left and right motors
     */
    public static void stopMotors() {
        leftMotor.stop();
        rightMotor.stop();
    }

}