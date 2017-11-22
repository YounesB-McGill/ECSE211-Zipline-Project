package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class contains all the tests that we need to carry out,
 * including unit and integration tests.
 * 
 * @author Younes Boubekeur
 */
public class Test{
    // Constants:
    private static final double TILE = Main.TILE;
    private static final double WHEEL_RADIUS = Main.WHEEL_RADIUS;
    private static final double TRACK = Main.TRACK;
    
    public static final int FWD_SPEED = Main.FWD_SPEED;
    public static final int ROTATE_SPEED = Main.ROTATE_SPEED;
    public static final int TRAVERSE_SPEED = Main.TRAVERSE_SPEED;
    
    private static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
    private static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
    private static EV3LargeRegulatedMotor traverseMotor = Main.traverseMotor;
    private static EV3UltrasonicSensor usSensor = Main.usSensor;
    private static EV3ColorSensor cSensor = Main.cSensor;
    private static TextLCD textLCD = Main.textLCD;
    
    private static Odometer odometer = Main.odometer;
    private static Navigation navigation = Main.navigation;
    private static Display display = Main.display;

    
}
