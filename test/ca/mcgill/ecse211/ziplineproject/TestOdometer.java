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
public class TestOdometer{
    // Constants:
    // TODO Add any missing constants
    private static final double TILE = Main.TILE;
    private static final double WHEEL_RADIUS = Main.WHEEL_RADIUS;
    private static final double TRACK = Main.TRACK;
    
    public static final int FWD_SPEED = Main.FWD_SPEED;
    public static final int ROTATE_SPEED = Main.ROTATE_SPEED;
    public static final int TRAVERSE_SPEED = Main.TRAVERSE_SPEED;
    
    private static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
    private static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
    private static TextLCD textLCD = Main.textLCD;
    
    private static Odometer odometer = Main.odometer;
    private static Display display = Main.display;
    
    // TODO Variables used for testing:
    /* ... */
    
    /**
     * Test the Odometer class 
     * 
     * This test does not depend on Navigation. The robot is instructed to perform the equivalent of 
     * the following pseudocode:
     * <pre><code>

    Wait for button press
    for i in range (0,4):   
        Go forward for 2 tiles
        Turn right by 90<sup>o</sup>
    Go forward for 2 tiles
    Turn right by 153.435<sup>o</sup>
    Go forward for 2.236 tiles
    Turn left by 108.435<sup>o</sup>
    Go forward for 1.414 tiles
    Turn left by 45<sup>o</sup>
    Stop
    
     * </code></pre>
     * 
     * This is a picture of the robot's trajectory. The starting coordinate is (1,1).
     * 
     * <pre><img src="{@docRoot}/doc-files/TestNavigation.png" /></pre>
     * 
     */
    public static void testOdometer() {
        // Wait for button press
        int button = 0;
        textLCD.drawString("Press a button ", 0, 0);
        textLCD.drawString("to start the ", 0, 1);
        textLCD.drawString("odometer test", 0, 2);
        
        /*System.out.println("Press a button \n"
                         + "to start the \n"
                         + "odometer test");*/
        
        while (button == 0)
            button = Button.waitForAnyPress();
        
        // Clear display
        textLCD.clear();
        /*for(int i = 0; i < 32; i++) {
            System.out.println();      
        }*/
        
        // Start odometer and display
        odometer.start();
        display.start();
        
        // Set odometer to start at (1,1)
        odometer.setX(1*TILE); odometer.setY(1*TILE);
        
        for(int i = 0; i < 4; i++) {
            // Go forward for 2 tiles
            int degrees = (int) (2 // Number of tiles
                                  *(360 // degrees/rotation
                                       *TILE/( // rotations needed to travel the length of one TILE  
                                              2*Math.PI*WHEEL_RADIUS))); // = TILE/Wheel circumference 
            
            leftMotor.setSpeed(FWD_SPEED); // Can't use Navigation helper methods here
            rightMotor.setSpeed(FWD_SPEED);
            leftMotor.rotate(degrees, true);
            rightMotor.rotate(degrees, false);
            try{Thread.sleep(100);}catch(Exception e) {}
            
            // Turn right by 90°
            degrees = (int) ((TRACK * 90) / (2 * WHEEL_RADIUS)); // Code from Lab 3, mathematically simplified
            leftMotor.setSpeed(ROTATE_SPEED);
            rightMotor.setSpeed(ROTATE_SPEED);
            leftMotor.rotate(degrees, true);
            rightMotor.rotate(-degrees, false); // Note the minus sign!
            try{Thread.sleep(100);}catch(Exception e) {}
        }
        
        // Go forward for 2 tiles
        int degrees = (int) (2*(360*TILE/(2*Math.PI*WHEEL_RADIUS)));
        leftMotor.setSpeed(FWD_SPEED); 
        rightMotor.setSpeed(FWD_SPEED);
        leftMotor.rotate(degrees, true);
        rightMotor.rotate(degrees, false);
        try{Thread.sleep(100);}catch(Exception e) {}
        
        // Turn right by 153.435°
        degrees = (int) ((TRACK * 153.435) / (2 * WHEEL_RADIUS));
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        leftMotor.rotate(degrees, true);
        rightMotor.rotate(-degrees, false);
        try{Thread.sleep(100);}catch(Exception e) {}
        
        // Go forward for 2.236 tile
        degrees = (int) (2.236*(360*TILE/(2*Math.PI*WHEEL_RADIUS)));
        leftMotor.setSpeed(FWD_SPEED);
        rightMotor.setSpeed(FWD_SPEED);
        leftMotor.rotate(degrees, true);
        rightMotor.rotate(degrees, false);
        try{Thread.sleep(100);}catch(Exception e) {}
        
        // Turn left by 108.435°
        degrees = (int) ((TRACK * 108.435) / (2 * WHEEL_RADIUS));
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        leftMotor.rotate(-degrees, true); // Minus here since we're turning left
        rightMotor.rotate(degrees, false);
        try{Thread.sleep(100);}catch(Exception e) {}
        
        // Go forward for 1.414 tiles
        degrees = (int) (1.414*(360*TILE/(2*Math.PI*WHEEL_RADIUS)));
        leftMotor.setSpeed(FWD_SPEED);
        rightMotor.setSpeed(FWD_SPEED);
        leftMotor.rotate(degrees, true);
        rightMotor.rotate(degrees, false);
        try{Thread.sleep(100);}catch(Exception e) {}
        
        // Turn left by 45°
        degrees = (int) ((TRACK * 45) / (2 * WHEEL_RADIUS));
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        leftMotor.rotate(-degrees, true); // Minus here since we're turning left
        rightMotor.rotate(degrees, false);
        try{Thread.sleep(100);}catch(Exception e) {}
        
        // Stop
        leftMotor.stop();
        rightMotor.stop();

    }
    
}
