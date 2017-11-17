/**
 * 
 */
package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Test the Navigation class 
 * 
 * The robot is instructed to perform the equivalent of the following pseudocode:
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
 * This is a picture of the robot's trajectory. The starting coordinate is (1,1), which is the red dot.
 * 
 * <pre><img src="{@docRoot}/doc-files/TestNavigation.png" /></pre>
 * 
 * @author Younes Boubekeur
 * 
 */
public class TestNavigation {
    
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
    private static EV3LargeRegulatedMotor traverseMotor = Main.traverseMotor;
    //private static EV3UltrasonicSensor usSensor = Main.usSensor;
    private static EV3ColorSensor cSensor = Main.cSensor;
    private static TextLCD textLCD = Main.textLCD;
    
    private static Odometer odometer = Main.odometer;
    private static Navigation navigation = Main.navigation;
    private static Display display = Main.display;
    
    public static void testNavigation() {
        // Wait for button press
        int button = 0;
        textLCD.drawString("Press a button ", 0, 0);
        textLCD.drawString("to start the ", 0, 1);
        textLCD.drawString("navigation test", 0, 2);
        
        while (button == 0)
            button = Button.waitForAnyPress();
        
        // Clear display
        textLCD.clear();
        
        // Start odometer and display
        odometer.start();
        display.start();
        
        // Set odometer to start at (1,1)
        odometer.setX(1*TILE); odometer.setY(1*TILE);
        
        double points[][] = new double[][]{{1,3},{3,3},{3,1},{1,1},{1,3},{2,1},{3,2}};

        for(int i = 0; i < points.length; i++) {
            navigation.travelTo(points[i][0], points[i][1]);
        }
        
        navigation.turnTo(0);

    }

}
