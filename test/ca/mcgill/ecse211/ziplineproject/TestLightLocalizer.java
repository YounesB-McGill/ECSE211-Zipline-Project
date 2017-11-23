package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Test the LightLocalization class 
 * 
 * The robot is placed at (1,1,0<sup>o</sup>), and is instructed to go the the points specified by the 
 * Wi-Fi parameters.
 * From the last point, it does light localization to orient itself to 180<sup>o</sup>.
 */
public class TestLightLocalizer {
    private static final double TILE = Main.TILE;
    
    private static Odometer odometer = Main.odometer;
    private static Navigation navigation = Main.navigation;
    private static Display display = Main.display;

    private static TextLCD textLCD = Main.textLCD;
    
    private static LightLocalizer lightLocalizer = new LightLocalizer();

    public static void testLightLocalizer() {
        
        // Uncomment if not using Wi-Fi
        
        // Wait for button press
        /*int button = 0;
        textLCD.drawString("Press a button ", 0, 0);
        textLCD.drawString("to start the ", 0, 1);
        textLCD.drawString("light loc. test", 0, 2);
        
        while (button == 0)
            button = Button.waitForAnyPress();
        */
        // Clear display
        textLCD.clear();
        
        // Remove this later
        odometer.start();
        display.start();
        
        // Set odometer to start at (1,1,0°)
        odometer.setX(1*TILE); odometer.setY(1*TILE); odometer.setTheta(0);
        
        /*navigation.travelTo(Main.red_ll_x, Main.red_ll_y); // replace with numbers if no Wi-Fi
        
        lightLocalizer.type = 1; // regular lightLocalization
        lightLocalizer.doLightLocalization(Main.red_ll_x, Main.red_ll_y);
        
        navigation.travelTo(Main.red_ur_x, Main.red_ur_y);
        
        lightLocalizer.doLightLocalization(Main.red_ur_x, Main.red_ur_y);*/
        
        // Temp hardcoded values:
        navigation.travelTo(1, 6); // replace with numbers if no Wi-Fi
        
        lightLocalizer.type = 1; // regular lightLocalization
        lightLocalizer.doLightLocalization(1, 6);
        
        navigation.travelTo(3, 6);
        
        lightLocalizer.doLightLocalization(3, 6);
        
        navigation.turnTo(180);
        

    }
    
}
