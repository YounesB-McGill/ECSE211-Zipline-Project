package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Test the LightLocalization class 
 * 
 * The robot is placed at (1,1), and is instructed to go the the points (2,3) then (3,2).
 * From that last point, it does light localization to orient itself to 0<sup>o</sup>.
 */
public class TestLightLocalizer {
    private static final double TILE = Main.TILE;
    
    private static Odometer odometer = Main.odometer;
    private static Navigation navigation = Main.navigation;
    private static Display display = Main.display;
    private static TextLCD textLCD = Main.textLCD;

    public static void testLightLocalizer() {
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
        
        //navigation.travelTo(2, 3);
        //navigation.travelTo(3, 2);
        
        LightLocalizer lo=new LightLocalizer();
        //lo.dotest();
        lo.doLightLocalizationNew(3, 2);
        
        // TODO Assert result
    }
    
}
