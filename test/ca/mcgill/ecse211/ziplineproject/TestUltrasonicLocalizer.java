package ca.mcgill.ecse211.ziplineproject;

import ca.mcgill.ecse211.ziplineproject.UltrasonicLocalizer.LocalizationType;
import lejos.hardware.Button;

/**
 * Test the UltrasonicLocalization class 
 * Includes test for usData filtering
 */
public class TestUltrasonicLocalizer {
    
    /**
     * Test the UltrasonicLocalization class 
     * Includes test for usData filtering
     */
    public static void testUltrasonicLocalizer() {
        // Wait for button press
        int button = 0;
        Main.textLCD.drawString("Press a button ", 0, 0);
        Main.textLCD.drawString("to start the ", 0, 1);
        Main.textLCD.drawString("US Loc test", 0, 2);
        
        while (button == 0) {
            button = Button.waitForAnyPress();
        }
        
        // Clear display
        Main.textLCD.clear();
        
        testUltrasonicLocalizer(false); // Test everything by default
    }
    
    /**
     * Helper method to test the UltrasonicLocalization class 
     * Allows testing the filtering separately
     */
    @SuppressWarnings("static-access") // To use USLocalizer LocType
    public static void testUltrasonicLocalizer(boolean onlyTestingFiltering) {
        /* TODO Write usData filtering test here. Include a way to save the data,
         * by using a file or printing to Console using out.println() */
        
        
        if(onlyTestingFiltering) {
            return;
        }
        
        // TODO Write the rest of the test here:
        new UltrasonicLocalizer(LocalizationType.FALLING_EDGE).doLocalization();
        new Navigation().turnTo(0);
        
        /*while(true)
            System.out.println(new UltrasonicLocalizer(LocalizationType.FALLING_EDGE).getData());*/
        
    }

}
