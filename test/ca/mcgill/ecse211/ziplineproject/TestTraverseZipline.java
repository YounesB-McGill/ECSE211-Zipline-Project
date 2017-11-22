package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;

/**
 * This class is used to test zipline traversal
 * @author Younes Boubekeur
 *
 */
public class TestTraverseZipline {

    /**
     * Test zipline traversal
     */
    public static void testTraverseZipline() {
        // Not a good way to write code!
        
        // Wait for button press
        int button = 0;
        Main.textLCD.drawString("Press a button ", 0, 0);
        Main.textLCD.drawString("to start the ", 0, 1);
        Main.textLCD.drawString("zipline test", 0, 2);
        
        while (button == 0) {
            button = Button.waitForAnyPress();
        }
        
        // Clear display
        Main.textLCD.clear();
        
        Main.leftMotor.setSpeed(Main.FWD_SPEED);
        Main.rightMotor.setSpeed(Main.FWD_SPEED);
        Main.traverseMotor.setSpeed(Main.FWD_SPEED);
        
        Main.leftMotor.forward();
        Main.rightMotor.forward();
        Main.traverseMotor.forward();
        
        
    }
}
