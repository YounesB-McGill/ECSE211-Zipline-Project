package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;

public class TestFlagCapture {

    public static final TextLCD textLCD = Main.textLCD;

    public static void testFlagCapture() {
        
     // Wait for button press
        int button = 0;
        textLCD.drawString("Press a button ", 0, 0);
        textLCD.drawString("to start flag ", 0, 1);
        textLCD.drawString("capture test", 0, 2);
        
        while (button == 0)
            button = Button.waitForAnyPress();
        
        // Clear display
        textLCD.clear();
        
        
        FlagCapture.doFlagCaptureTest();
        
    }

}
