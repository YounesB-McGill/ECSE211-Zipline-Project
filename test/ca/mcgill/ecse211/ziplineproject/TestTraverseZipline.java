package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;

public class TestTraverseZipline {
	 private static final double TILE = Main.TILE;
	    
	    private static Odometer odometer = Main.odometer;
	    private static Navigation navigation = Main.navigation;
	    private static Display display = Main.display;
	    private static TextLCD textLCD = Main.textLCD;

	    public static void testTraverseZipline() {
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
	        
	        odometer.setX(2*TILE); odometer.setY(1*TILE);
	        
	      /*  TraverseZipline tz= new TraverseZipline(2,2);
	        tz.run();*/
	        
	    }
}
