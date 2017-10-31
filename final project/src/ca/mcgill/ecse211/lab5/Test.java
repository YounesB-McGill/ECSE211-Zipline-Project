package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class contains all the tests that we need to carry out,
 * including unit and integration tests.
 */
public class Test{
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
    private static EV3UltrasonicSensor usSensor = Main.usSensor;
    private static EV3ColorSensor cSensor = Main.cSensor;
    private static TextLCD textLCD = Main.textLCD;
    
    private static Odometer odometer = Main.odometer;
    private static Navigation navigation = Main.navigation;
    private static OdometryDisplay odometryDisplay = Main.odometryDisplay;
    
    // TODO Variables used for testing:
    /* ... */
    
    /**
     * Test the Odometer class 
     * This test does not depend on Navigation
     * @return 
     */
    
    public void testing(){
    	testOdometer();
    }
    
    
    private static void testOdometer() {
        // Wait for button press
        int button = 0;
        System.out.println("Press a button \n"
                         + "to start the \n"
                         + "odometer test");
        while (button == 0)
            button = Button.waitForAnyPress();
        
        // Clear display
        textLCD.clear();
        
        // Start odometer and display
        odometer.start();
        odometryDisplay.start();
        
        for(int i = 0; i < 4; i++) {
            // Go forward for 2 tiles
            int degrees = (int) (2 // Number of tiles
                                  *(360 // degrees/rotation
                                       *TILE/( // rotations needed to travel the length of one TILE  
                                              2*Math.PI*WHEEL_RADIUS))); // = TILE/Wheel circumference 
            
            setFwdSpeed(); // Can't use Navigation helper methods here
            leftMotor.rotate(degrees, true);
            leftMotor.rotate(degrees, false);
            
            // Turn right by 90锟�
            degrees = (int) ((TRACK * 90) / (2 * WHEEL_RADIUS)); // Code from Lab 3, mathematically simplified
            setRotSpeed(); 
            leftMotor.rotate(degrees, true);
            rightMotor.rotate(-degrees, false); // Note the minus sign!
        }
        
        // Go forward for 2 tiles
        int degrees = (int) (2*(360*TILE/(2*Math.PI*WHEEL_RADIUS)));
        setFwdSpeed(); 
        leftMotor.rotate(degrees, true);
        leftMotor.rotate(degrees, false);
        
        // Turn right by 153.435锟�
        degrees = (int) ((TRACK * 153.435) / (2 * WHEEL_RADIUS));
        setRotSpeed(); 
        leftMotor.rotate(degrees, true);
        rightMotor.rotate(-degrees, false);
        
        // Go forward for 2.236 tile
        degrees = (int) (2.236*(360*TILE/(2*Math.PI*WHEEL_RADIUS)));
        setFwdSpeed(); 
        leftMotor.rotate(degrees, true);
        leftMotor.rotate(degrees, false);
        
        // Turn left by 108.435锟�
        degrees = (int) ((TRACK * 108.435) / (2 * WHEEL_RADIUS));
        setRotSpeed(); 
        leftMotor.rotate(-degrees, true); // Minus here since we're turning left
        rightMotor.rotate(degrees, false);
        
        // Go forward for 1.414 tiles
        degrees = (int) (1.414*(360*TILE/(2*Math.PI*WHEEL_RADIUS)));
        setFwdSpeed(); 
        leftMotor.rotate(degrees, true);
        leftMotor.rotate(degrees, false);
        
        // Turn left by 45锟�
        degrees = (int) ((TRACK * 45) / (2 * WHEEL_RADIUS));
        setRotSpeed(); 
        leftMotor.rotate(-degrees, true); // Minus here since we're turning left
        rightMotor.rotate(degrees, false);
        
        // Stop
        stop();

    }
    
    /**
     * Test the Navigation class 
     */
    private static void testNavigation() {
    	// Wait for button press
        int button = 0;
        System.out.println("Press a button \n"
                         + "to start the \n"
                         + "navigation test");
        while (button == 0)
            button = Button.waitForAnyPress();
        
        // Clear display
        textLCD.clear();
        
        // Start odometer and display
        odometer.start();
        odometryDisplay.start();
        
        int[][] destination = new int[][]{
			 {0,2},
			 {2,2},
			 {2,0},
			 {0,0},
			 {0,2},
			 {1,0},
			 {2,1},
	  };		
        
        //travel in a square
	   for (int i = 0; i < 7; i++){
		  double x = destination[i][0];
		  double y = destination[i][1];
		  navigation.travelTo(x,y);
		  
		  //stop motors
		  stop();	  
		  //wait 1 second
		 waitAWhile(1);
	    }        
        stop();
           
           
        }
        
      
        

    
    

	/**
     * Test the UltrasonicLocalization class 
     * Includes test for usData filtering
     */
    private static void testUltrasonicLocalizer() {
            testUltrasonicLocalizer(false); // Test everything by default
    }
    
    /**
     * Helper method to test the UltrasonicLocalization class 
     * Allows testing the filtering separately
     */
    private static void testUltrasonicLocalizer(boolean onlyTestingFiltering) {
        /* TODO Write usData filtering test here. Include a way to save the data,
         * by using a file or printing to Console using out.println() */
        
        if(onlyTestingFiltering) {
            return;
        }
        
        // TODO Write the rest of the test here:
        
    }
    
    /**
     * Test the LightLocalization class 
     */
    private static void testLightLocalizer() {
        
    }
    
    /**
     * Test the TraverseZipline class 
     */
    private static void testTraverseZipline() {
        
    }
    
    /**
     * Test the Thread timing
     */
    private static void testThreads() {
        // TODO Week of 11/6
    }
    
    /**
     * Test the Wi-Fi class 
     */
    private static void testWiFi() {
        
    }
    
    /**
     * Integration test
     */
    private static void integrationTest() {
        
    }
    
    
    private static void waitAWhile(int i) {
		// TODO Auto-generated method stub
    	 try {
			  Thread.sleep(i*1000);
		  } 
		  catch (InterruptedException e) {
		  }
		
	}
    private static void setFwdSpeed(){
    leftMotor.setSpeed(FWD_SPEED); 
    rightMotor.setSpeed(FWD_SPEED);
    }
    
    private static void setRotSpeed(){
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    }
    
    private static void stop(){
    leftMotor.stop();
    rightMotor.stop();
    }
    
}
