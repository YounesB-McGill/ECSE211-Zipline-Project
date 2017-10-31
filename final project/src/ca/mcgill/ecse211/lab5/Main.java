package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/** This is the main class that actually performs the requirements of lab 5 
- input is drawn to the brick so coordinates can be chosen
- after the starting corner is chosen, the robot localizes
- the robot then navigates to the coordinates of the starting end of the zipline and then mounts and dismounts the zipline 
	-while using odometery to keep track of its location

*/
public class Main {

	/**
	 * initialize all the constants in the lab
	 */
    public static final double WHEEL_RADIUS = 2.13;
    public static final double TRACK = 14.80; // wait to decide
    public static final int TRAVERSE_SPEED = 100;
    public static final double TILE = 30.48;	
    public static final int FWD_SPEED=250;
    public static final int ROTATE_SPEED=200;
    
    /**
     * initialze motors
     */
    public static final EV3LargeRegulatedMotor leftMotor = 
    		new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    public static final EV3LargeRegulatedMotor rightMotor = 
    		new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    public static final EV3LargeRegulatedMotor traverseMotor = 
    		new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    
    /**
     * initialize sensors
     */
    public static final Port usPort = LocalEV3.get().getPort("S1");
    public static final Port sensorPort = LocalEV3.get().getPort("S4");
    
    /**
     * declare class objects that might be used 
     */
    public static Odometer odometer;
    public static TextLCD textLCD;
    public static EV3UltrasonicSensor usSensor;
    public static EV3ColorSensor cSensor;
    public static Navigation navigation;
    public static OdometryDisplay odometryDisplay ;
    
    /**
     * declare variables that might be used
     */
    private static int buttonChoice;
    static int x = 0;
    static int y = 0;
    static int startCorner = 0;      
    static int xc = 0; 
    static int yc = 0;
    
    /**
     * start main method of the project
     * @param args
     */
    public static void main(String[] args) {

    	/**
    	 * initialize odometer and screen display
    	 */
        textLCD = LocalEV3.get().getTextLCD();
        odometer = new Odometer(leftMotor, rightMotor, WHEEL_RADIUS, TRACK);
        OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, textLCD);
        
        /**
         * initialize ultrasonic sensor and the object for reading ultrasonic sensor data
         */
        SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
        SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
        float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned
        UltrasonicPoller usPoller = null;
        
        /**
         * initialize color sensor
         */
        EV3ColorSensor cSensor = new EV3ColorSensor(sensorPort);
        
        /**
         * ask for user to input values if necessary
         */       
        //do display
        //settingDisplay();
        
        
        /**
         * start running robot
         */
        buttonChoice = Button.waitForAnyPress();
        if (buttonChoice == Button.ID_ENTER) {
        	        	
        	//start test
        	Test test=new Test();
        	test.testing();
        	
        	
            // TODO Make robot localize according to Lab 4
        
          /*  odometer.start(); // thread 1
            odometryDisplay.start(); // thread 2

           UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor,
                    WHEEL_RADIUS, TRACK, startCorner);
            usPoller = new UltrasonicPoller(usDistance, usData, ultrasonicLocalizer);
            usPoller.start(); // thread 3

            // run the falling edge ultrasonic localizer
            ultrasonicLocalizer.start(); // thread 4
            dispose(ultrasonicLocalizer); // wait until thread 4 dies
            
           // when this is finished, start light localizer method
			Navigation navigation = 
					new Navigation(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK, x, y);
			LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftMotor, rightMotor,
					WHEEL_RADIUS, TRACK, navigation, startCorner,cSensor);
            lightLocalizer.start(); // thread 4
            dispose(lightLocalizer);
                    
           //localization ends, navigation starts
            if(startCorner == 0 || startCorner == 3) { // SW, NW
                navigation.travelTo(x, y); // This will take us to (x0, y0)
            } else if(startCorner == 1) { // SE|
                navigation.travelTo(1, 1); //avoid entering zip line area
                navigation.travelTo(x, y); // This will take us to (x0, y0)
            } else { // NE
                navigation.travelTo(1, 7); //avoid entering zip line area
                navigation.travelTo(x, y); // This will take us to (x0, y0)
            }
            
            // TODO Localize and turn to 90 deg
            // Localize position using same localization to go to (x0, y0)
            (new LightLocalizer(odometer, leftMotor, rightMotor, 
                    WHEEL_RADIUS, TRACK,navigation,startCorner,cSensor)).doLightLocalizationAgain(x,y);           
            
            // Pause and wait for user input
            while (Button.waitForAnyPress() == Button.ID_ENTER) {
                ; // do nothing
            }
          
           // Navigation navigation = new Navigation(leftMotor, rightMotor, odometer);     
          // traver to xc,yc, mount and traverse the zip line
           TraverseZipLine traverse = new TraverseZipLine(odometer, leftMotor, rightMotor, traverseMotor,
                 navigation,xc,yc);
            traverse.run();
          */  
            

        }

        while (Button.waitForAnyPress() != Button.ID_ESCAPE)
            ;
        System.exit(0);
    }

    
    /**
     * display content on the screen to remind user what to input
     * change the value of variables according to user input
     */
    private static void settingDisplay() {
    	 // choose x,y
        do {
            // clear the display
            textLCD.clear();

            textLCD.drawString("   Choose x,y   ", 0, 0);
            textLCD.drawString("   coordinate:  ", 0, 1);
            textLCD.drawString(" x=" + x + "    y=" + y, 0, 2);
            textLCD.drawString("left/right for x ", 0, 3);
            textLCD.drawString("up/down for y   ", 0, 4);
            textLCD.drawString("press center    ", 0, 5);
            textLCD.drawString("button to confirm", 0, 6);

            buttonChoice = Button.waitForAnyPress();

            if ((buttonChoice == Button.ID_RIGHT) && x < 8) {
                x++;
            } else if ((buttonChoice == Button.ID_LEFT) && x > 0) {
                x--;
            } else if ((buttonChoice == Button.ID_UP) && y < 8) {
                y++;
            } else if ((buttonChoice == Button.ID_DOWN) && y > 0) {
                y--;
            }
        } while (buttonChoice != Button.ID_ENTER);

        
        // choose xc,yc
        do {
            // clear the display
            textLCD.clear();

            textLCD.drawString("   Choose x,y   ", 0, 0);
            textLCD.drawString("   coordinate:  ", 0, 1);
            textLCD.drawString(" xc=" + xc + " yc=" + yc, 0, 2);
            textLCD.drawString("left/right for x ", 0, 3);
            textLCD.drawString("up/down for y   ", 0, 4);
            textLCD.drawString("press center    ", 0, 5);
            textLCD.drawString("button to confirm", 0, 6);

            buttonChoice = Button.waitForAnyPress();

            if ((buttonChoice == Button.ID_RIGHT) && x < 8) {
                xc++;
            } else if ((buttonChoice == Button.ID_LEFT) && x > 0) {
                xc--;
            } else if ((buttonChoice == Button.ID_UP) && y < 8) {
                yc++;
            } else if ((buttonChoice == Button.ID_DOWN) && y > 0) {
                yc--;
            }
        } while (buttonChoice != Button.ID_ENTER);

        // choose starting corner
        do {
            // clear the display
            textLCD.clear();

            textLCD.drawString(" Choose starting", 0, 0);
            textLCD.drawString("     corner:    ", 0, 1);
            textLCD.drawString("start from " + startCorner, 0, 2);
            textLCD.drawString("Left/right to   ", 0, 3);
            textLCD.drawString("change.         ", 0, 4);
            textLCD.drawString("Press center    ", 0, 5);
            textLCD.drawString("button to confirm", 0, 6);

            buttonChoice = Button.waitForAnyPress();

            if ((buttonChoice == Button.ID_RIGHT) && startCorner < 3) {
                startCorner++;
            } else if ((buttonChoice == Button.ID_LEFT) && startCorner > 0) {
                startCorner--;
            }
        } while (buttonChoice != Button.ID_ENTER);
		
	}
    
    /**
     * let a thread begin after current thread dies
     * @param thread
     */

	public static void dispose(Thread thread) {
        try {
            thread.join(); // wait till thread dies
        } catch (InterruptedException e) {}
    }
    
    
}
