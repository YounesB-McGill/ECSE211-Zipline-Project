package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class contains all the tests that we need to carry out, including unit
 * and integration tests.
 */
public class Test {
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

	public Test() {

	}

	/**
	 * Test the Odometer class This test does not depend on Navigation
	 * 
	 * @return
	 * @return
	 */
	public void testing() {
		//testOdometer();
		 testNavigation() ;
	}

	private static void testOdometer() {
		// Wait for button press
		int button = 0;
		System.out.println("Press a button \n" + "to start the \n" + "odometer test");
		while (button == 0)
			button = Button.waitForAnyPress();

		// Clear display
		textLCD.clear();

		// Start odometer and display
		odometer.start();
		odometryDisplay.start();

		for (int i = 0; i < 4; i++) {
			// Go forward for 2 tiles
			forward(TILE*2);

			// Turn right by 90 degree
			clockwise(90);
		}

		// Go forward for 2 tiles
		forward(TILE*2);

		// Turn right by 153.435
		clockwise(153.435);

		// Go forward for 2.236 tile
		forward(TILE*Math.sqrt(5));

		// Turn left by 108.435
		counterclockwise(108.435);

		// Go forward for 1.414 tiles
		forward(TILE*Math.sqrt(2));
		
		// Turn left by 45
		counterclockwise(45);
		
		// Stop
		stop();

	}

	/**
	 * Test the Navigation class
	 */
	private static void testNavigation() {
		// Wait for button press
		int button = 0;
		System.out.println("Press a button \n" + "to start the \n" + "navigation test");
		while (button == 0)
			button = Button.waitForAnyPress();

		// Clear display
		textLCD.clear();

		// Start odometer and display
		odometer.start();
		odometryDisplay.start();

		int[][] destination = new int[][] { { 0, 2 }, { 2, 2 }, { 2, 0 }, { 0, 0 }, { 0, 2 }, { 1, 0 }, { 2, 1 } };

		// travel
		for (int i = 0; i < 7; i++) {
			double x = destination[i][0];
			double y = destination[i][1];
			navigation.travelTo(x, y);

			// stop motors
			stop();
			// wait 1 second
			waitAWhile(1);
		}
		stop();

	}

	/**
	 * Test the UltrasonicLocalization class Includes test for usData filtering
	 */
	private static void testUltrasonicLocalizer() {
		testUltrasonicLocalizer(false); // Test everything by default
	}

	/**
	 * Helper method to test the UltrasonicLocalization class Allows testing the
	 * filtering separately
	 */
	private static void testUltrasonicLocalizer(boolean onlyTestingFiltering) {
		/*
		 * TODO Write usData filtering test here. Include a way to save the
		 * data, by using a file or printing to Console using out.println()
		 */

		if (onlyTestingFiltering) {
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
			Thread.sleep(i * 1000);
		} catch (InterruptedException e) {
		}

	}

	private static void setFwdSpeed() {
		leftMotor.setSpeed(FWD_SPEED);
		rightMotor.setSpeed(FWD_SPEED);
	}

	private static void setRotSpeed() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
	}

	private static void stop() {
		leftMotor.stop();
		rightMotor.stop();
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static void clockwise(double turningTheta) {
		setRotSpeed();
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, turningTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, turningTheta), false);
	}
	
	private static void counterclockwise(double turningTheta) {
		setRotSpeed();
		leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, turningTheta), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, turningTheta), false);
	}
	
	private static void forward(double distance) {
		setFwdSpeed();
		leftMotor.rotate(convertDistance(WHEEL_RADIUS,distance ), true);
		rightMotor.rotate(convertDistance(WHEEL_RADIUS,distance), false);
	}

}
