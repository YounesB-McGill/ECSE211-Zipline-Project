package ca.mcgill.ecse211.lab5;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.port.MotorPort;
import lejos.hardware.lcd.TextLCD;

public class Navigation extends Thread implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int MOTOR_SLOW = 120;

	// variables only set once
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double wheelRadius;
	private double width;

	// changing variables
	private boolean isNavigating;
	private boolean isTurning;
	double[] position = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta;
	private int distance;
	private int filterControl;
	private int sum;
	private int desx;
	private int desy;

	// static values
	private static final int FORWARD_SPEED = 300;
	private static final int ROTATE_SPEED = 200;
	private static final double TILE = 30.48;

	// constructor
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double wheelRadius, double width, int desx, int desy) {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.wheelRadius = wheelRadius;
		this.width = width;
		this.filterControl = 0;
		this.desx = desx;
		this.desy = desy;

	}

	/**
	* Run the Navigation thread
	*/
	public void run() {
		// wait 4 second
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}
		// first travel to (0,0) to see the error
		travelTo(0, 0);
		turnTo(transferAngle(0)); // we convert angle 0 to polar coordinate to
									// match turnTo calculation

		// wait 2 second
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}

		navigating();
	}

	private void navigating() {
		// travel in the path specified

		travelTo(desx, desy);

		// wait 1 second
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}

	}
	
	/**
	* convert to polar coordinate
	*/
	public double transferAngle(double angle) {
		
		angle = (360.0 - angle) + 90.0;
		return angle;

	}

	/**
	* Travel to specified point
	* @param {double} x - X coordinate based on gridlines
	* @param {double} y - Y coordinate based on gridlines
	*/
	public void travelTo(double x, double y) {
		x = x * TILE;
		y = y * TILE;
		getPosition(odometer);
		// calculate the angle we need to turn to
		double theta1 = Math.atan((y - nowY) / (x - nowX)) * 360.0 / (2 * Math.PI);
		if (x - nowX < 0) {
			theta1 = 180.0 + theta1;
		}
		// turn to the proper angle
		turnTo(theta1);
		// drive forward
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		forwards();
		// keep calling turnTo and checking the distance
		while (true) {
			getPosition(odometer);
			// if we've reached our destination, break the loop
			if (Math.abs(y - nowY) < 0.8 && Math.abs(x - nowX) < 0.8)
				break;
		}

		leftMotor.stop(true);
		rightMotor.stop(true);

	}

	/**
	* Turn to specified angle
	* @param {double} theta - Theta in degrees
	*/
	public void turnTo(double theta) {

		isTurning = true;
		// convert to polar coordinate
		nowTheta = (360.0 - nowTheta) + 90.0;
		if (nowTheta > 360.0)
			nowTheta = nowTheta - 360.0;
		// calculate the angle we need to turn
		double turningTheta = theta - nowTheta;
		// make sure it is the minimal angle
		if (turningTheta > 180)
			turningTheta = turningTheta - 360.0;
		else if (turningTheta < -180)
			turningTheta = turningTheta + 360.0;

		// make a turn
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(wheelRadius, width, turningTheta), true);
		rightMotor.rotate(convertAngle(wheelRadius, width, turningTheta), false);
		isTurning = false;
	}

	/**
	* @returns true if the robot is either Navigating or Turning
	*/
	boolean isNavigating() {
		return isNavigating || isTurning;
	}

	/**
	* Get position from odometer
	*/
	void getPosition(Odometer odometer) {
		odometer.getPosition(position, new boolean[] { true, true, true });
		nowX = position[0];
		nowY = position[1];
		nowTheta = position[2];
	}
	
	/**
	* Make motors go forward
	*/
	private void forwards(){
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.endSynchronization();
    	}

	/**
	* Convert to wheel rotations based on the wheel radius and travel distance
	* @param {double} radius - Wheel radius of the regular EV3 wheels
	* @param {double} distance - Desired distance to travel
	* @returns Number of degrees that motor must rotate to travel the required distance
	*/
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	* Calls convertDistance to get wheel rotations based on the wheel radius, width, and angle
	* @param {double} radius - Wheel radius of the regular EV3 wheels
	* @param {double} width - Width of the robot (TRACK)
	* @param {double} angle - Angle that defines the distance
	* @returns Number of degrees that motor must rotate to travel the required distance
	*/
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	* Process ultrasonic distance data and filter it
	*/
	@Override
	public void processUSData(int distance) {
		// Filter for the sensor
		if (distance >= 150)
			distance = 150; // caps off readings at 150
		// Sets the distance to the average of every 5 readings
		if (filterControl < 6) {
			filterControl++;
			sum = sum + distance;
		} else {
			filterControl = 0;
			distance = sum / 5;
			sum = 0;
		}

		this.distance = distance;

	}
	
	/**
	* @returns Distance read from the ultrasonic sensor
	*/
	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
