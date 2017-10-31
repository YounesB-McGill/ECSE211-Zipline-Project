package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** This class implements untrasonic localization */
public class UltrasonicLocalizer extends Thread implements UltrasonicController {

	// variables only set once
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double wheelRadius;
	private double width;
	private final int wallDistance = 25;
	private final int k = 3;
	private final int rotateSpeed = 90;
	private static final int ACCE_SPEED = 100;
	public double fp;

	private int i = 0;

	// changing variables
	int filterControl = 0;
	int distance;
	int sum;;

	/** 
	  * Constructor
	  * @param odometer
	  * @param leftMotor
	  * @param RightMotor
	  * @param wheelRadius
	  * @param width 
	  * @param startpoint
	  */
	public UltrasonicLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double wheelRadius, double width, int startpoint) {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.wheelRadius = wheelRadius;
		this.width = width;
		this.fp = 0;
		leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] { rightMotor });
	}

	/** Method required for starting the thread */
	public void run() {
		// wait 2 second
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}

		// start falling edge localization
		fallingEdge();
		// test();
	}

	/** method that implements falling edge ultrasonic localization */
	private void fallingEdge() {
		// ADD METHOD FOR FALLING EDGE

		setRotateSpeed();
		// rotate until the robot doesn't see a wall
		while (distance < (wallDistance + k)) {
			clockwise();

		}

		Sound.beep();
		clockwise();

		// detecting first falling point
		double fallingPoint1;
		while (true) {
			if (distance < (wallDistance + k)) {
				// enter noise margin
				Sound.beep();
				double heading1 = odometer.getTheta();
				while (distance > (wallDistance - k)) {
					// continue rotating
					clockwise();

				}
				double heading2 = odometer.getTheta();
				fallingPoint1 = ((heading1 + heading2) / 2) * 180 / Math.PI;
				break;
			}
		}
		fp = fallingPoint1;

		Sound.beep();
		// switch direction, rotate until the robot doesn't see a wall
		while (distance < (wallDistance + k)) {
			counterclockwise();
		}
		Sound.beep();
		counterclockwise();	
		
		// detecting second falling point
		double fallingPoint2;
		while (true) {
			if (distance < (wallDistance + k)) {
				// enter noise margin
				Sound.beep();
				double heading3 = odometer.getTheta();
				while (distance > (wallDistance - k)) {
					// continue rotating
					counterclockwise();

				}
				double heading4 = odometer.getTheta();
				fallingPoint2 = ((heading3 + heading4) / 2) * 180 / Math.PI;
				break;
			}
		}
		fp = fallingPoint2;

		Sound.beep();

		// stop the motor to do calculation
		leftMotor.startSynchronization();
		leftMotor.stop(true);
		rightMotor.stop();
		leftMotor.endSynchronization();

		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
		}

		double averageAngle;
		double turningAngle;

		// check if falling point 1 pass 0 point
		if (fallingPoint1 > fallingPoint2) { // start from facing the wall
			fallingPoint1 = fallingPoint1 - 360;
			averageAngle = (fallingPoint1 + fallingPoint2) / 2;
			turningAngle = 23 - averageAngle + odometer.getTheta() * 180 / Math.PI;
		} else { // start not facing the wall
			averageAngle = (fallingPoint1 + fallingPoint2) / 2;
			turningAngle = 15 - averageAngle + odometer.getTheta() * 180 / Math.PI;
		}

		Sound.beep();
		// make turn clockwise

		leftMotor.rotate(convertAngle(wheelRadius, width, turningAngle), true);
		rightMotor.rotate(-convertAngle(wheelRadius, width, turningAngle), false);

		odometer.setTheta(0);

		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
		}

	}

	/** Method that sets the current acceleration to a certain value */
	private void setAcce() {
		leftMotor.setAcceleration(ACCE_SPEED);
		rightMotor.setAcceleration(ACCE_SPEED);
	}

	/** a test method created to help debugging */
	private void test() {

		// turn 360 degrees clockwise
		setRotateSpeed();
		leftMotor.rotate(convertAngle(wheelRadius, 14.78, 360.0), true);
		rightMotor.rotate(-convertAngle(wheelRadius, 14.78, 360.0), false);

		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
		}
		// turn 360 degrees clockwise
		setRotateSpeed();
		leftMotor.rotate(convertAngle(wheelRadius, 14.8, 360.0), true);
		rightMotor.rotate(-convertAngle(wheelRadius, 14.8, 360.0), false);

		// to test TRACK value
	}

	/* method to turn clockwise */
	private void clockwise() {
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.backward();
		leftMotor.endSynchronization();
	}

	/* method to turn counterclockwise */
	private void counterclockwise() {
		leftMotor.startSynchronization();
		leftMotor.backward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}

	/* method that sets the rotate speed to a certain value */
	private void setRotateSpeed() {
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

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

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	public double readfp() {
		return this.fp;
	}

}
