package ca.mcgill.ecse211.ziplineproject;


import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class UltrasonicLocalizer extends Thread implements UltrasonicController {

	// variables only set once
	
	private final int rotateSpeed = Main.ROTATE_SPEED;
	private static final int ACCE_SPEED = 100;
	public double fp;
	public static final double ROTATION_SPEED = Main.ROTATE_SPEED;
	private final int wallDistance = 25;
	private final int k = 3;
	private double wheelRadius=Main.WHEEL_RADIUS;
	private double width=Main.TRACK;
	

	private static final Odometer odometer = Main.odometer;
	private static final Navigation navigation = Main.navigation;

	public static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
	public static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
	private static EV3LargeRegulatedMotor traverseMotor=Main.traverseMotor;

	private static LocalizationType localType;
	public static enum LocalizationType { RISING_EDGE, FALLING_EDGE };

	private static int count;
	private static float[] usData = new float[] {0};
	

	private int i = 0;

	// changing variables
	int filterControl = 0;
	static int distance;
	int sum;;

	private static int type;

	public UltrasonicLocalizer(int type){
		this.type=type;
	}

	public void run(){
		if(type==0)
			fallingEdge();
		if(type==1)
			doLastLocalization();
	}
	public static void doLastLocalization(){
		int i=0;
		 while(distance>10 || i<10){
			if(distance<=10){
				i++;
			}
		 } 
		 LightLocalizer ll=new LightLocalizer(2);
		 while(true){
			 if (ll.hitGridLine()) {			
					Sound.beep();
					break;
				}
		 }
		 //stop the robot
	         leftMotor.stop();
	         rightMotor.stop();
	         traverseMotor.stop();
	         Sound.beep();
	         Sound.beep();
	         Sound.beep();
	         ll.run();
		 
	}
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
		stopMotor();

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
			turningAngle = 30- averageAngle + odometer.getTheta() * 180 / Math.PI;
		} else { // start not facing the wall
			averageAngle = (fallingPoint1 + fallingPoint2) / 2;
			turningAngle = 30 - averageAngle + odometer.getTheta() * 180 / Math.PI;
		}

		Sound.beep();
		// make turn clockwise

		clockwise(turningAngle,false);

		odometer.setTheta(0);

		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
		}

	}

	private void setAcce() {
		leftMotor.setAcceleration(ACCE_SPEED);
		rightMotor.setAcceleration(ACCE_SPEED);
	}

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
	
	/**
	 * Make robot rotate clockwise for a certain angle
	 * The second argument controls whether the next instruction(s)
	 * should be executed when this rotation hasn't ended.
	 */
	private void clockwise(double theta, boolean con) {
		setRotateSpeed();
		leftMotor.startSynchronization();
		leftMotor.rotate(convertAngle(wheelRadius, width, theta), true);
		rightMotor.rotate(-convertAngle(wheelRadius, width, theta), true);
		leftMotor.endSynchronization();
		if (con == false) {
			leftMotor.waitComplete();
			rightMotor.waitComplete();
		}

	}


	/**
	 * Make robot rotate clockwise
	 */
	private void clockwise() {
		setRotateSpeed();
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.backward();
		leftMotor.endSynchronization();
	}

	/**
	 * Make robot rotate counterclockwise
	 */
	private void counterclockwise() {
		setRotateSpeed();
		leftMotor.startSynchronization();
		leftMotor.backward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}

	/**
	 * Make robot stop
	 */
	private void stopMotor() {
		leftMotor.startSynchronization();
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.endSynchronization();
		leftMotor.waitComplete();
		rightMotor.waitComplete();
	}
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
