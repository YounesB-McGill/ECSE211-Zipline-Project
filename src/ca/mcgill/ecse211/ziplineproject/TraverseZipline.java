package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class contains the zipline traversal and dismount logic.
 * @author Younes Boubekeur
 *
 */
public class TraverseZipline extends Thread{
	private Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static EV3LargeRegulatedMotor traverseMotor;
	private Navigation navigation;
	private int xc;
	private int yc;

	private static final int FORWARD_SPEED = 100;
	private static final int TRAVERSE_SPEED = Main.TRAVERSE_SPEED;
	private static final int SLOWDOWN_SPEED = 50;

	public TraverseZipline(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor traverseMotor, Navigation navi, int xc, int yc) {
		this.odometer = odometer;
		TraverseZipline.leftMotor = leftMotor;
		TraverseZipline.rightMotor = rightMotor;
		TraverseZipline.traverseMotor = traverseMotor;
		navigation = navi;
		this.xc=xc;
		this.yc=yc;

	}

	public void run() {

		// left and right motor still need to run
		setSpeed(FORWARD_SPEED);
		forward();
		
		// traverse motor starts to rotate
		traverseMotor.setSpeed(TRAVERSE_SPEED);
		traverseMotor.forward();
		
		//navigation.moveTo(xc,yc); 

		// because motors runs at the same speed, I wonder whether we can get distances traveled by odometer.
		// If not, we have to change a lot in the odometer class
		/*while (odometer.getX() < 5) {
			;// do nothing
		}

		// slow down
		setSpeed(SLOWDOWN_SPEED);
		forward();
		traverseMotor.setSpeed(SLOWDOWN_SPEED);
		traverseMotor.forward();
		
		while (odometer.getX() < 7) {
			;// do nothing
		}
		
		//stop
		leftMotor.stop(true);
		rightMotor.stop(true);
		traverseMotor.stop(true);*/
		
		

	}

	public static void setSpeed(float speed) {
		setLeftSpeed(speed);
		setRightSpeed(speed);
	}

	public static void setLeftSpeed(float speed) {
		leftMotor.setSpeed(speed);
	}

	public static void setRightSpeed(float speed) {
		rightMotor.setSpeed(speed);
	}

	public static void forwardLeft() {
		leftMotor.forward();
	}

	public static void forwardRight() {
		rightMotor.forward();
	}

	public static void forward() {
		forwardLeft();
		forwardRight();
	}
}