package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * This class contains the zipline traversal and dismount logic.
 * @author Younes Boubekeur
 *
 */
public class TraverseZipline extends Thread{
	

	private int xc;
	private int yc;

	private static final int FORWARD_SPEED = Main.FWD_SPEED;
	private static final int TRAVERSE_SPEED = Main.TRAVERSE_SPEED;
	private static final int SLOWDOWN_SPEED = 50;
	public static final double WHEEL_RADIUS = Main.WHEEL_RADIUS;
	public static final double TRACK = Main.TRACK;
	
	private EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
	private EV3LargeRegulatedMotor rightMotor = Main.rightMotor;	
	private EV3LargeRegulatedMotor traverseMotor=Main.traverseMotor;
	private EV3ColorSensor colorSensor = Main.cSensor;

	private Odometer odometer = Main.odometer;
	private Navigation navigation = Main.navigation;

	public TraverseZipline(int xc, int yc) {
		this.xc=xc;
		this.yc=yc;

	}

	public void run() {

		//navigation.turnTo(90);
		//navigation.forward(2);
		
		// left and right motor still need to run
		setSpeed(FORWARD_SPEED);
		forward();
		
		// traverse motor starts to rotate
		traverseMotor.setSpeed(TRAVERSE_SPEED);
		traverseMotor.forward();
		
		
		navigation.travelTo(xc,yc);
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
	


	public void setSpeed(float speed) {
		setLeftSpeed(speed);
		setRightSpeed(speed);
	}

	public void setLeftSpeed(float speed) {
		leftMotor.setSpeed(speed);
	}

	public void setRightSpeed(float speed) {
		rightMotor.setSpeed(speed);
	}

	public void forwardLeft() {
		leftMotor.forward();
	}

	public void forwardRight() {
		rightMotor.forward();
	}

	public void forward() {
		forwardLeft();
		forwardRight();
	}
}