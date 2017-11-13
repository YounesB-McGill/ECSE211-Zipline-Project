package ca.mcgill.ecse211.ziplineproject;

import java.util.Timer;
import java.util.TimerTask;

import ca.mcgill.ecse211.ziplineproject.UltrasonicLocalizer.LocalizationType;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * This class contains the zipline traversal and dismount logic.
 * @author Younes Boubekeur
 *
 */
public class TraverseZipline extends Thread  {
	

	private int xc;
	private int yc;

	private static final int FORWARD_SPEED = 50;
	private static final int TRAVERSE_SPEED = Main.TRAVERSE_SPEED;
	private static final int ROTATE_SPEED = Main.ROTATE_SPEED;
	private static final int SLOWDOWN_SPEED = 30;
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
		leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] { rightMotor });

	}

	public void run() {

		//navigation.turnTo(90);
		//navigation.forward(2);
		
		stopMotor();		
		
	    navigation.turnTo(0);
	    driveBackABit();
	    // traverse motor starts to rotate
		traverseMotor.setSpeed(TRAVERSE_SPEED);
		traverseMotor.forward();
		
		navigation.travelTo(xc,yc);
		clockwise(3,false);
					
		setSpeed(FORWARD_SPEED);
		forward();
		
		
		Timer timer = new Timer(true);  
		long delay=35*1000; //35 sec
		TimerTask task=new TimerTask(){
		        public void run() {
		        	/*UltrasonicLocalizer ul= new UltrasonicLocalizer(1);
		    		ul.run();*/
		        	/*setSpeed(SLOWDOWN_SPEED);
		    		forward();
		    		LightLocalizer ll=new LightLocalizer(2);
		    		int i=0;
		    		 while(true){
		    			 if (ll.hitGridLine()) {			
		    					Sound.beep();
		    					i++;
		    				}
		    			 if(i>2){
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
		    	         ll.run();*/
		        	 leftMotor.stop();
	    	         rightMotor.stop();
		        	
		        }
		    };
		
	 	timer.schedule(task,delay);
	 
		
		/*
		UltrasonicLocalizer ul= new UltrasonicLocalizer(1);
		ul.run();*/
		
		

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
	
	/**
	 * Set forward speed of the motors
	 */
	private void setForwardSpeed() {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	}

	private void driveBackABit() {
		stopMotor();
		setForwardSpeed();
		leftMotor.startSynchronization();
		leftMotor.rotate(-convertDistance(WHEEL_RADIUS, 1.5), true);
		rightMotor.rotate(-convertDistance(WHEEL_RADIUS, 1.5), true);
		leftMotor.endSynchronization();
		leftMotor.waitComplete();
		rightMotor.waitComplete();
		stopMotor();
	}
	
	/**
	 * Set rotation speed of the motors
	 */
	private void setRotateSpeed() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
	}

	
	/**
	 * Make robot rotate clockwise for a certain angle
	 * The second argument controls whether the next instruction(s)
	 * should be executed when this rotation hasn't ended.
	 */
	private void clockwise(double theta, boolean con) {
		stopMotor();
		setRotateSpeed();
		leftMotor.startSynchronization();
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), true);
		leftMotor.endSynchronization();
		if (con == false) {
			leftMotor.waitComplete();
			rightMotor.waitComplete();
		}

	}	
	/**
	 * Calls convertDistance to get wheel rotations based on the wheel radius,
	 * width, and angle
	 * 
	 * @param radius
	 *            Wheel radius of the regular EV3 wheels
	 * @param width
	 *            Width of the robot (TRACK)
	 * @param angle
	 *            Angle that defines the distance
	 * @returns Number of degrees that motor must rotate to travel the required
	 *          distance
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
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