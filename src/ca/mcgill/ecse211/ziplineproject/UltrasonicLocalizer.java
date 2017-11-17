
package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class UltrasonicLocalizer extends Thread implements UltrasonicController {

	// variables only set once
	private static final int FORWARD_SPEED = Main.FWD_SPEED;
	private final int rotateSpeed = Main.ROTATE_SPEED;
	private static final int ACCE_SPEED = 100;
	public double fp;
	public static final double ROTATION_SPEED = Main.ROTATE_SPEED;
	private final int wallDistance = 25;
	private final int k = 3;
	private double wheelRadius = Main.WHEEL_RADIUS;
	private double width = Main.TRACK;

	private static final Odometer odometer = Main.odometer;
	private static final Navigation navigation = Main.navigation;

	public static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
	public static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
	private static EV3LargeRegulatedMotor traverseMotor = Main.traverseMotor;

	private static LocalizationType localType;

	public static enum LocalizationType {
		RISING_EDGE, FALLING_EDGE
	};

	private static int count;
	private static float[] usData = new float[] { 0 };

	private int i = 0;

	// changing variables
	int filterControl = 0;
	static int distance;
	int sum;;

	private static int type;

	public UltrasonicLocalizer(int type) {
		this.type = type;
	}

	public void run() {
		//if (type == 0)
			fallingEdge();
		
	}

	/*public static void doLastLocalization() {
		driveForward();
		int i = 0;
		while (i < 10) {
			if (distance <= 30) {
				i++;
			}
		}

		// stop the robot
		leftMotor.stop();
		rightMotor.stop();
		traverseMotor.stop();
		LightLocalizer ll = new LightLocalizer(2);
		Sound.beep();
		Sound.beep();
		Sound.beep();
		ll.run();
		//navigation.travelTo(Main.xd, Main.yd);

	}*/

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


		double averageAngle;
		double turningAngle;

		// check if falling point 1 pass 0 point
		if (fallingPoint1 > fallingPoint2) { // start from facing the wall
			fallingPoint1 = fallingPoint1 - 360;
			averageAngle = (fallingPoint1 + fallingPoint2) / 2;
			turningAngle = 30 - averageAngle + odometer.getTheta() * 180 / Math.PI;
		} else { // start not facing the wall
			averageAngle = (fallingPoint1 + fallingPoint2) / 2;
			turningAngle = 30 - averageAngle + odometer.getTheta() * 180 / Math.PI;
		}

		Sound.beep();
		// make turn clockwise

		clockwise(turningAngle, false);

		odometer.setTheta(0);


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
	 * Make robot rotate clockwise for a certain angle The second argument
	 * controls whether the next instruction(s) should be executed when this
	 * rotation hasn't ended.
	 */
	private void clockwise(double theta, boolean con) {
		setRotateSpeed();
		//leftMotor.startSynchronization();
		leftMotor.rotate(convertAngle(wheelRadius, width, theta), true);
		rightMotor.rotate(-convertAngle(wheelRadius, width, theta), true);
		//leftMotor.endSynchronization();
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
		//leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.backward();
		//leftMotor.endSynchronization();
	}

	/**
	 * Make robot rotate counterclockwise
	 */
	private void counterclockwise() {
		setRotateSpeed();
		//leftMotor.startSynchronization();
		leftMotor.backward();
		rightMotor.forward();
		//leftMotor.endSynchronization();
	}

	/**
	 * Make robot stop
	 */
	private static void stopMotor() {
		//leftMotor.startSynchronization();
		leftMotor.stop();
		rightMotor.stop();
		//leftMotor.endSynchronization();
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

	/**
	 * Make both motors go forward
	 */
	private static void driveForward() {
		stopMotor();
		setForwardSpeed();
		//leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		//leftMotor.endSynchronization();
	}

	/**
	 * Set forward speed of the motors
	 */
	private static void setForwardSpeed() {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
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

  /*
package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
*/
/**
 * This class contains the ultrasonic localization logic.
 * 
 * @author Paarth Kalia
 *
 */
  /*
public class UltrasonicLocalizer {
    private static final int WALL_DIST = 45;
    private static final int DISTANCE_CAP = 60;
    private static final int LARGE_ANGLE = 225-180;
    private static final int SMALL_ANGLE = 45+180;
    private static final int ORIGIN = 80;
    public static final double ROTATION_SPEED = 50;
    
    private static final Odometer odometer = Main.odometer;
    private static final Navigation navigation = Main.navigation;
    
    public static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
    public static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
    
    private static EV3UltrasonicSensor usSensor = Main.usSensor;

    private static LocalizationType localType;
    public static enum LocalizationType { RISING_EDGE, FALLING_EDGE };
    
    private static int distance;
    private static int count;
    private static float[] usData = new float[] {0};
    
    public UltrasonicLocalizer(LocalizationType type){
        localType = type;
    }*/
    /**
     * Do the ultrasonic localization
     */
    @SuppressWarnings("static-access")
//    public static void doLocalization(){
//        double[] pos = new double[3];
//        double angle1, angle2, deltAngle;
//        //clear the screen (removed)
//        //textLCD.clear(); 
//        //draw usData
//        System.out.println(getData());
//        
//        if(localType==LocalizationType.FALLING_EDGE){ //this means we have to rotate until wall is seen
//            while(getData()<DISTANCE_CAP){
//                //draw usData
//                System.out.println(getData());
//                //set speed to the rotation speed
//                navigation.setSpeed((float) ROTATION_SPEED);
//                // turn clockwise
//                navigation.forwardLeft();
//                navigation.backwardRight();
//            }
//            //stop the robot
//            leftMotor.stop();
//            rightMotor.stop();
//            
//            try{Thread.sleep(500);} catch(InterruptedException e){} //rotate until wall is seen, note the angle
//            
//            while(getData()==DISTANCE_CAP){
//                //draw usData
//                System.out.println(getData());
//                //set speed to rotation speed
//                navigation.setSpeed((float) ROTATION_SPEED);
//                //turn clockwise
//                navigation.forwardLeft();
//                navigation.backwardRight();
//            }
//            //stop robot
//            leftMotor.stop();
//            rightMotor.stop();
//            //make sound to signal change
//            Sound.playNote(Sound.PIANO, 700, 250);
//            
//            angle1 = odometer.getThetaInDegrees();
//            //draw the angle
//            System.out.println("Angle1 is: "+angle1);
//            //draw usData
//            System.out.println(getData());
//            while(getData() < DISTANCE_CAP){
//                //draw usData
//                System.out.println(getData());
//                //set rotation speed to negative
//                navigation.setSpeed((float) ROTATION_SPEED);
//                // turn counterclockwise
//                navigation.backwardLeft();
//                navigation.forwardRight();
//            }
//            //stop robot
//            leftMotor.stop();
//            rightMotor.stop();
//            //rotate until wall is seen, note angle 
//            while(getData() == DISTANCE_CAP){
//                //draw usData
//                System.out.println(getData());
//                //set rotation speed to negative
//                navigation.setSpeed((float) ROTATION_SPEED);
//                // turn counterclockwise
//                navigation.backwardLeft();
//                navigation.forwardRight();
//            }
//            //stop robot
//            leftMotor.stop();
//            rightMotor.stop();
//            //make sound to signal change
//            Sound.playNote(Sound.PIANO, 700, 250);
//            
//            angle2 = odometer.getThetaInDegrees();
//            //draw angle
//            System.out.println("Angle2 is: "+angle2);
//            //draw usData
//            System.out.println(getData());
//            
//            if(Math.abs(angle1)<Math.abs(angle2)){                                          
//                //since angle1 is clockwise from angle2, average of angles right of angle2 is 45 deg
//                deltAngle = LARGE_ANGLE-((angle1+Math.abs(angle2))/2);
//                try {Thread.sleep(500);}
//                catch(InterruptedException e){}
//                navigation.turnTo(ORIGIN-180);
//                odometer.setTheta(0);
//            }
//            else{
//                deltAngle = SMALL_ANGLE+((angle1+Math.abs(angle2))/2);
//                try {Thread.sleep(500);}
//                catch(InterruptedException e){}
//                navigation.turnTo(ORIGIN-90);
//                odometer.setTheta(0);
//            }
//            //update position --> odometer.gettheta+deltAngle
//            /*odometer.setPosition(new double[] { odometer.getX(), odometer.getY(), odometer.getTheta() + deltAngle },
//                    new boolean[] { true, true, true });*/
//            
//            
//        }
//        
//        else{ //RISING_EDGE
//            while (getData() == DISTANCE_CAP){
//                //draw usData
//                System.out.println(getData());
//                //set RotationSpeed
//                navigation.setSpeed((float) ROTATION_SPEED);
//                // turn clockwise
//                navigation.forwardLeft();
//                navigation.backwardRight();
//            }
//            //keep rotating until the robot sees no wall, then latch the angle
//            while (getData() < DISTANCE_CAP){
//                //draw usData
//                System.out.println(getData());
//                //set RotationSpeed
//                navigation.setSpeed((float) ROTATION_SPEED);
//                // turn clockwise
//                navigation.forwardLeft();
//                navigation.backwardRight();
//            }
//            try {Thread.sleep(500);}catch (InterruptedException e) {}
//            //stop robot
//            leftMotor.stop();
//            rightMotor.stop();
//            //make sound to signal change
//            Sound.playNote(Sound.PIANO, 700, 250);
//            
//            angle1=odometer.getThetaInDegrees();
//            //draw angle
//            System.out.println("Angle1 is: "+angle1);
//            //draw usData
//            System.out.println(getData());
//            
//            // switch direction and wait until it sees a wall
//            while (getData() == DISTANCE_CAP){
//                //draw usData
//                System.out.println(getData());
//                //set RotationSpeed to negative
//                navigation.setSpeed((float) ROTATION_SPEED);
//                // turn counterclockwise
//                navigation.backwardLeft();
//                navigation.forwardRight();
//            }
//            
//            // rotate until no wall is seen, note angle
//            while (getData() < DISTANCE_CAP){
//                //draw usData
//                System.out.println(getData());
//                //set rotation speed to negative
//                navigation.setSpeed((float) ROTATION_SPEED);
//                // turn counterclockwise
//                navigation.backwardLeft();
//                navigation.forwardRight();
//            }
//            //stop robot
//            leftMotor.stop();
//            rightMotor.stop();
//            
//            //make sound to signal change
//            Sound.playNote(Sound.PIANO, 700, 250);
//            
//            angle2=odometer.getThetaInDegrees();
//            //draw angle
//            System.out.println("Angle2 is: "+angle2);
//            //draw usData
//            System.out.println(getData());
//
//            if(Math.abs(angle1)<Math.abs(angle2)){
//                deltAngle = LARGE_ANGLE - ((angle1+Math.abs(angle2))/2);
//                try {Thread.sleep(500);} 
//                catch (InterruptedException e) {}
//                navigation.turnTo(ORIGIN+10);
//                odometer.setTheta(0);
//            } else if(Math.abs(angle1)>Math.abs(angle2)) {
//                deltAngle = SMALL_ANGLE + ((angle1+Math.abs(angle2))/2);
//                try {Thread.sleep(500);} 
//                catch (InterruptedException e) {}
//                navigation.turnTo(-ORIGIN-90);
//                odometer.setTheta(0);
//            }
//            //update position --> odometer.gettheta+deltAngle
//            /*odometer.setPosition(new double[] { odometer.getX(), odometer.getY(), odometer.getTheta() + deltAngle },
//                    new boolean[] { true, true, true });*/
//            Sound.playNote(Sound.PIANO, 700, 250);
//
//        }
//    }
    
    /**
     * 
     * @return Distance from the US sensor in cm
     */
    private static int getData(){
        int dist;
        try { Thread.sleep(50); } catch (InterruptedException e) {}
        
        // there will be a delay here
        usSensor.getDistanceMode().fetchSample(usData, 0);
        dist = (int) (usData[0]*100);
                                                                            
        if(dist>DISTANCE_CAP && count<=3){ //filter for false positives or negatives
            count++; 
            return distance;
        }
        else if(dist>DISTANCE_CAP && count>3){
            return DISTANCE_CAP;
        }
        else{
            count=0;
            distance=dist;
            return dist;
        }
    }
    

}

