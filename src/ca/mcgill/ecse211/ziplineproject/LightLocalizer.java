package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * This class is responsible for light localization using the color sensor.
 * 
 * @author Chaoyi Liu
 */
public final class LightLocalizer extends Thread {

	private static double WHEEL_RADIUS = Main.WHEEL_RADIUS;
	private static double TRACK = Main.TRACK;
	private static final int FORWARD_SPEED = 120;
	private static final int ROTATE_SPEED = 80;
	private static final int ACCE_SPEED = 150;
	private static final int forDis = 15;
	private static final double sensorDis = -11; // -4
	private static final double TILE = Main.TILE;
	private double intensity;
	private int startCorner = Main.startCorner;

	private EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
	private EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
	private EV3ColorSensor colorSensor = Main.cSensor;

	private Odometer odometer = Main.odometer;
	private Navigation navigation = Main.navigation;

	private SampleProvider lightIntensity;
	private int sampleSize;
	private float[] csData;
	private int type;
	// Setting up the light sensor

	/*
	 * LightLocalizer Constructor
	 */
	public LightLocalizer(int type) {

		this.lightIntensity = colorSensor.getRedMode();
		this.sampleSize = lightIntensity.sampleSize();
		this.csData = new float[sampleSize];
		this.type=type;
		leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] { rightMotor });
	}

	/**
	 * Run the LightLocalization
	 */
	public void run() {
		if(type==0)	{
			doLightLocalizationBegin();
			doLightLocalizationNew(1, 1);
		}
		if(type==1) doLightLocalizationNew(2, 1);
	}

	
	public void doLightLocalizationBegin() {

		// drive forwards
		driveForward();

		// when hit a grid line, stop
		stopAtGridline();

		// drive backward a bit
		driveBackABit();

		// turn 90 degrees clockwise and drive forwards again
		setRotateSpeed();
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 90.0), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 90.0), false);

		driveForward();

		stopAtGridline();

		// drive backward a small distance
		driveBackABit();

	/*	// do a 360 degree clockwise turn, record 4 heading degrees
		setRotateSpeed();
		clockwise();
		double[] heading = new double[4];
		int i = 0;
		while (i < 4) {
			if (hitGridLine()) {
				Sound.beep();
				heading[i] = odometer.getTheta();
				i++;
			}
		}
		// heading[3]: vertical line lower point
		// heading[0]: horizontal line left point
		// heading[2]: horizontal line right point
		// heading[1]: vertival line upper point
		double thetaY = Math.abs(heading[1] - heading[3]);
		double x = sensorDis * Math.cos(Math.PI * thetaY / (2 * 180)) + X;

		double thetaX = heading[2] + 360 - heading[0];
		double y = -sensorDis * Math.cos(Math.PI * thetaX / (2 * 180)) + Y;

		odometer.setX(x);
		odometer.setY(y);

		Sound.beep();

		navigation.travelTo(X, Y);
		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
		}
		navigation.turnTo(navigation.transferAngle(0));

		// correction
		setRotateSpeed();
		leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 5.0), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 5.0), false);

		odometer.setTheta(0);
		resetAccordingToCorner();

		leftMotor.startSynchronization();
		leftMotor.stop(true);
		rightMotor.stop(true);
		leftMotor.endSynchronization();

		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
		}
*/
	}

	/**
	 * do test things, ignore it 
	 *//*
	public void dotest() {
		//clockwise(90, false);
		// counterclockwise(90,false);
		// driveForward(30.98,false);
		// driveBackABit();
		//navigation.turnLeftBy(90);
	    //navigation.turnTo(180);
		int button = 0;

		while (button == 0)
			button = Button.waitForAnyPress();
		dotest();
	}
	*/
	
/**
	 *   This is the new LightLocalization method, which fits for localizing at any point 
	 * on the Panel. The argument X and Y is the approximate coordinates of the robot's 
	 * location.
	 * 
	 * @param X
	 *            <i>x</i> coordinate
	 * @param Y
	 *            <i>y</i> coordinate
	 */
	public void doLightLocalizationNew(int X, int Y) {
		// turn 360 degrees
		clockwise(360, true);

		// record a heading when detect a grid line
		double[] heading = new double[4];
		int i = 0;
		while (leftMotor.isMoving() && i < 4) {
			if (hitGridLine()) {
				Sound.beep();
				heading[i] = odometer.getTheta() * 180 / Math.PI;
				i++;
			}
		}

        //if detect less than 4 grid line, the location is not good
        //and need to move to a better location
		if (i < 4) {
			Sound.beepSequenceUp();
			Sound.beepSequenceUp();
			// move to a better location, and do it again
			if (i == 0 || i == 1) {
				//the robot locate at the center of a block
				//so we drive forward until we find a grid line
				driveForward();
				while (true) {
					if (hitGridLine()) {			
						stopMotor();
						Sound.beep();
						break;
					}
				}
				driveBackABit();
				
			} else if (i == 2) {
				// the robot locates on a line 
				//so we follow the line to find a point 
				navigation.turnTo(heading[1]);
				driveForward(12, false);
			} else if (i == 3){
				double[] angle = new double[3];
				angle = getAngles(heading, angle);
				double MinAng = Math.min(angle[0], Math.min(angle[1], angle[2]));
				double directionAngle = findNewDirection(heading, angle, MinAng);
				//here turnTo method go to the reverse direction, so I just drive backwards
				navigation.turnTo(directionAngle);
				driveForward(-10, false);
			}
			
			//recursively call doLightLocalization
			doLightLocalizationNew(X, Y);
			return;
		}

		// find the 4 angles between 4 headings and find the Minimum value between them
		double[] angle = new double[4];
		angle = getAngles(heading, angle);
		double MinAng = Math.min(angle[0], Math.min(angle[1], Math.min(angle[2], angle[3])));

		
		//if there is an angle too small(less than 40 degrees), 
		//then the location of robot is not good.
		if (MinAng < 40) {
			Sound.beepSequence();
			Sound.beepSequence();
			// move to a better location, and do it again
			// here we move to the direction of the smallest angle
			double directionAngle = findNewDirection(heading, angle, MinAng);
			//here turnTo method go to the reverse direction, so I just drive backwards
			navigation.turnTo(directionAngle);
			driveForward(-10, false);
			//recursively call doLightLocalization
			doLightLocalizationNew(X, Y);
			return;
		}

		
		// now the location is good, do the real light localization
		// first calculate according to headings
		double theta1 = calAng(heading[1], heading[3]);
		double direction1 = calAveAng(heading[1], heading[3]);
		double distance1 = sensorDis * Math.cos((Math.PI * theta1 / 180) / 2);

		double theta2 = calAng(heading[0], heading[2]);
		double direction2 = calAveAng(heading[0], heading[2]);
		double distance2 = sensorDis * Math.cos((Math.PI * theta2 / 180) / 2);

		
		// do the localization
		navigation.turnTo(direction1);
		driveForward(distance1, false);
		navigation.turnTo(direction2);
		driveForward(distance2, false);

		// stop motor
		stopMotor();
		Sound.beepSequenceUp();
		
		//find the actual location x,y
		double currentX=findClosestCoordinate(odometer.getX(),X*TILE);
		double currentY=findClosestCoordinate(odometer.getY(),Y*TILE);
		
		// set odometer
		odometer.setX(currentX);
		odometer.setY(currentY);

		// turn 360 degrees
		clockwise(360, true);

		//when a grid line is detected, 
		//stop and set Theta to the closest direction between 4 directions
		while (leftMotor.isMoving()) {
			if (hitGridLine()) {			
				stopMotor();
				Sound.beep();
				break;
			}
		}		
		double currentTheta = odometer.getTheta() * 180 / Math.PI;
		int calibration = setToClosestTheta(currentTheta);
		odometer.setTheta(calibration * Math.PI / 180);

		//driveForward(40,false);
		/*// Wait for button press
		int button = 0;
		while (button == 0)
			button = Button.waitForAnyPress();
		doLightLocalizationNew(3, 2);*/

	}

	/**
	 * On a panel, find the closest crossing point's coordinate value 
	 * according to current value.
	 * @param currentVal
	 * @param estimateVal
	 * @return
	 */
	private double findClosestCoordinate(double currentVal, double estimateVal) {
		double calibratedVal=estimateVal;
		while(Math.abs(calibratedVal-currentVal)> TILE/2){
			if(calibratedVal>currentVal)
				calibratedVal=calibratedVal-TILE;
			else
				calibratedVal=calibratedVal+TILE;
		}	
	    return calibratedVal;
    }

    /**
     * When the robot locates exactly on a crossing point,
     * we turn the robot until we detect a grid line.
     * Now theta can have 4 possibilities: 0, 90, 180, 360.
     * This method helps figure out which to choose between these 4 directions.
     * It returns in degree.
     * @param currTheta
     * @return
     */
	private int setToClosestTheta(double currTheta) {
		double[] error = new double[4];
		double closestValue = 360;
		int closest = 0;
		for (int i = 0; i < 4; i++) {
			error[i] = Math.abs(currTheta - i * 90);
			if (closestValue > error[i]) {
				closestValue = error[i];
				closest = i * 90; // 2 is for the delay of stop
			}
		}
		if (closestValue > 45) {
			if (Math.abs(currTheta - 360) < 40)
				closest = 0;
		}
		return closest;
	}

	/**
	 *   When we detect 4 grid lines but there is an angle smaller
	 * than a threshold(I set it as 40), the location of robot may not 
	 * be good. Moving to the direction of the smallest angle can increase
	 * the angle value. This method helps us find this direction.
	 * it returns in degree.
	 * @param heading
	 * @param Angle
	 * @param MinAng
	 * @return
	 */
	private double findNewDirection(double[] heading, double[] Angle, double MinAng) {
		double direction = 0;
		for (int i = 0; i < heading.length; i++) {
			if (Angle[i] == MinAng) {
				if (i !=  heading.length-1) {
					direction = calAveAng(heading[i], heading[i + 1]);
				} else {
					direction = calAveAng(heading[i], heading[0]);
				}
			}
		}
		return direction;
	}

	/**
	 * Given 2 angles in degree, find the average angle between them. Wrap around 
	 * take into consideration here.
	 * It returns in degree.
	 * @param a
	 * @param b
	 * @return
	 */
	private double calAveAng(double a, double b) {
		double Ave;
		if (a <= b) // normal condition
			Ave = (a + b) / 2;
		else { // when the second angle passes 0
			Ave = ((a - 360) + b) / 2;
			if (Ave < 0) {
				Ave = Ave + 360;
			}
		}
		return Ave;
	}

	/**
	 * Give an array of headings, store the angle of neighboring headings in another array
	 * @param heading
	 * @param Angle
	 * @return
	 */
	private double[] getAngles(double[] heading, double[] Angle) {
		for (int i = 0; i < heading.length; i++) {
			if (i != heading.length-1) {
				Angle[i] = calAng(heading[i], heading[i + 1]);
			} else {
				Angle[i] = calAng(heading[i], heading[0]);
			}
		}
		return Angle;
	}
	
    
	/**
	 * Calculate the angle between 2 headings.
	 * Wrap around take into consideration here.
	 * It returns in degree.
	 * @param a
	 * @param b
	 * @return
	 */
	private double calAng(double a, double b) {
		double Ang;
		if (a <= b) // normal condition
			Ang = b - a;
		else // when the second angle passes 0
			Ang = b - (a - 360);
		return Ang;
	}

	/**
	 * Stop at gridline
	 */
	public void stopAtGridline() {
		while (true) {
			if (hitGridLine()) {
				Sound.beep();
				break;
			}
		}
	}

	/**
	 * Reset the odometer based on the starting corner
	 */
	private void resetAccordingToCorner() {
		if (startCorner == 0) {
			odometer.setX(1 * TILE);
			odometer.setY(1 * TILE);
		} else if (startCorner == 1) {
			// reset x,y,theta
			odometer.setX(7 * TILE);
			odometer.setY(1 * TILE);
			odometer.setTheta(Math.PI * 3 / 2);
		} else if (startCorner == 2) {
			// reset x,y,theta
			odometer.setX(7 * TILE);
			odometer.setY(7 * TILE);
			odometer.setTheta(Math.PI);
		} else if (startCorner == 3) {
			// reset x,y,theta
			odometer.setX(1 * TILE);
			odometer.setY(7 * TILE);
			odometer.setTheta(Math.PI / 2);
		}

	}

	/**
	 * Make both motors go forward for a certain distance
	 * The second argument controls whether the next instruction(s)
	 * should be executed when this rotation hasn't ended.
	 */
	private void driveForward(double distance, boolean con) {
		setForwardSpeed();
		leftMotor.startSynchronization();
		leftMotor.rotate(convertDistance(WHEEL_RADIUS, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, distance), true);
		leftMotor.endSynchronization();
		if (con == false) {
			leftMotor.waitComplete();
			rightMotor.waitComplete();
		}
	}

	/**
	 * Make robot rotate clockwise for a certain angle
	 * The second argument controls whether the next instruction(s)
	 * should be executed when this rotation hasn't ended.
	 */
	private void clockwise(double theta, boolean con) {
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
	 * Make robot rotate counter clockwise for a certain angle.
	 * The second argument controls whether the next instruction(s)
	 * should be executed when this rotation hasn't ended.
	 */
	private void counterclockwise(double theta, boolean con) {
		setRotateSpeed();
		leftMotor.startSynchronization();
		leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);
		leftMotor.endSynchronization();
		if (con == false) {
			leftMotor.waitComplete();
			rightMotor.waitComplete();
		}
	}

	/**
	 * Returns true if a grid line is crossed
	 */
	private boolean hitGridLine() {
		lightIntensity.fetchSample(csData, 0);
		intensity = csData[0];
		return intensity < 0.3;
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

	/**
	 * Set acceleration of the motors
	 */
	private void setAcce() {
		leftMotor.setAcceleration(ACCE_SPEED);
		rightMotor.setAcceleration(ACCE_SPEED);
	}

	/**
	 * Set rotation speed of the motors
	 */
	private void setRotateSpeed() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
	}

	/**
	 * Set forward speed of the motors
	 */
	private void setForwardSpeed() {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	}

	/**
	 * Make both motors go forward
	 */
	private void driveForward() {
		setForwardSpeed();
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}

	/**
	 * Make both motors drive back a bit.
	 * The next instruction will be executed after this rotation ends.
	 */
	private void driveBackABit() {
		setForwardSpeed();
		leftMotor.startSynchronization();
		leftMotor.rotate(-convertDistance(WHEEL_RADIUS, forDis), true);
		rightMotor.rotate(-convertDistance(WHEEL_RADIUS, forDis), true);
		leftMotor.endSynchronization();
		leftMotor.waitComplete();
		rightMotor.waitComplete();
	}

	/**
	 * Convert to wheel rotations based on the wheel radius and travel distance
	 * 
	 * @param radius
	 *            Wheel radius of the regular EV3 wheels
	 * @param distance
	 *            Desired distance to travel
	 * @returns Number of degrees that motor must rotate to travel the required
	 *          distance
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
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

}
