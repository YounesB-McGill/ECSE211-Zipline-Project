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
public final class LightLocalizer { // Does NOT extend Thread, nor is it a Runnable

    private static double WHEEL_RADIUS = Main.WHEEL_RADIUS;
    private static double TRACK = Main.TRACK;
    private static final int FORWARD_SPEED = 120;
    private static final int ROTATE_SPEED = 80;
    private static final int ACCE_SPEED = 150;
    private static final int forDis = 15;
    private static final double sensorDis = -8; // -4
    private static final double TILE = Main.TILE;
    private static final double THETA_OFFSET = 9;
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
    // Setting up the light sensor

    /**
     * LightLocalizer Constructor
     */
    @SuppressWarnings("static-access") // Need to use same navigation and motor objects in this class
    public LightLocalizer() {

        this.lightIntensity = colorSensor.getRedMode();
        this.sampleSize = lightIntensity.sampleSize();
        this.csData = new float[sampleSize];

        navigation.setSpeed(Main.FWD_SPEED);
        navigation.setAcceleration(Main.FWD_ACC);
    }

    // No run() method needed

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
    public void resetAccordingToCorner() {
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

    public void dotest() {
        // clockwise(90,false);
        // counterclockwise(90,false);
        // driveForward(30.98,false);
        // driveBackABit();
        navigation.turnTo(270);
        navigation.turnTo(181);
    }

    /**
     * Do the LightLocalization. This method is recursive.
     * 
     * @param X <i>x</i> coordinate
     * @param Y <i>y</i> coordinate
     */
    @SuppressWarnings("static-access") // Need to use same navigation and motor objects in this class
    public void doLightLocalization(int X, int Y) {
        // turn 360 degrees
        clockwise(360, true);

        // record headings
        double[] heading = new double[4];
        int i = 0;
        while (leftMotor.isMoving() && i < 4) {
            if (hitGridLine()) {
                Sound.playNote(Sound.PIANO, 3*(180+60*i), 450);
                heading[i] = odometer.getThetaInDegrees();
                i++;
            }
        }

        // check if detect 4 points
        if (i < 4) {
            Sound.beepSequence();
            // move to a better location, and do it again
            if (i == 0 || i == 1) {
                // wait to decide the direction
                navigation.travelTo(X, Y);
            } else if (i == 2 || i == 3) {
                // wait to decide the direction
                navigation.turnTo(heading[0]);
            }

            navigation.travelFor(10);
            Sound.beep();
            doLightLocalization(X, Y);
            return;
        }

        // find 4 angles and find the Min between them
        double[] angle = new double[4];
        angle = getAngles(heading, angle);
        double MinAng = Math.min(angle[0], Math.min(angle[1], Math.min(angle[2], angle[3])));

        // check if detect 4 correct points
        if (MinAng < 40) {
            Sound.beepSequence();
            Sound.beepSequence();
            // move to a better location, and do it again
            double directionAngle = findNewDirection(heading, angle, MinAng);
            navigation.turnTo(directionAngle);
            navigation.travelFor(-10);
            Sound.beep();
            doLightLocalization(X, Y);

            return;
        }

        // now the location is good, do the real light localization
        // first calculate according to headings
        double theta1 = calculateAngle(heading[1], heading[3]);
        double direction1 = calculateAverageAngle(heading[1], heading[3]);
        double distance1 = sensorDis * Math.cos((Math.PI * theta1 / 180) / 2);

        double theta2 = calculateAngle(heading[0], heading[2]);
        double direction2 = calculateAverageAngle(heading[0], heading[2]);
        double distance2 = sensorDis * Math.cos((Math.PI * theta2 / 180) / 2);

        // do the localization
        navigation.turnTo(direction1);
        navigation.travelFor(distance1);
        Sound.beep();
        navigation.turnTo(direction2);
        navigation.travelFor(distance2);

        // stop motor
        stopMotor();

        Sound.beepSequenceUp();
        // set odometer
        odometer.setX(X * TILE);
        odometer.setY(Y * TILE);

        // turn 360 degrees
        clockwise(360, true);

        while (leftMotor.isMoving()) {
            if (hitGridLine()) {
                Sound.beep();
                break;
            }
        }
        stopMotor();
        double currentTheta = odometer.getThetaInDegrees();
        int calibration = setToClosestTheta(currentTheta);
        odometer.setTheta((calibration * Math.PI / 180));
        
        /* STM MR-73 */{
            Sound.playNote(Sound.PIANO, 180*3, 450);
            Sound.playNote(Sound.PIANO, 240*3, 450);
            Sound.playNote(Sound.PIANO, 360*3, 500);
        }
        
        // TODO Calculate offset as a function of the distance
        navigation.turnLeftBy(9);
        odometer.setTheta(0); // Also change this based on odometer state
        
        return;
    }

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

    private double findNewDirection(double[] heading, double[] Angle, double MinAng) {
        double direction = 0;
        for (int i = 0; i < 4; i++) {
            if (Angle[i] == MinAng) {
                if (i != 3) {
                    direction = calculateAverageAngle(heading[i], heading[i + 1]);
                } else {
                    direction = calculateAverageAngle(heading[3], heading[0]);
                }
            }
        }
        return direction;
    }

    private double calculateAverageAngle(double a, double b) {
        double avg;
        if (a <= b) // normal condition
            avg = (a + b) / 2;
        else { // when the second angle passes 0
            avg = ((a - 360) + b) / 2;
            if (avg < 0) {
                avg = avg + 360;
            }
        }
        return avg;
    }

    // get the angles
    private double[] getAngles(double[] heading, double[] Angle) {
        for (int i = 0; i < 4; i++) {
            if (i != 3) {
                Angle[i] = calculateAngle(heading[i], heading[i + 1]);
            } else {
                Angle[3] = calculateAngle(heading[3], heading[0]);
            }
        }
        return Angle;
    }

    /** calculate the angle between 2 headings */
    private double calculateAngle(double a, double b) {
        double angle;
        if (a <= b) // normal condition
            angle = b - a;
        else // when the second angle passes 0
            angle = b - (a - 360);
        return angle;
    }

    /**
     * Make robot rotate clockwise for a certain angle
     */
    private void clockwise(double theta, boolean con) {
        setRotateSpeed();
        //leftMotor.startSynchronization();
        leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);
        rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), true); // immediate return
    }

    /**
     * Make robot rotate counter clockwise for a certain angle
     */
    public void counterclockwise(double theta, boolean con) {
        setRotateSpeed();
        //leftMotor.startSynchronization();
        leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta), true);
        rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta), true);
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
        leftMotor.forward();
        rightMotor.backward();
    }

    /**
     * Make robot rotate counterclockwise
     */
    public void counterclockwise() {
        leftMotor.backward();
        rightMotor.forward();
    }

    /**
     * Make robot stop
     */
    private void stopMotor() {
        leftMotor.stop(true);
        rightMotor.stop(false);
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
     * Make both motors drive back a bit
     */
    private void driveBackABit() {
        setForwardSpeed();
        leftMotor.rotate(-convertDistance(WHEEL_RADIUS, forDis), true);
        rightMotor.rotate(-convertDistance(WHEEL_RADIUS, forDis), false);
    }

    /**
     * Convert to wheel rotations based on the wheel radius and travel distance
     * 
     * @param radius Wheel radius of the regular EV3 wheels
     * @param distance Desired distance to travel
     * @returns Number of degrees that motor must rotate to travel the required distance     
     */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
     * Calls convertDistance to get wheel rotations based on the wheel radius,
     * width, and angle
     * 
     * @param radius Wheel radius of the regular EV3 wheels
     * @param width Width of the robot (TRACK)
     * @param angle Angle that defines the distance
     * @returns Number of degrees that motor must rotate to travel the required distance
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }

}
