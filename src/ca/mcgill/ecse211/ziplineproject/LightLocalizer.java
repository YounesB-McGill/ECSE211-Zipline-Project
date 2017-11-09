package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
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
    private static final int ROTATE_SPEED = 70;
    private static final int ACCE_SPEED = 150;
    private static final int forDis = 15;
    private static final double sensorDis = -4;
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
    // Setting up the light sensor

    /*
     * LightLocalizer Constructor 
     */
    public LightLocalizer() {

        this.lightIntensity = colorSensor.getRedMode();
        this.sampleSize = lightIntensity.sampleSize();
        this.csData = new float[sampleSize];

        leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] { rightMotor });
    }
    
    /**
    * Run the LightLocalization
    */
    public void run() {
        doLightLocalization(0, 0);
    }
    
    /**
    * Do the LightLocalization
    * @param X <i>x</i> coordinate
    * @param Y <i>y</i> coordinate
    */
    public void doLightLocalization(int X, int Y) {

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

        // do a 360 degree clockwise turn, record 4 heading degrees
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
        try { Thread.sleep(3000); } catch (InterruptedException e) {}
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

        try { Thread.sleep(3000); } catch (InterruptedException e) {}

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
    * Do the LightLocalization at (x0,y0)
    */
    public void doLightLocalizationAgain(int X, int Y) {

        // do a 360 degree clockwise turn, record 4 heading degrees
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
        double x = -(sensorDis * Math.cos(Math.PI * thetaY / (2 * 180))) + X * TILE;
        // double x = X * Tile_length;

        //double thetaX = heading[2] + 360 - heading[0];
        //double y = -(-sensorDis * Math.cos(Math.PI * thetaX / (2 * 180))) + Y * Tile_length;
        double y = Y * TILE;
        odometer.setX(x);
        odometer.setY(y);

        Sound.beep();
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
        }

        navigation.travelTo(X, Y);
        navigation.turnTo(navigation.transferAngle(90));
        Sound.beep();

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
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
        leftMotor.startSynchronization();
        leftMotor.forward();
        rightMotor.backward();
        leftMotor.endSynchronization();
    }
    
    /**
    * Make robot rotate counterclockwise
    */
    private void counterclockwise() {
        leftMotor.startSynchronization();
        leftMotor.backward();
        rightMotor.forward();
        leftMotor.endSynchronization();
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
    * Make both motors go forward
    */
    private void driveForward() {
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.forward();
        rightMotor.forward();
    }

    /**
    * Make both motors drive back a bit
    */
    private void driveBackABit() {
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.rotate(-convertDistance(WHEEL_RADIUS, forDis), true);
        rightMotor.rotate(-convertDistance(WHEEL_RADIUS, forDis), false);
    }

    /**
    * Convert to wheel rotations based on the wheel radius and travel distance
    * @param radius Wheel radius of the regular EV3 wheels
    * @param distance Desired distance to travel
    * @returns Number of degrees that motor must rotate to travel the required distance
    */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
    * Calls convertDistance to get wheel rotations based on the wheel radius, width, and angle
    * @param radius Wheel radius of the regular EV3 wheels
    * @param width Width of the robot (TRACK)
    * @param angle Angle that defines the distance
    * @returns Number of degrees that motor must rotate to travel the required distance
    */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }

}
