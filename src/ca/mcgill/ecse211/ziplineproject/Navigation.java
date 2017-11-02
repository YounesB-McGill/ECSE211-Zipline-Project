package ca.mcgill.ecse211.ziplineproject;

import java.awt.*;
import java.awt.event.*;

import ca.mcgill.ecse211.ziplineproject.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * This class provides the logic needed for the robot to navigate the game field,
 * allowing it to travel to specified points and orient itself to various angles.
 * 
 * @author Chaoyi Liu
 *
 */
public class Navigation {
    
    //initializing variables
    public static final double WHEEL_RADIUS = Main.WHEEL_RADIUS;
    public static final double TRACK = Main.TRACK;
    public static final double TILE = Main.TILE;
    
    private static final int FORWARD_SPEED = Main.FWD_SPEED;
    public static final int ROTATE_SPEED = Main.ROTATE_SPEED;
    
    
    public static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
    public static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
    //public static UltrasonicPoller usPoller;
    
    public static Odometer odometer = Main.odometer;
    
    public Object lock;

    
    // Points array of all possible trajectories
    public static double[][][] points;
    public static int version;
    public static int xBefore;
    public static int yBefore;
    
    public static int x0;
    public static int y0;
    
    double[] position = new double[3];
    private double nowX;
    private double nowY;
    private double nowTheta;

    public boolean isTurning = false;
    public boolean isTraveling = false;
    public boolean inDanger = false;
    
    private static long SLEEPINT = 1500;
    
    public Navigation(){
        lock = new Object();
    }
    
    public void run(){
      
        for(int i =0; i <= points.length; i++){
            // Remember last point before obstacle is encountered
            setXBefore((int) points[version-1][i][0]);
            setYBefore((int) points[version-1][i][1]);
            
            travelTo( points[version-1][i][0], points[version-1][i][1] );
        }
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
         forward();
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
    
    public void moveTo(double x, double y) {
        isTraveling = true;
        x = x * TILE;
        y = y * TILE;
        while (isTraveling) { 
            // getting the current position of robot
            double currX = odometer.getX();
            double currY = odometer.getY();
            // getting the theta of the destination, converting to degrees
            double destTheta = Math.atan2((x - currX), (y - currY)) * 180 / Math.PI;
            // getting distance needed to travel
            double distance = Math.sqrt((x - currX) * (x - currX) + (y - currY) * (y - currY));
            double deltaX = (x - currX);
            double deltaY = (y - currY);
            // Rotate to point towards destination theta
            turnTo(destTheta);
            
            try {Thread.sleep(SLEEPINT);} catch (InterruptedException e) {}
            
            forward(distance);

            isTraveling = false;
        }
    }
    
    public void moveToTrue(double x, double y) {
        isTraveling = true;
        x = x * TILE;
        y = y * TILE;
        while (isTraveling) { 
            // getting the current position of robot
            double currX = odometer.getX();
            double currY = odometer.getY();
            // getting the theta of the destination, converting to degrees
            double destTheta = Math.atan2((x - currX), (y - currY)) * 180 / Math.PI;
            // getting distance needed to travel
            double distance = Math.sqrt((x - currX) * (x - currX) + (y - currY) * (y - currY));
            double deltaX = (x - currX);
            double deltaY = (y - currY);
            // Rotate to point towards destination theta
            turnTo(destTheta);
            
            try {Thread.sleep(SLEEPINT);} catch (InterruptedException e) {}
            
            forwardTrue(distance);

            isTraveling = false;
        }
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
        leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, turningTheta), true);
        rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, turningTheta), false);
        isTurning = false;
    }

    /**
    * @returns <code>true</code> if the robot is either Navigating or Turning
    */
    public boolean isNavigating(){
        return isTurning || isTraveling;
    }

    public static void turn(double diff){
        leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, diff), true);
        rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, diff), false);
    }

    public static void turnLeft(double diff){
        leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, diff), true);
        rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, diff), false);
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
     * Set the speed for the left motor.
     * @param {float} speed - The speed of the motor in degrees per second
     */
    public static void setLeftSpeed(float speed) {
        leftMotor.setSpeed(speed);
    }

    /**
     * Set the speed for the right motor.
     * @param {float} speed - The speed of the motor in degrees per second
     */
    public static void setRightSpeed(float speed) {
        rightMotor.setSpeed(speed);
    }
    
    /**
     * Set the speed for both the left and right motors.
     * @param {float} speed - The speed of the motor in degrees per second
     */
    public static void setSpeed(float speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }
    
    /**
     * Make left motor go forward
     */
    public static void forwardLeft(){
        leftMotor.forward();
    }
    
    /**
     * Make right motor go forward
     */
    public static void forwardRight(){
        rightMotor.forward();
    }

    /**
     * Make both right and left motors go forward
     */
    public static void forward(){
        forwardLeft();
        forwardRight();
    }
    
    /**
     * Make left motor go backward
     */
    public static void backwardLeft(){
        leftMotor.backward();
    }
    
    /**
     * Make right motor go backward
     */
    public static void backwardRight(){
        rightMotor.backward();
    }

    /**
     * Make both right and left motors go backward
     */
    public static void backward(){
        backwardLeft();
        backwardRight();
    }
    
    /**
     * Go forward for a specified distance
     * @param {double} distance - Desired distance in cm.
     */
    public static void forward(double distance){
        leftMotor.rotate(convertDistance(WHEEL_RADIUS, distance), true);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS, distance), false);
    }
    
    public static void forwardTrue(double distance){
        leftMotor.rotate(convertDistance(WHEEL_RADIUS, distance), true);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS, distance), true);
    }

    public boolean getIsTurning() {
        synchronized (lock) { return isTurning; }
    }

    public boolean getIsTraveling() {
        synchronized (lock) { return isTraveling; }
    }

    public boolean getInDanger() {
        synchronized (lock) { return inDanger; }
    }

    public void setIsTurning(boolean isTurning) {
        synchronized (lock) { this.isTurning = isTurning; }
    }

    public void setIsTraveling(boolean isTraveling) {
        synchronized (lock) { this.isTraveling = isTraveling; }
    }

    public void setInDanger(boolean inDanger) {
        synchronized (lock) { this.inDanger = inDanger; }
    }

    /**
     * Stop the left and right motors
     */
    public static void stopMotors() {
        leftMotor.stop();
        rightMotor.stop();
    }

    public int getXBefore() {
        synchronized (lock) { return xBefore; }
    }

    public int getYBefore() {
        synchronized (lock) { return yBefore; }
    }

    public void setXBefore(int xBefore) {
        synchronized (lock) { Navigation.xBefore = xBefore; }
    }

    public void setYBefore(int yBefore) {
        synchronized (lock) { Navigation.yBefore = yBefore; }
    }
    
    public double transferAngle(double angle) {
        // convert to polar coordinate
        angle = (360.0 - angle) + 90.0;
        return angle;
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

}