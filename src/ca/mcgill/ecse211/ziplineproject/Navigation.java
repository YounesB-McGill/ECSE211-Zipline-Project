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
public class Navigation extends Thread{
    
    //initializing variables
    /**The radius of the robot's wheel*/public static final double WHEEL_RADIUS = Main.WHEEL_RADIUS;
    /**The width of the robot */public static final double TRACK = Main.TRACK;
    /**The length of one competition floor tile, 30.48 cm.*/public static final double TILE = Main.TILE;
    
    /**The speed at which the robot travels forward*/private static final int FORWARD_SPEED = Main.FWD_SPEED;
    /**The speed at which the robot rotates*/public static final int ROTATE_SPEED = Main.ROTATE_SPEED;
    
    
    /**The EV3 left motor*/public static EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
    /**The EV3 right motor*/public static EV3LargeRegulatedMotor rightMotor = Main.rightMotor;
    //public static UltrasonicPoller usPoller;
    
    /**Initialization of the odometer which calculates the position and orientation of the robot*/
    public static Odometer odometer = Main.odometer;
   
    private int startCorner = Main.greenCorner;
    /**Used to synchronize access to a resource across multiple threads*/public Object lock;

    
    // Points array of all possible trajectories
    public static double[][][] points;
    public static int version;
    public static int xBefore;
    public static int yBefore;
    
    public int x;
    public int y;
    public int type;
    
    double[] position = new double[3];
    private double nowX;
    private double nowY;
    private double nowTheta;

    public boolean isTurning = false;
    public boolean isTraveling = false;
    public boolean inDanger = false;
    /** Track whether the robot has arrived to a destination*/public boolean reachedDestination = false;
    
    private static long SLEEPINT = 150;
    
    public Navigation(int x, int y,int type){
        lock = new Object();
        this.x=x;
        this.y=y;
        this.type=type;
        //leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] { rightMotor });
    }
    
    public void run(){
      /*
        for(int i =0; i <= points.length; i++){
            // Remember last point before obstacle is encountered
            setXBefore((int) points[version-1][i][0]);
            setYBefore((int) points[version-1][i][1]);
            
            travelTo( points[version-1][i][0], points[version-1][i][1] );
        }*/
    	if(type==0){
    		//localization ends, navigation starts
            if(startCorner == 0 || startCorner == 3) { // SW, NW
            	travelTo(Main.zo_g_x,Main.zo_g_y); // This will take us to (x0, y0)
            } else if(startCorner == 1) { // SE|
                travelTo(1, 1); //avoid entering zip line area
                travelTo(Main.zo_g_x,Main.zo_g_y); // This will take us to (x0, y0)
            } else { // NE
                travelTo(1, 7); //avoid entering zip line area
                travelTo(Main.zo_g_x,Main.zo_g_y); // This will take us to (x0, y0)
            }
            
          /*  double nowx=odometer.getX();
            double nowy=odometer.getY();
            if(nowx!=Main.zo_g_x || nowy!=Main.zo_g_y){
            	travelTo(Main.zo_g_x,Main.zo_g_y);
            }*/
    	   
    	}
    	else{
    		travelTo(x,y);
    	}
    	   
    }

    /**
     * Travel to specified point
     * @param x <i>x</i> coordinate based on gridlines
     * @param y <i>y</i> coordinate based on gridlines
     */
     public void travelTo(double x, double y) {
         
         /*travelTo(0.1*x, 0.1*y, (int) (0.2*FORWARD_SPEED));
         travelTo(0.2*x, 0.2*y, (int) (0.5*FORWARD_SPEED));
         travelTo(0.5*x, 0.5*y, FORWARD_SPEED);
         travelTo(0.8*x, 0.8*y, (int) (0.5*FORWARD_SPEED));
         travelTo(x, y, (int) (0.2*FORWARD_SPEED));*/
         
         travelTo(x, y, (int) (FORWARD_SPEED));
         
         // Stop motors when we reach destination
         //leftMotor.startSynchronization();
         leftMotor.stop();
         rightMotor.stop();
         //leftMotor.endSynchronization();
         
     }
     
     public void travelTo(double x, double y, int speed) {
         isTraveling = true;
         
         double xSaver=x;
         double ySaver=y;
         
         // Convert to cm
         x = x * TILE;
         y = y * TILE;
         
         // getting the current position of robot
         double currX = odometer.getX();
         double currY = odometer.getY();
         
         double deltaX = x - currX;
         double deltaY = y - currY;
         
         // getting the theta of the destination, converting to degrees
         double destTheta = Math.atan2(deltaX, deltaY) * 180 / Math.PI;
         
         // getting distance needed to travel
         double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
         
         if(distance>3*TILE+20){
        	 // turn to destination theta
             setSpeed(0);
             turnTo(destTheta);
             
             try {Thread.sleep(SLEEPINT);} catch (InterruptedException e) {}
             
             setSpeed(0);
             setSpeed(speed);
             forward(2*TILE);
                     	
        	 LightLocalizer lo=new LightLocalizer(1);
        	 lo.run();
        	
        	 travelTo(xSaver,ySaver,speed);
        	 return;
         }
         
         // turn to destination theta
         setSpeed(0);
         turnTo(destTheta);
         
         try {Thread.sleep(SLEEPINT);} catch (InterruptedException e) {}
         
         // travel to destination
         setSpeed(0);
         setSpeed(speed);
         forward(distance);
         stopMotor();
         
         isTraveling = false;
                        
    } // end travelTo()
    
   
    /* (Template for a JavaDoc w picture)
     * Turn to specified angle
     * 
     * <pre><img src="{@docRoot}/doc-files/surface-subway.png" /></pre>
     * 
     * @param theta Theta in degrees
     * 
     */

    /**
    * Turn to specified angle
    * 
    * @param theta Theta in degrees
    * 
    */
    public void turnTo(double theta) {
        isTurning = true;
        double currTheta = odometer.getThetaInDegrees();
        
        // Convert both thetas to [-180,180) scale
        theta = convertAngleTo180Scale(theta);
        currTheta = convertAngleTo180Scale(currTheta);
        
        double otherSide = currTheta-180;
        otherSide = convertAngleTo180Scale(otherSide);
        
        setSpeed(0);
        setSpeed(ROTATE_SPEED);
        
        double deltaTheta;
        if(theta*currTheta>=0) { // if they're both in the ranges [0,180) or [-180,0)
            // No discontinuity in either 180� or 360� scale
            deltaTheta = Math.abs(theta - currTheta);
            if(currTheta>theta) {
                // Turn left by deltaTheta
                turnLeftBy(deltaTheta);
            } else {
                // Turn right by deltaTheta
                turnRightBy(deltaTheta);
            }
        } else {
            /* Handle discontinuity by comparing the angle difference in both scales, [-180,180) and [0,360),
             * and select the scale that has the lesser angle difference to turn to the minimal angle */
            double theta360=theta, currTheta360=currTheta;
            if(theta<0) theta360 += 360; 
            if(currTheta<0) currTheta360 += 360;
            
            double deltaTheta180 = Math.abs(theta-currTheta);
            double deltaTheta360 = Math.abs(theta360-currTheta360);
            if(deltaTheta180 <= deltaTheta360) {
                // Use 180� scale
                if(currTheta>theta) {
                    // Turn left by deltaTheta in the 180� scale
                    turnLeftBy(deltaTheta180);
                } else {
                    // Turn right by deltaTheta in the 180� scale
                    turnRightBy(deltaTheta180);
                }
            } else {
                // Use 360� scale
                if(currTheta360>theta360) {
                    // Turn left by deltaTheta in the 360� scale
                    turnLeftBy(deltaTheta360);
                } else {
                    // Turn right by deltaTheta in the 360� scale
                    turnRightBy(deltaTheta360);
                }
            }
        }
        
        setSpeed(0);
        stopMotor();
        isTurning = false;
        
    }

    /**
    * @returns <code>true</code> if the robot is either Navigating or Turning
    */
    public boolean isNavigating(){
        return isTurning || isTraveling;
    }

    /**
     * Turn right by the number of degrees specified. This method is mutually recursive with 
     * <b><code>turnLeftBy()</code></b> to guard against turning with a maximal angle.
     * @param angle
     */
    public static void turnRightBy(double angle){
    	if(angle == -180){
    		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, angle), true);
            rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, angle), false);
            return;
    	}
        angle = convertAngleTo180Scale(angle);
        if(angle < 0) {
            turnLeftBy(-angle);
        } else {
            // Input is correct, so turn right
            leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, angle), true);
            rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, angle), false);
        }
    }

    /**
     * Turn left by the number of degrees specified. This method is mutually recursive with 
     * <b><code>turnRightBy()</code></b> to guard against turning with a maximal angle.
     * @param angle
     */
    public static void turnLeftBy(double angle){
        angle = convertAngleTo180Scale(angle);
        if(angle < 0) {
            turnRightBy(-angle);
        } else {
            // Input is correct, so turn left
            leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, angle), true);
            rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, angle), false);
        }
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

    /**
     * Set the speed for the left motor.
     * @param speed The speed of the motor in degrees per second
     */
    public static void setLeftSpeed(float speed) {
        leftMotor.setSpeed(speed);
    }

    /**
     * Set the speed for the right motor.
     * @param speed The speed of the motor in degrees per second
     */
    public static void setRightSpeed(float speed) {
        rightMotor.setSpeed(speed);
    }
    
  
    
    /**
     * Set the acceleration for the left motor.
     * @param acceleration The acceleration of the motor in degrees per second 
     */
    public static void setLeftAcceleration(int acceleration) {
        leftMotor.setAcceleration(acceleration);
    }

    /**
     * Set the acceleration for the right motor.
     * @param acceleration The acceleration of the motor in degrees per second
     */
    public static void setRightAcceleration(int acceleration) {
        rightMotor.setAcceleration(acceleration);
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
     * Go backward for a specified distance
     * @param distance Desired distance in cm.
     */
    public static void backward(double distance){
        leftMotor.rotate(-convertDistance(WHEEL_RADIUS, distance), true);
        rightMotor.rotate(-convertDistance(WHEEL_RADIUS, distance), false);
    }
    /**
     * Set the speed for both the left and right motors.
     * @param speed The speed of the motor in degrees per second
     */
    public static void setSpeed(float speed) {
        // Stop both motors at once
        leftMotor.stop(true); // boolean is for immediate return
        rightMotor.stop(false);
        // Clear default speed value
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        // Set the speed level to the parameter
        leftMotor.setSpeed(speed);
        rightMotor.setSpeed(speed);
    }
    
    /**
     * Set the acceleration for both the left and right motors.
     * @param acceleration The acceleration of the motor in degrees per second squared
     */
    public static void setAcceleration(int acceleration) {
     // Stop both motors at once
        leftMotor.stop(true); // boolean is for immediate return
        rightMotor.stop(false);
        // Clear default acceleration value
        leftMotor.setAcceleration(0);
        rightMotor.setAcceleration(0);
        // Set the acceleration level to the parameter
        leftMotor.setAcceleration(acceleration);
        rightMotor.setAcceleration(acceleration);
    }
    
    /**
	 * Make robot stop
	 */
	private void stopMotor() {
		//leftMotor.startSynchronization();
		leftMotor.stop();
		rightMotor.stop();
		//leftMotor.endSynchronization();
		//leftMotor.waitComplete();
		//rightMotor.waitComplete();
	}

    /**
     * Go forward for a specified distance
     * @param distance Desired distance in cm.
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
     * Takes an angle in degrees
     * @return The converted angle in the scale [-180,180).
     */
    public static double convertAngleTo180Scale(double angle) {
        // Wraparound 360�
        angle %= 360;
        if(angle>=180) {
            angle -= 360;
        }
        if(angle<-180) {
            angle +=360;
        }
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