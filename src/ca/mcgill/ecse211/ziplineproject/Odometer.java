package ca.mcgill.ecse211.ziplineproject;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The <code>Odometer</code> class keeps track of the robot's position and orientation 
 * at all times by counting wheel rotations. It runs in a separate Thread. 
 * 
 * @author Chaoyi Liu
 *
 */
public class Odometer implements Runnable {
    // robot position
    /**Current <i>x</i> coordinate of the robot*/private double x;
    /**Current <i>y</i> coordinate of the robot*/private double y;
    /**Current <i>theta</i> of the robot, in radians, using a scale from 0 to 2 pi*/private double theta;
    /**Theoretical <i>x</i> coordinate of the robot*/private double xTh;
    /**Theoretical <i>y</i> coordinate of the robot*/private double yTh;
    /**Theoretical <i>theta</i> of the robot, in radians, using a scale from 0 to 2 pi*/private double thetaTh;
    /**Array of last 10 thetas, used to determine the robot's turning state*/private double[] last10thetas;
    /*private double deltaL;
    private double deltaR;
    private double deltaTheta;
    private double deltaD;*/
    /**Tachometer reading of the left motor*/private int leftMotorTachoCount;
    /**Tachometer reading of the right motor*/private int rightMotorTachoCount;
    /**State of the robot*/private int state;
    /**<b><code>true</code></b> if the robot is turning*/private boolean isTurning;
    /**Local version of <b><code>Main</code></b> global constant*/
    private EV3LargeRegulatedMotor leftMotor = Main.leftMotor;
    /**Local version of <b><code>Main</code></b> global constant*/
    private EV3LargeRegulatedMotor rightMotor = Main.rightMotor;

    /**odometer update period, in ms */private static final long ODOMETER_PERIOD = 35; 
    /**Local version of <b><code>Main</code></b> global constant*/
    private static double WHEEL_RADIUS = Main.WHEEL_RADIUS;
    /**Constant to reduce error in theta*/
    private static final double THETA_OFFSET = 362d/365; // reduce error in theta. d is for double
    /**Threshold that robot must turn before entering Turning state, in degrees*/
    private static final double TURNING_THRESHOLD = 5;
    /**Local version of <b><code>Main</code></b> global constant*/
    private static double TRACK = Main.TRACK;
    /**Local version of <b><code>Main</code></b> global constant*/
    private static final double TILE = Main.TILE;
    
    /**Tacho L at last sample*/public static int lastTachoL;
    /**Tacho R at last sample*/public static int lastTachoR;
    /**Current tacho L*/public static int nowTachoL;
    /**Current tacho R*/public static int nowTachoR;


    /** lock object for mutual exclusion */private Object lock; 
    /**Used to run the Odometer as a <b><code>Thread</code></b>*/public Thread runner; 

    /**Default constructor*/
    public Odometer() {
        this.x = 0.0;
        this.y = 0.0;
        this.theta = 0.0;
        this.last10thetas = new double[]{0,0,0,0,0,0,0,0,0,0};
        this.leftMotorTachoCount = 0;
        this.rightMotorTachoCount = 0;
        this.state = 0;
        this.isTurning = false;
        lock = new Object();
    }
    
    /** Run method required for thread */
    public void run() {
      long updateStart, updateEnd;

      while (true) {
        updateStart = System.currentTimeMillis();
        // TODO put (some of) your odometer code here
        
        //get the tachometer counts
        int nowTachoL = leftMotor.getTachoCount();
        int nowTachoR = rightMotor.getTachoCount();
        
        //calculate small distances travelled by each wheel
        double distL = Math.PI*WHEEL_RADIUS*(nowTachoL - getLeftMotorTachoCount())/180;
        double distR = Math.PI*WHEEL_RADIUS*(nowTachoR - getRightMotorTachoCount())/180;
        setLeftMotorTachoCount(nowTachoL);
        setRightMotorTachoCount(nowTachoR);
        
        //deltaD is the average of the small distance
        double deltaD = 0.5*(distL+distR);
        //deltaT is the small angle
        double deltaT = (distL-distR)/TRACK;
        
        synchronized (lock) {
          /**
           * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
           * and theta in this block. Do not perform complex math
           * 
           */
   
          // ensuring the value of theta is between 0 and 359 degrees
          theta = getTheta() + deltaT;
          //if theta is greater than or equal to 360, subtract 360
          if (theta >= 2*Math.PI) {
              this.setTheta(theta - 2*Math.PI);
          } 
          //if theta is less than 0, add 360
          else if (theta < 0) {
              this.setTheta(theta + 2*Math.PI);
          } 
          //else it's in the bounds so don't add or subtract anything
          else {
              this.setTheta(theta);
          }
          
          //calculates change in dx and dy based on the angle
          double dx = deltaD*Math.sin(theta);
          double dy = deltaD*Math.cos(theta);
          //set x and y values
          setX(getX() + dx);
          setY(getY() + dy);
         
        }

        // this ensures that the odometer only runs once every period
        updateEnd = System.currentTimeMillis();
        if (updateEnd - updateStart < ODOMETER_PERIOD) {
          try {
            Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
          } catch (InterruptedException e) {
            // there is nothing to be done here because it is not
            // expected that the odometer will be interrupted by
            // another thread
          }
        }
      }
    } // end run()

    // This method determines the state of the odometer, based on theta and turning status
    public void determineState() {
        int result = 0;
        // determine theta orientation
        double thetaD = getThetaInDegrees();
        
        // Decide if we're turning or going straight
        double[] last10thetas = filterThetas(getLast10Thetas());
        // thetaDiff is the difference of the averages of the first 2 and last 2 thetas
        double thetaDiff = (Math.abs(last10thetas[0]-last10thetas[1])/2d)
                -((Math.abs(last10thetas[8]-last10thetas[9])/2d));
        if (Math.abs(thetaDiff) >= TURNING_THRESHOLD) {
            Sound.playNote(Sound.PIANO, 500, 250);
            setIsTurning(true);
        } else {
            setIsTurning(false);
        }
        
        if(!getIsTurning()){ // robot is going mostly straight
            if(thetaD > 360-22.5 || thetaD < 22.5) result = 1; // North; 45/2=22.5
            else if(thetaD > 22.5 && thetaD < 45+22.5) result = 2; // Northeast
            else if(thetaD > 45+22.5 && thetaD < 90+22.5) result = 3; // East
            else if(thetaD > 90+22.5 && thetaD < 135+22.5) result = 4; // Southeast
            else if(thetaD > 135+22.5 && thetaD < 180+22.5) result = 5; // South
            else if(thetaD > 180+22.5 && thetaD < 225+22.5) result = 6; // Southwest
            else if(thetaD > 225+22.5 && thetaD < 270+22.5) result = 7; // West
            else if(thetaD > 270+22.5 && thetaD < 315+22.5) result = 8; // Northwest
            else ; // Something is wrong!
        } else {
            result = 9; // robot is turning
        }
        
        setRobotState(result);
    }   

    /**
     * Get the current robot position in cm and radians
     * @param position The current position of the robot
     * @param update Boolean array that controls which variables to update
     */
    public void getPosition(double[] position, boolean[] update) {
        // ensure that the values don't change while the odometer is running
        synchronized (lock) {
            if (update[0]) position[0] = x;
            if (update[1]) position[1] = y;
            if (update[2]) position[2] = theta;
        }
    }
    
    /**
     * Get the current robot position in units suitable for displaying and debugging.
     * <i>x</i> and <i>y</i> are in units of Tile length.
     * Theta is in degrees.
     * @param position The current position of the robot
     * @param update Boolean array that controls which variables to update
     */
    public void getPositionForDisplay(double[] position, boolean[] update) {
        // ensure that the values don't change while the odometer is running
        //getPosition(position, update);
        synchronized (lock) {
            if (update[0]) position[0] = x/TILE;
            if (update[1]) position[1] = y/TILE;
            if (update[2]) position[2] = getThetaInDegrees();
        }
    }

    /**
     * @return The current <i>x</i> coordinate of the robot
     */
    public double getX() {
        double result;
        synchronized (lock) { result = x; }
        return result;
    }

    /**
     * @return The current <i>y</i> coordinate of the robot
     */
    public double getY() {
        double result;
        synchronized (lock) { result = y; }
        return result;
    }

    /**
     * @return The current theta of the robot, in radians
     */
    public double getTheta() {
        double result;
        synchronized (lock) { result = theta; }
        return result;
    }
    
    /**
     * @return The theoretical <i>x</i> coordinate of the robot
     */
    public double getXTh() {
        double result;
        synchronized (lock) { result = xTh; }
        return result;
    }

    /**
     * @return The theoretical <i>y</i> coordinate of the robot
     */
    public double getYTh() {
        double result;
        synchronized (lock) { result = yTh; }
        return result;
    }

    /**
     * @return The theoretical theta of the robot, in radians
     */
    public double getThetaTh() {
        double result;
        synchronized (lock) { result = thetaTh; }
        return result;
    }
    
    /**
     * @return <b><code>double</code></b> array of last 10 thetas, in degrees
     */
    public double[] getLast10Thetas() {
        double[] result;
        synchronized (lock) { result = last10thetas; }
        return result;
    }    
    
    /**
     * @return The current theta of the robot, in degrees using the [0-360<sup>o</sup>) scale
     */
    public double getThetaInDegrees() {
        double result;
        synchronized (lock) {
            result = THETA_OFFSET*theta*180/Math.PI;
        }
        return result;
    }
    
    /**
     * @return The theoretical theta of the robot, in degrees using the [0-360<sup>o</sup>) scale
     */
    public double getThetaThInDegrees() {
        double result;
        synchronized (lock) {
            result = thetaTh*180/Math.PI;
        }
        return result;
    }
    
    /**
     * @return <b><code>true</code></b> if the robot is turning
     */
    public boolean getIsTurning() { 
        boolean result;
        synchronized (lock) { result = isTurning; }
        return result;
    }
    
    public int getRobotState() { // can't use name getState()
        int result;
        synchronized (lock) { result = state; }
        return result;
    }

    // mutators
    public void setPosition(double[] position, boolean[] update) {
        // ensure that the values don't change while the odometer is running
        synchronized (lock) {
            if (update[0]) x = position[0];
            if (update[1]) y = position[1];
            if (update[2]) theta = position[2];
        }
    }
    
    public void setPositionTh(double[] position, boolean[] update) {
        // ensure that the values don't change while the odometer is running
        synchronized (lock) {
            if (update[0]) xTh = position[0];
            if (update[1]) yTh = position[1];
            if (update[2]) thetaTh = position[2];
        }
    }

    public void setX(double x) {
        synchronized (lock) { this.x = x; }
    }

    public void setY(double y) {
        synchronized (lock) { this.y = y; }
    }

    public void setTheta(double theta) {
        synchronized (lock) { this.theta = theta; }
    }
    
    public void setXTh(double xTh) {
        synchronized (lock) { this.xTh = xTh; }
    }

    public void setYTh(double yTh) {
        synchronized (lock) { this.yTh = yTh; }
    }

    public void setThetaTh(double thetaTh) {
        synchronized (lock) { this.thetaTh = thetaTh; }
    }
    
    public void updateThetaArray() {
        synchronized (lock) { 
            double[] result = getLast10Thetas(); // previous version of theta array
            double currentTheta = getThetaInDegrees();
            // Shift all thetas by 1
            for(int i = 0; i < 9; i++){
                result[i] = result[i+1];
            }
            result[9] = currentTheta;
            last10thetas = result;
            //System.out.println("Updated theta array with currentTheta="+currentTheta);
        }
    }
    
    public void setRobotState(int state) {
        synchronized (lock) { this.state = state; }
    }
    
    public void setIsTurning(boolean isTurning) {
        synchronized (lock) { this.isTurning = isTurning; }
    }

    public int getLeftMotorTachoCount() {
        return leftMotorTachoCount;
    }

    public void setLeftMotorTachoCount(int leftMotorTachoCount) {
        synchronized (lock) {
            this.leftMotorTachoCount = leftMotorTachoCount;
        }
    }

    public int getRightMotorTachoCount() {
        return rightMotorTachoCount;
    }

    public void setRightMotorTachoCount(int rightMotorTachoCount) {
        synchronized (lock) {
            this.rightMotorTachoCount = rightMotorTachoCount;
        }
    }
    
    public double[] filterThetas(double[] thetas){
        // TODO Implement filtering
        return thetas;
    }
    
    /**
     * Starts the Odometer Thread
     */
    public void start() {
        runner = new Thread(this);
        runner.start();
    }

}