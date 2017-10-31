package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** Calculates and tracks position based on wheel rotations*/
public class Odometer extends Thread {
	
  private final double WHEEL_RADIUS;    
  private final double  TRACK;
  double[] position = new double[3];
  private double angle = 0;
  
  // robot position
  private double x;
  private double y;
  private double theta;
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/

  private Object lock; /*lock object for mutual exclusion*/

  /* 
   * Default Constructor
   * @param leftMotor
   * @param rightMotor
   * @param WHEEL_RADIUS
   * @param TRACK
   *
   */
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double WHEEL_RADIUS, double TRACK) {
    this.WHEEL_RADIUS = WHEEL_RADIUS;
    this.TRACK = TRACK;
	this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
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
    	angle = getTheta() + deltaT;
    	//if theta is greater than or equal to 360, subtract 360
    	if (angle >= 2*Math.PI) {
    		this.setTheta(angle - 2*Math.PI);
    	} 
    	//if theta is less than 0, add 360
    	else if (angle < 0) {
    		this.setTheta(angle + 2*Math.PI);
    	} 
    	//else it's in the bounds so don't add or subtract anything
    	else {
    		this.setTheta(angle);
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
  }

  /** Returns the current position based on the odometer */ 	
  public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = theta * (180/Math.PI);
    }
  }

  /** Returns the current X position */
  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  /** Returns the current Y position */
  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  /* Returns the current Theta orientation */ 
  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  /* Method to set the current position in the odometer */ 
  public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  /** Method to set the current X position */
  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  /** Method to set the current  position */
  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  /** Method to set the current  position */
  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /**
   * @return the leftMotorTachoCount
   */
  public int getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * @param leftMotorTachoCount the leftMotorTachoCount to set
   */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /**
   * @return the rightMotorTachoCount
   */
  public int getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

  /**
   * @param rightMotorTachoCount the rightMotorTachoCount to set
   */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}
