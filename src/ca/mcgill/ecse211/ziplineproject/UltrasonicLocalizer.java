package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This class contains the ultrasonic localization logic.
 * 
 * @author Paarth Kalia
 *
 */
public class UltrasonicLocalizer {
    private static final int WALL_DIST = 45;
    private static final int DISTANCE_CAP = 60;
    private static final int LARGE_ANGLE = 225-180;
    private static final int SMALL_ANGLE = 45+180;
    private static final int ORIGIN = 80;
    public static final double ROTATION_SPEED = 125;
    
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
    
    private static boolean printToConsole = Main.printToConsole;
    
    public UltrasonicLocalizer(LocalizationType type){
        localType = type;
    }
    /**
     * Do the ultrasonic localization
     */
    @SuppressWarnings("static-access") // Need to access global variable
    public static void doLocalization(){
        double[] pos = new double[3];
        double angle1, angle2, deltAngle;

        //set speed to the rotation speed only once
        navigation.setSpeed((float) ROTATION_SPEED);
        //draw usData
        if(printToConsole) System.out.println(getData());
        
        if(localType==LocalizationType.FALLING_EDGE){ //this means we have to rotate until wall is seen
            while(getData()<DISTANCE_CAP){
                //draw usData
                if(printToConsole) System.out.println(getData());
                // turn clockwise
                clockwise();
            }
            //stop the robot
            stopMotors();
            
            try{Thread.sleep(500);} catch(InterruptedException e){} //rotate until wall is seen, note the angle
            
            while(getData()==DISTANCE_CAP){
                //draw usData
                if(printToConsole) System.out.println(getData());
                //turn clockwise
                clockwise();
            }
            stopMotors();
            //make sound to signal change
            Sound.playNote(Sound.PIANO, 700, 250);
            
            angle1 = odometer.getThetaInDegrees();
            //draw the angle
            if(printToConsole) System.out.println("Angle1 is: "+angle1);
            //draw usData
            if(printToConsole) System.out.println(getData());
            while(getData() < DISTANCE_CAP){
                //draw usData
                if(printToConsole) System.out.println(getData());
                // turn counterclockwise
                counterclockwise();
            }
            stopMotors();
            //rotate until wall is seen, note angle 
            while(getData() == DISTANCE_CAP){
                //draw usData
                if(printToConsole) System.out.println(getData());
                // turn counterclockwise
                counterclockwise();
            }
            stopMotors();
            //make sound to signal change
            Sound.playNote(Sound.PIANO, 700, 250);
            
            angle2 = odometer.getThetaInDegrees();
            //draw angle
            if(printToConsole) System.out.println("Angle2 is: "+angle2);
            //draw usData
            if(printToConsole) System.out.println(getData());
            
            if(Math.abs(angle1)<Math.abs(angle2)){                                          
                //since angle1 is clockwise from angle2, average of angles right of angle2 is 45 deg
                deltAngle = LARGE_ANGLE-((angle1+Math.abs(angle2))/2);
                try {Thread.sleep(500);}catch(InterruptedException e){}
                // TODO Set Odometer here and change turnTo accordingly
                navigation.turnTo(ORIGIN-180);
                odometer.setTheta(0);
            }
            else{
                deltAngle = SMALL_ANGLE+((angle1+Math.abs(angle2))/2);
                try {Thread.sleep(500);}
                catch(InterruptedException e){}
                navigation.turnTo(ORIGIN-90);
                odometer.setTheta(0);
            }
            //update position --> odometer.gettheta+deltAngle
            /*odometer.setPosition(new double[] { odometer.getX(), odometer.getY(), odometer.getTheta() + deltAngle },
                    new boolean[] { true, true, true });*/
            
            
        }
        
        else{ //RISING_EDGE
            while (getData() == DISTANCE_CAP){
                //draw usData
                if(printToConsole) System.out.println(getData());
                //set RotationSpeed
                
                // turn clockwise
                clockwise();
            }
            //keep rotating until the robot sees no wall, then latch the angle
            while (getData() < DISTANCE_CAP){
                //draw usData
                if(printToConsole) System.out.println(getData());
                // turn clockwise
                clockwise();
            }
            try {Thread.sleep(500);}catch (InterruptedException e) {}
            stopMotors();
            //make sound to signal change
            Sound.playNote(Sound.PIANO, 700, 250);
            
            angle1=odometer.getThetaInDegrees();
            //draw angle
            if(printToConsole) System.out.println("Angle1 is: "+angle1);
            //draw usData
            if(printToConsole) System.out.println(getData());
            
            // switch direction and wait until it sees a wall
            while (getData() == DISTANCE_CAP){
                //draw usData
                if(printToConsole) System.out.println(getData());
                // turn counterclockwise
                counterclockwise();
            }
            
            // rotate until no wall is seen, note angle
            while (getData() < DISTANCE_CAP){
                //draw usData
                if(printToConsole) System.out.println(getData());
                // turn counterclockwise
                counterclockwise();
            }
            stopMotors();
            
            //make sound to signal change
            Sound.playNote(Sound.PIANO, 700, 250);
            
            angle2=odometer.getThetaInDegrees();
            //draw angle
            if(printToConsole) System.out.println("Angle2 is: "+angle2);
            //draw usData
            if(printToConsole) System.out.println(getData());

            if(Math.abs(angle1)<Math.abs(angle2)){
                deltAngle = LARGE_ANGLE - ((angle1+Math.abs(angle2))/2);
                try {Thread.sleep(500);} 
                catch (InterruptedException e) {}
                navigation.turnTo(ORIGIN+10);
                odometer.setTheta(0);
            } else if(Math.abs(angle1)>Math.abs(angle2)) {
                deltAngle = SMALL_ANGLE + ((angle1+Math.abs(angle2))/2);
                try {Thread.sleep(500);}catch (InterruptedException e) {}
                navigation.turnTo(-ORIGIN-90);
                odometer.setTheta(0);
            }
            //update position --> odometer.gettheta+deltAngle
            /*odometer.setPosition(new double[] { odometer.getX(), odometer.getY(), odometer.getTheta() + deltAngle },
                    new boolean[] { true, true, true });*/
            Sound.playNote(Sound.PIANO, 700, 250);

        }
    }
    
    
    /**
     * Stop the left and right Motors
     */
    public static void stopMotors() {
        leftMotor.stop();
        rightMotor.stop();
    }
    
    /**
     * Make robot rotate clockwise
     */
    public static void clockwise() {
        leftMotor.forward();
        rightMotor.backward();
    }

    /**
     * Make robot rotate counterclockwise
     */
    public static void counterclockwise() {
        leftMotor.backward();
        rightMotor.forward();
    }
    
    /**
     * 
     * @return Distance from the US sensor in cm
     */
    public static int getData(){
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
