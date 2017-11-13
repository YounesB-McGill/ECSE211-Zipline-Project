package ca.mcgill.ecse211.ziplineproject;




import ca.mcgill.ecse211.ziplineproject.UltrasonicLocalizer.LocalizationType;
import lejos.hardware.Button;
import lejos.hardware.Device;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.UARTSensor;
import lejos.robotics.SampleProvider;

/**
 * Main class of the ECSE211 Zipline Design Project. All constants and hardware devices are initialized here. 
 * This class also contains the core game logic. <br>
 * 
 * <p>The goal of the project is to create an autonomous robot capable of playing the game Capture the Flag.
 * The robot is transmitted information about the playing field at the beginning of the game, including its team color,
 * starting location, zipline location, and the color of the target flag it needs to capture. 
 * </p>
 * <p>At the start of the game, the robot is placed in the appropriate start corner according to whether it is on the 
 * RED or GREEN team. The robot then does ultrasonic localization to orient itself to a suitable angle. After that, 
 * light localization is performed in order to reliably determine its <i>x</i> and <i>y</i> coordinates.
 * </p>
 * <p>The robot then navigates to the opponent's side of the field, via the zipline or the shallow crossing, depending 
 * on its team color, avoiding any obstacles on the way. The robot searches for the opponent flag in the indicated 
 * search zone and captures it by beeping three times. The robot returns to its own start corner using the other method 
 * of travel.
 * </p>
 * <p>This entire exercise must be done in no more than 5 minutes. This is an image of the competition supplied by
 * the client:
 * </p>
 * 
 * <pre><img src="{@docRoot}/doc-files/competition.png" /></pre>
 * 
 * @author Younes Boubekeur 
 */
public class Main {

    /**The radius of the robot tire, 2.13 cm.*/public static final double WHEEL_RADIUS = 2.13;
    /**The width of the robot, as measured between the left and right wheels, 14.80 cm.*/ 
    public static final double TRACK = 14.68; // was 14.80, then 15.0, then 15.2. 15.0 was better
    /**The length of one competition floor tile, 30.48 cm.*/public static final double TILE = 30.48;
    
    /**The speed used by the robot to travel forward.*/public static final int FWD_SPEED = 180;
    /**The speed used by the robot to rotate.*/public static final int ROTATE_SPEED = 120;
    /**The speed used by the robot to traverse the zipline.*/public static final int TRAVERSE_SPEED = 300;
    
    /**The <i>x</i> coordinate of the zipline approach.*/public static int x0 = 0;
    /**The <i>y</i> coordinate of the zipline approach.*/public static int y0 = 0;
    /**The <i>x</i> coordinate of the zipline.*/public static int xc = 0;
    /**The <i>y</i> coordinate of the zipline.*/public static int yc = 0;
    /**The <i>x</i> coordinate of the opponent flag.*/public static int xd = 0;
    /**The <i>y</i> coordinate of the opponent flag.*/public static int yd = 0;
    
    /**The start corner for the competition.*/
    public static int startCorner;
    /**<code>int</code> corresponding to the EV3 button being pressed.*/
    public static int buttonChoice;
    
    /**
     * The Motors used in the application, namely 
     * <b><i><code>leftMotor</code></i></b>,
     * <b><i><code>rightMotor</code></i></b>, and 
     * <b><i><code>traverseMotor</code></i></b>.
     */
    public static EV3LargeRegulatedMotor[] motors = new EV3LargeRegulatedMotor[] {
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")),
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D")),
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"))
    };

    /**EV3 left motor.*/
    public static final EV3LargeRegulatedMotor leftMotor = motors[0];
    /**EV3 right motor.*/
    public static final EV3LargeRegulatedMotor rightMotor = motors[1];
    /**EV3 traverse motor, which is used for traversing the zipline.*/
    public static final EV3LargeRegulatedMotor traverseMotor = motors[2];
    
    /**
     * The Sensors used in the application, namely the two
     * <b><i><code>EV3ColorSensor</code></i></b>s and the
     * <b><i><code>EV3UltrasonicSensor</code></i></b>
     */
    public static UARTSensor[] sensors = new UARTSensor[] {
            //new EV3UltrasonicSensor(LocalEV3.get().getPort("S1")),
            new EV3ColorSensor(LocalEV3.get().getPort("S4")),
            // TODO Second ColorSensor for flag capture
            // new EV3ColorSensor(LocalEV3.get().getPort("S?"))
    };
    
    public static final Port usPort = LocalEV3.get().getPort("S1");
    /**Ultrasonic sensor used for localization and obstacle avoidance*/
    //public static final EV3UltrasonicSensor usSensor = (EV3UltrasonicSensor) sensors[0]; 
    /**Color sensor used to detect gridlines*/
    public static final EV3ColorSensor cSensor = (EV3ColorSensor) sensors[0];
    /**Color sensor used to recognize the flag*/
    public static final EV3ColorSensor flagSensor = null; //TODO (EV3ColorSensor) sensors[2];
    
    /**EV3 Hardware LCD display*/
    public static TextLCD textLCD = LocalEV3.get().getTextLCD();
    
    /**Odometer used to know the position and orientation of the robot*/
    public static Odometer odometer = new Odometer();
    /**Displays the user interface and odometry information on the EV3 LCD*/
    public static Display display = new Display();
    /**Navigation object used to navigate across the game environment*/
    public static Navigation navigation = new Navigation(x0,y0,2);
    
    /**
     * The possible team colors of the robot, either Red or Green.
     */
    public static enum TeamColor { RED, GREEN };
    
    /**
     * The current team color of the robot, based on the information sent by the Wi-Fi server
     */
    public static TeamColor teamColor;
    
    /*************************************************************************************************/
    
    /**
     * Main entry point for the game logic
     * @param {String[]} args
     */
    @SuppressWarnings("static-access")
    public static void main(String[] args) {
        //Sound.beep();
        Sound.beepSequenceUp();
        setTeamColor();
        //startCorner = Display.getStartCornerUI();
        
        //TestOdometer.testOdometer();
        //TestNavigation.testNavigation();
        startCorner=1;
        x0=1;
        y0=6;
        xc=2;
        yc=6;
        xd=7;
        yd=5;
        //TestLightLocalizer.testLightLocalizer();
        //TestTraverseZipline.testTraverseZipline();
        
        int button = 0;
        textLCD.drawString("Press a button ", 0, 0);
        textLCD.drawString("to start the ", 0, 1);
        textLCD.drawString("navigation test", 0, 2);
        
        while (button == 0)
            button = Button.waitForAnyPress();
        
        // Clear display
        textLCD.clear();
        
        SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
        SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
        float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned
        UltrasonicPoller usPoller = null;
        
        // Start odometer and display
        odometer.start();
        display.start();
        
       
        
        UltrasonicLocalizer ul= new UltrasonicLocalizer(0); 
        usPoller = new UltrasonicPoller(usDistance, usData, ul);
        usPoller.start(); 

        ul.start();
        dispose(ul);
        
        LightLocalizer lo=new LightLocalizer(0);
        lo.start();
        dispose(lo);
        
        Navigation na=new Navigation(x0,y0,0);
        na.start();
        dispose(na);
        
        LightLocalizer lo1=new LightLocalizer(1);
        lo1.start();
        dispose(lo1);
        
        TraverseZipline tz= new TraverseZipline(xc,yc);
        tz.start();
        dispose(tz);
        
        Navigation na1=new Navigation(xd,yd,2);
        na1.start();
        dispose(na1);
        
        while (Button.waitForAnyPress() != Button.ID_ESCAPE)
            ;
        System.exit(0);
    } // end main method
    
    /**
     * @return The current team color of the robot
     */
    public static TeamColor getTeamColor() {
        // TODO After Beta demo, add team color logic here
        return TeamColor.GREEN;
    }
    
    /**
     * Sets the team color of the robot based on the information obtained from Wi-Fi
     */
    public static void setTeamColor(/* TODO Add Wi-Fi parameters here*/) {
        // TODO After Beta demo, add team color logic here
        teamColor = TeamColor.GREEN;
    }
    public static void dispose(Thread thread) {
        try {
            thread.join(); // wait till thread dies
        } catch (InterruptedException e) {}
    }
    
    /*************************************************************************************************/

    
    // These methods are for internal testing and are therefore excluded from the API
    /*public static void dispose(Thread thread) {
        try { thread.join(); // wait till thread dies
        } catch (InterruptedException e) {}
    }
    
    public static void waitForever() {
        try { Thread.sleep(60*60*1000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    
    public static void endProgram() {
        try { throw new Exception();
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }*/
    
    
} // end Main class

