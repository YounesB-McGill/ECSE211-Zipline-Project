package ca.mcgill.ecse211.ziplineproject;

import ca.mcgill.ecse211.ziplineproject.UltrasonicLocalizer.LocalizationType;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

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
 * <pre><img src="{@docRoot}/doc-files/competition.png"></img></pre>
 * 
 * @author Younes Boubekeur 
 */
@SuppressWarnings("rawtypes") // From supplied Wi-Fi code
public class Main {
    
    /**The IP address of the computer running the server application*/
    private static final String SERVER_IP = "192.168.2.31"; // TA or Prof: 192.168.2.3 // I'm 16
    /**Contestants' Team number*/
    private static final int TEAM_NUMBER = 10;
    /** Enable/disable printing of debug info from the WiFi class*/
    private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
    
    /** Enable/disable experimental Navigation corrcetion*/
    public static boolean navigationCorrection = false;

    /**The radius of the robot tire, 2.13 cm.*/public static final double WHEEL_RADIUS = 2.115; // was 2.13
    /**The width of the robot, as measured between the left and right wheels, 14.80 cm.*/ 
    public static final double TRACK = 14.68; // was 14.68, 14.80, 15.0, 15.2.
    /**The length of one competition floor tile, 30.48 cm.*/public static final double TILE = 30.48;
    /**The dimension of the playing field, in Tile lengths. 8 in beta demo and 12 in final competition*/
    public static final double BOARD_SIZE = 8;
    
    /**The speed used by the robot to travel forward.*/public static final int FWD_SPEED = 300;
    /**The acceleration used by the robot to travel forward.*/public static final int FWD_ACC = 175;
    /**The speed used by the robot to rotate.*/public static final int ROTATE_SPEED = 200;
    /**The speed used by the robot to traverse the zipline.*/public static final int TRAVERSE_SPEED = 375;
    
    // Wi-Fi parameters
    /**Red team number*/public static int redTeam;
    /**Red team's starting corner*/public static int redCorner;
    /**Green team number*/public static int greenTeam;
    /**Green team's starting corner*/public static int greenCorner;
    /**Color of green opponent flag*/public static int og;
    /**Color of red opponent flag*/public static int or;
    /**The <i>x</i> coordinate of the lower left hand corner of Red Zone*/public static int red_ll_x;
    /**The <i>y</i> coordinate of the lower left hand corner of Red Zone*/public static int red_ll_y;
    /**The <i>x</i> coordinate of the upper right hand corner of Red Zone*/public static int red_ur_x;
    /**The <i>y</i> coordinate of the upper right hand corner of Red Zone*/public static int red_ur_y;
    /**The <i>x</i> coordinate of the lower left hand corner of Red Zone*/public static int green_ll_x;
    /**The <i>y</i> coordinate of the lower left hand corner of Red Zone*/public static int green_ll_y;
    /**The <i>x</i> coordinate of the upper right hand corner of Red Zone*/public static int green_ur_x;
    /**The <i>y</i> coordinate of the upper right hand corner of Red Zone*/public static int green_ur_y;
    /**The <i>x</i> coordinate of the Red Zone zip line endpoint*/public static int zc_r_x;
    /**The <i>y</i> coordinate of the Red Zone zip line endpoint*/public static int zc_r_y;
    /**The <i>x</i> coordinate of the Red Zone zip line approach*/public static int zo_r_x; // Was xd
    /**The <i>y</i> coordinate of the Red Zone zip line approach*/public static int zo_r_y; // Was yd
    /**The <i>x</i> coordinate of the Green Zone zip line endpoint*/public static int zc_g_x; // Was xc
    /**The <i>y</i> coordinate of the Green Zone zip line endpoint*/public static int zc_g_y; // Was yc
    /**The <i>x</i> coordinate of the Green Zone zip line approach*/public static int zo_g_x; // Was x0
    /**The <i>y</i> coordinate of the Green Zone zip line approach*/public static int zo_g_y; // Was y0
    /**The <i>x</i> coordinate of the lower left hand corner of the horizontal shallow water zone*/
    public static int sh_ll_x;
    /**The <i>y</i> coordinate of the lower left hand corner of the horizontal shallow water zone*/
    public static int sh_ll_y;
    /**The <i>x</i> coordinate of the upper right hand corner of the horizontal shallow water zone*/
    public static int sh_ur_x;
    /**The <i>y</i> coordinate of the upper right hand corner of the horizontal shallow water zone*/
    public static int sh_ur_y;
    /**The <i>x</i> coordinate of the lower left hand corner of the vertical shallow water zone*/
    public static int sv_ll_x;
    /**The <i>y</i> coordinate of the lower left hand corner of the vertical shallow water zone*/
    public static int sv_ll_y;
    /**The <i>x</i> coordinate of the upper right hand corner of the vertical shallow water zone*/
    public static int sv_ur_x;
    /**The <i>y</i> coordinate of the upper right hand corner of the vertical shallow water zone*/
    public static int sv_ur_y;
    /**The <i>x</i> coordinate of the lower left hand corner of search region in Red Zone*/public static int sr_ll_x;
    /**The <i>y</i> coordinate of the lower left hand corner of search region in Red Zone*/public static int sr_ll_y;
    /**The <i>x</i> coordinate of the upper right hand corner of search region in Red Zone*/public static int sr_ur_x;
    /**The <i>y</i> coordinate of the upper right hand corner of search region in Red Zone*/public static int sr_ur_y;
    /**The <i>x</i> coordinate of the lower left hand corner of search region in Green Zone*/public static int sg_ll_x;
    /**The <i>y</i> coordinate of the lower left hand corner of search region in Green Zone*/public static int sg_ll_y;
    /**The <i>x</i> coordinate of the upper right hand corner of search region in Green Zone*/public static int sg_ur_x;
    /**The <i>y</i> coordinate of the upper right hand corner of search region in Green Zone*/public static int sg_ur_y;

    /**<code>int</code> corresponding to the EV3 button being pressed.*/
    public static int buttonChoice;
    /**Controls whether to print odometry and battery information to Console (and on the robot screen)*/
    public static boolean printToConsole = true;
    
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
            new EV3UltrasonicSensor(LocalEV3.get().getPort("S1")),
            /*null,*/new EV3ColorSensor(LocalEV3.get().getPort("S4")),
            // Second ColorSensor for flag capture
            //new EV3ColorSensor(LocalEV3.get().getPort("S4")) // TODO Change S4 to correct port
    };
    
    /**Ultrasonic sensor used for localization and obstacle avoidance*/
    public static final EV3UltrasonicSensor usSensor = (EV3UltrasonicSensor) sensors[0]; 
    /**Color sensor used to detect gridlines*/
    public static final EV3ColorSensor cSensor = (EV3ColorSensor) sensors[1];
    /**Color sensor used to recognize the flag*/
    public static final EV3ColorSensor flagSensor = null;// (EV3ColorSensor) sensors[2];
    
    /**EV3 Hardware LCD display*/
    public static TextLCD textLCD = LocalEV3.get().getTextLCD();
    
    /**Odometer used to know the position and orientation of the robot*/
    public static Odometer odometer = new Odometer();
    /**Displays the user interface and odometry information on the EV3 LCD*/
    public static Display display = new Display();
    /**Navigation object used to navigate across the game environment*/
    public static Navigation navigation = new Navigation();
    
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
     * @param args
     */
    @SuppressWarnings("static-access")
    public static void main(String[] args) {
        // Indicate program has loaded
        Sound.beepSequenceUp();
        // Get Wi-Fi parameters from the server
        getWiFiParameters();
        
        // Start odometer and display runnables here
        odometer.start();
        display.start();
        
        setTeamColor();
        
        System.out.println("Green team:" + greenTeam);
        //startCorner = Display.getStartCornerUI();
        
        //TestOdometer.testOdometer();
        //TestNavigation.testNavigation();
        //TestUltrasonicLocalizer.testUltrasonicLocalizer();
        //TestLightLocalizer.testLightLocalizer();
        //TestTraverseZipline.testTraverseZipline();
        //TestFlagCapture.testFlagCapture();
        IntegrationTest.integrationTest();
        
        
        // To confirm control returns to main
        Sound.beepSequenceUp();
        
        while (Button.waitForAnyPress() != Button.ID_ESCAPE)
            ; // do nothing
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
    public static void setTeamColor() {
    	if(greenTeam == TEAM_NUMBER)
    		teamColor = TeamColor.GREEN;
    	else // RED
    		teamColor = TeamColor.RED;
    }
    
    /**
     * Get Wi-Fi parameters from the server
     */
    public static void getWiFiParameters() {
        // Initialize WifiConnection class
        WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
        try {
            Map data = conn.getData();
            // Initialize the global variables 
            redTeam = ((Long) data.get("RedTeam")).intValue();
            redCorner = ((Long) data.get("RedCorner")).intValue();
            greenTeam = ((Long) data.get("GreenTeam")).intValue();
            greenCorner = ((Long) data.get("GreenCorner")).intValue();
            og = ((Long) data.get("OG")).intValue();
            or = ((Long) data.get("OR")).intValue();
            red_ll_x = ((Long) data.get("Red_LL_x")).intValue();
            red_ll_y = ((Long) data.get("Red_LL_y")).intValue();
            red_ur_x = ((Long) data.get("Red_UR_x")).intValue();
            red_ur_y = ((Long) data.get("Red_UR_y")).intValue();
            green_ll_x = ((Long) data.get("Green_LL_x")).intValue();
            green_ll_y = ((Long) data.get("Green_LL_y")).intValue();
            green_ur_x = ((Long) data.get("Green_UR_x")).intValue();
            green_ur_y = ((Long) data.get("Green_UR_y")).intValue();
            zc_r_x = ((Long) data.get("ZC_R_x")).intValue();
            zc_r_y = ((Long) data.get("ZC_R_y")).intValue();
            zo_r_x = ((Long) data.get("ZO_R_x")).intValue();
            zo_r_y = ((Long) data.get("ZO_R_y")).intValue();
            zc_g_x = ((Long) data.get("ZC_G_x")).intValue();
            zc_g_y = ((Long) data.get("ZC_G_y")).intValue();
            zo_g_x = ((Long) data.get("ZO_G_x")).intValue();
            zo_g_y = ((Long) data.get("ZO_G_y")).intValue();
            sh_ll_x = ((Long) data.get("SH_LL_x")).intValue();
            sh_ll_y = ((Long) data.get("SH_LL_y")).intValue();
            sh_ur_x = ((Long) data.get("SH_UR_x")).intValue();
            sh_ur_y = ((Long) data.get("SH_UR_y")).intValue();
            sv_ll_x = ((Long) data.get("SV_LL_x")).intValue();
            sv_ll_y = ((Long) data.get("SV_LL_y")).intValue();
            sv_ur_x = ((Long) data.get("SV_UR_x")).intValue();
            sv_ur_y = ((Long) data.get("SV_UR_y")).intValue();
            sr_ll_x = ((Long) data.get("SR_LL_x")).intValue();
            sr_ll_y = ((Long) data.get("SR_LL_y")).intValue();
            sr_ur_x = ((Long) data.get("SR_UR_x")).intValue();
            sr_ur_y = ((Long) data.get("SR_UR_y")).intValue();
            sg_ll_x = ((Long) data.get("SG_LL_x")).intValue();
            sg_ll_y = ((Long) data.get("SG_LL_y")).intValue();
            sg_ur_x = ((Long) data.get("SG_UR_x")).intValue();
            sg_ur_y = ((Long) data.get("SG_UR_y")).intValue();

        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
        }
        
        
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

