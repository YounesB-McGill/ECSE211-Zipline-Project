package ca.mcgill.ecse211.ziplineproject;

import ca.mcgill.ecse211.ziplineproject.Main.TeamColor;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class contains the flag detection and capture logic used in the competition.
 * 
 * @author Younes Boubekeur
 */
public class FlagCapture {
    // Class variable definitions
    /**The team color of the robot*/public static TeamColor teamColor = Main.teamColor;
    /**Indicates whether flag has been captured*/public static boolean flagCaptured = false;
    
    /**<b><code>int</code></b> that represents the color of the flag to be captured, based on the following mapping:
    <table border="1">
      <tr><td>&nbsp; 1 &nbsp;</td><td>Red &nbsp;</td></tr> <tr><td>&nbsp; 2 &nbsp;</td><td>Blue &nbsp;</td></tr>
      <tr><td>&nbsp; 3 &nbsp;</td><td>Yellow &nbsp; </td></tr> <tr><td>&nbsp; 4 &nbsp;</td><td>White &nbsp;</td></tr>
    </table>*/
    public static int targetFlag; // 1 Red, 2 Blue, 3 Yellow, 4 White
    
    /**The <i>x</i> coordinate of the lower left hand corner of search region*/public static int searchZone_ll_x;
    /**The <i>y</i> coordinate of the lower left hand corner of search region*/public static int searchZone_ll_y;
    /**The <i>x</i> coordinate of the upper right hand corner of search region*/public static int searchZone_ur_x;
    /**The <i>y</i> coordinate of the upper right hand corner of search region*/public static int searchZone_ur_y;

    /**Ultrasonic sensor used to detect proximity to flag*/
    public static final EV3UltrasonicSensor usSensor = Main.usSensor;
    /**Color sensor used to recognize the flag*/public static final EV3ColorSensor flagSensor = Main.flagSensor;
    
    /**<b><code>Odometer</code></b> object used throughout the application*/
    private static Odometer odometer = Main.odometer;
    /**<b><code>Navigation</code></b> object used throughout the application*/
    private static Navigation navigation = Main.navigation;
    
    // No constructor, use static methods
    
    /**
     * Execute the flag capture logic by navigating to the search zone, detecting flags, and indicating capture
     */
    public static void doFlagCapture() { // Instead of a start method
        setTargetFlag();
        setSearchZoneCoordinates();
        navigation.travelTo(searchZone_ur_x, searchZone_ur_y); // or (searchZone_ll_x, searchZone_ll_y)
        // TODO Use navigation to go around search area, and then call detectFlag() every once in awhile
        while(!flagCaptured) {
            // TODO
        }
    }
    
    /**
     * Set the search zone coordinates based on the team color
     */
    public static void setSearchZoneCoordinates() {
        if(teamColor.equals(TeamColor.GREEN)) {
            // The search zone is in the RED zone (opponent's zone)
            searchZone_ll_x = Main.sr_ll_x;
            searchZone_ll_y = Main.sr_ll_y;
            searchZone_ur_x = Main.sr_ur_x;
            searchZone_ur_y = Main.sr_ur_y;
        } else { // TeamColor is RED
            // The search zone is in the GREEN zone
            searchZone_ll_x = Main.sg_ll_x;
            searchZone_ll_y = Main.sg_ll_y;
            searchZone_ur_x = Main.sg_ur_x;
            searchZone_ur_y = Main.sg_ur_y;
        }
    }
    
    /**
     * Set the flag targeted for capture
     */
    public static void setTargetFlag() {
        if(teamColor.equals(TeamColor.GREEN)) {
            targetFlag = Main.or; // TODO Confirm this
        } else {
            targetFlag = Main.og;
        }
    }
    
    /**
     * Check if the robot is close to a flag and detect its color in that case
     */
    public static void detectFlag() {
        int detected = 0; 
        // TODO Add flag detection logic here
        
        if(detected==targetFlag) {
            captureFlag();
            flagCaptured = true;
        }
    }
    
    /**
     * Beeps three times to indicate flag capture
     */
    public static void captureFlag() {
        // STM MR-73
        Sound.playNote(Sound.PIANO, 180*3, 450);
        Sound.playNote(Sound.PIANO, 240*3, 450);
        Sound.playNote(Sound.PIANO, 360*3, 500);
    }
}
