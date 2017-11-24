package ca.mcgill.ecse211.ziplineproject;

import ca.mcgill.ecse211.ziplineproject.Main.TeamColor;
import ca.mcgill.ecse211.ziplineproject.UltrasonicLocalizer.LocalizationType;

/**
 * Integration test
 */
public class IntegrationTest {

    private static final double TILE = Main.TILE;
    private static TeamColor teamColor = Main.teamColor;
    private static Navigation navigation = Main.navigation;
    private static LightLocalizer lightLocalizer = new LightLocalizer();
    private static Odometer odometer = Main.odometer;

    /**
     * Integration test. This test contains the entire competition logic
     */
    @SuppressWarnings("static-access") // Need to set global variable
	public static void integrationTest() {
        
        double yPoint;
        if(Main.green_ur_x <= 6) yPoint = 2;
        else yPoint = 10;
        

        if (teamColor.equals(TeamColor.GREEN)) {
            UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer(LocalizationType.FALLING_EDGE);
            ultrasonicLocalizer.doLocalization(); // prevent NullPointerExceptions

            lightLocalizer.type = 0; // First localization
            lightLocalizer.runLightLocalization();
            
            double xStart;
            if(Main.greenCorner==0) xStart = 0;
            else if(Main.greenCorner==1) xStart = 12;
            else if(Main.greenCorner==2) xStart = 12;
            else xStart = 0;
            
            double yStart;
            if(Main.greenCorner==0) yStart = 0;
            else if(Main.greenCorner==1) yStart = 0;
            else if(Main.greenCorner==2) yStart = 12;
            else yStart = 12;
            
            double distToZipline = Math.sqrt((xStart-Main.zo_g_x)*(xStart-Main.zo_g_x)
            		+(yStart-Main.zo_g_y)*(yStart-Main.zo_g_y));

            if(distToZipline < 6) { // TODO Change this
                navigation.travelTo(Main.zo_g_x, Main.zo_g_y);
            } else { // zipline too far to travel to it directly
                navigation.travelTo((xStart+Main.zo_g_x)/2d, (yStart+Main.zo_g_y)/2d);
                lightLocalizer.type = 1; // regular localization
                lightLocalizer.runLightLocalization();
                navigation.travelTo(Main.zo_g_x, Main.zo_g_y);
            }

            lightLocalizer.type = 1; // Zipline localization
            lightLocalizer.runLightLocalization();
            
            navigation.pointTo(Main.zc_g_x, Main.zc_g_y);
            
            TraverseZipline.traverseZipline();
            
            lightLocalizer.type = 2; // Localization after dismount
            lightLocalizer.runLightLocalization();
            
            // TODO Look for flag
            
            // Shallow crossing
            navigation.travelTo(Main.sh_ll_x, (Main.sh_ll_y + Main.sh_ur_y)/2); // Don't drown in river!
            FlagCapture.captureFlag();
            navigation.travelTo((Main.sv_ll_x + Main.sv_ur_x)/2, (Main.sh_ll_y + Main.sh_ur_y)/2);
            navigation.travelTo((Main.sv_ll_x + Main.sv_ur_x)/2, Main.sv_ll_y);
            
            // Go home!
            double xEnd, yEnd;
            if(Main.greenCorner==0) {xEnd = 0.5; yEnd = 0.5;}
            else if(Main.greenCorner==1) {xEnd = 11.5; yEnd = 0.5;}
            else if(Main.greenCorner==2) {xEnd = 11.5; yEnd = 11.5;}
            else {xEnd = 0.5; yEnd = 11.5;}
            navigation.travelTo(xEnd, yEnd); 

        } else if (teamColor.equals(TeamColor.RED)){ // RED
            UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer(LocalizationType.FALLING_EDGE);
            ultrasonicLocalizer.doLocalization(); // prevent NullPointerExceptions

            navigation.turnTo(0);

            lightLocalizer.type = 0; // First localization
            lightLocalizer.runLightLocalization();
            
            // Shallow crossing
            navigation.travelTo(Main.sh_ll_x, (Main.sh_ll_y + Main.sh_ur_y)/2); // Don't drown in river!
            FlagCapture.captureFlag();
            navigation.travelTo((Main.sv_ll_x + Main.sv_ur_x)/2, (Main.sh_ll_y + Main.sh_ur_y)/2);
            navigation.travelTo((Main.sv_ll_x + Main.sv_ur_x)/2, Main.sv_ll_y);
            
            // TODO Look for flag
            
            double xStart;
            if(Main.redCorner==0) xStart = 0;
            else if(Main.redCorner==1) xStart = 12;
            else if(Main.redCorner==2) xStart = 12;
            else xStart = 0;
            
            double yStart;
            if(Main.redCorner==0) yStart = 0;
            else if(Main.redCorner==1) yStart = 0;
            else if(Main.redCorner==2) yStart = 12;
            else yStart = 12;
            
            double distToZipline = Math.sqrt((xStart-Main.zo_g_x)*(xStart-Main.zo_g_x)
            		+(yStart-Main.zo_g_y)*(yStart-Main.zo_g_y));
            
            if(distToZipline < 6) { 
                navigation.travelTo(Main.zo_g_x, Main.zo_g_y);
            } else { // zipline too far to travel to it directly
                navigation.travelTo((Main.sv_ll_x+Main.zo_g_x)/2d, (Main.sv_ll_y+Main.zo_g_y)/2d);
                lightLocalizer.type = 1; // regular localization
                lightLocalizer.runLightLocalization();
                navigation.travelTo(Main.zo_g_x, Main.zo_g_y);
            }
            
            FlagCapture.captureFlag();
            
            lightLocalizer.type = 1; // Zipline localization
            lightLocalizer.runLightLocalization();
            
            navigation.pointTo(Main.zc_g_x, Main.zc_g_y);
            
            TraverseZipline.traverseZipline();
            
            lightLocalizer.type = 2; // Localization after dismount
            lightLocalizer.runLightLocalization();
            
            // Go home!
            double xEnd, yEnd;
            if(Main.redCorner==0) {xEnd = 0.5; yEnd = 0.5;}
            else if(Main.redCorner==1) {xEnd = 11.5; yEnd = 0.5;}
            else if(Main.redCorner==2) {xEnd = 11.5; yEnd = 11.5;}
            else {xEnd = 0.5; yEnd = 11.5;}
            navigation.travelTo(xEnd, yEnd);

        } // end else // RED

    }
}
