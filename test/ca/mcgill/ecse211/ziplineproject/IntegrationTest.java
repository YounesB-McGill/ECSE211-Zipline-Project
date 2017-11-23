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

        if (teamColor.equals(TeamColor.GREEN)) {
            new UltrasonicLocalizer(LocalizationType.FALLING_EDGE).doLocalization();

            navigation.turnTo(0);

            lightLocalizer.type = 0; // First localization
            lightLocalizer.runLightLocalization();

            if(Main.zo_g_x > 6) { // TODO Change this
                navigation.travelTo(Main.zo_g_x, Main.zo_g_y);
            } else { // zipline too far to travel to it directly
                navigation.travelTo(6, 2);
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
            navigation.travelTo(Main.sh_ll_x, Main.sh_ll_y + 0.5); // Don't drown in river!
            FlagCapture.captureFlag();
            navigation.travelTo(Main.sh_ur_x - 0.5, Main.sh_ur_y - 0.5);
            navigation.travelTo(Main.sv_ll_x + 0.5, Main.sv_ll_y - 0.5);
            
            // Go home!
            navigation.travelTo(11.5, 0.5); 

        } else { // RED
            UltrasonicLocalizer.doLocalization();

            navigation.turnTo(0);

            lightLocalizer.type = 0; // First localization
            lightLocalizer.runLightLocalization();
            
            // Shallow crossing
            navigation.travelTo(Main.sh_ll_x, Main.sh_ll_y + 0.5); // Don't drown in river!
            navigation.travelTo(Main.sh_ur_x - 0.5, Main.sh_ur_y - 0.5);
            navigation.travelTo(Main.sv_ll_x + 0.5, Main.sv_ll_y - 0.5);
            
            // TODO Look for flag
            
            if(Math.abs(Main.sv_ll_x - Main.zo_g_x) < 6) { 
                navigation.travelTo(Main.zo_g_x, Main.zo_g_y);
            } else { // zipline too far to travel to it directly
                navigation.travelTo(6, 2);
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
            navigation.travelTo(0.5, 11.5);

        } // end else // RED

    }
}
