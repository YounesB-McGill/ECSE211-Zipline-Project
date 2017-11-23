/*
 * OdometryDisplay.java
 */
package ca.mcgill.ecse211.ziplineproject;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/**
 * The <code>Display</code> class displays Odometer data on the device LCD using a Thread. It
 * also contains methods to format and display other information on the screen, including the user interface.
 */
public class Display implements Runnable {
    private static final long DISPLAY_PERIOD = 250;
    private Odometer odometer = Main.odometer;
    private static TextLCD textLCD = Main.textLCD;
    public Thread runner;
    
    /**
     * Controls whether to print odometry and battery information to Console (and on the robot screen)
     */
    public boolean printToConsole = Main.printToConsole;

    /**
     * Display constructor
     */
    public Display() {

    }

    /**
     * Runs the main display Thread, which displays Odometer data
     */
    public void run() {
        long displayStart, displayEnd;
        double[] position = new double[3];

        // clear the display once
        textLCD.clear();

        while (true) {
            displayStart = System.currentTimeMillis();

            // clear the lines for displaying odometry information
            textLCD.drawString("X:              ", 0, 0);
            textLCD.drawString("Y:              ", 0, 1);
            textLCD.drawString("T:              ", 0, 2);

            // get the odometry information
            odometer.getPosition(position, new boolean[] { true, true, true });

            // display odometry information in cm and deg
            for (int i = 0; i < 3; i++) {
                if(i < 2) // x or y
                    textLCD.drawString(formattedDoubleToString(position[i]/Main.TILE, 2), 3, i);
                else { // theta
                    if(position[i]<Math.PI)
                        textLCD.drawString(formattedDoubleToString(position[i]*180/Math.PI, 2), 3, i);
                    else // 180 <= theta <= 360
                        textLCD.drawString(formattedDoubleToString((position[i]*180/Math.PI)-360, 2), 3, i);
                }
            }

            // throttle the OdometryDisplay
            displayEnd = System.currentTimeMillis();
            if (displayEnd - displayStart < DISPLAY_PERIOD) {
                try {
                    Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
                } catch (InterruptedException e) {
                    // there is nothing to be done here because it is not
                    // expected that OdometryDisplay will be interrupted
                    // by another thread
                }
            }
            
            if(printToConsole) {
                System.out.println(formattedDoubleToString(position[0]/Main.TILE, 2) + ", " 
                             + formattedDoubleToString(position[1]/Main.TILE, 2) + ", " 
                             + formattedDoubleToString(position[2]*180/Math.PI, 2) + ", "
                             + formattedDoubleToString(LocalEV3.get().getPower().getVoltageMilliVolt(), 2)
            );}
        } // end while(true)
    }

    /**
     * Formats a double into a String representation of a decimal number 
     * @param x The double to be converted
     * @param places The desired number of decimal places
     * @return A string of the formatted double value
     */
    private static String formattedDoubleToString(double x, int places) {
        String result = "", stack = "";
        long t;
        // put in a minus sign as needed
        if (x < 0.0) result += "-";
        // put in a leading 0
        if (-1.0 < x && x < 1.0)
            result += "0";
        else {
            t = (long) x;
            if (t < 0) t = -t;
            while (t > 0) {
                stack = Long.toString(t % 10) + stack;
                t /= 10;
            }
            result += stack;
        }
        // put the decimal, if needed
        if (places > 0) {
            result += ".";
            // put the appropriate number of decimals
            for (int i = 0; i < places; i++) {
                x = Math.abs(x);
                x = x - Math.floor(x);
                x *= 10.0;
                result += Long.toString((long) x);
            }
        }
        return result;
    }
    
    /**
     * User interface method to select the start corner
     * @return The start corner, a number in the range [0, 3]
     */
    public static int getStartCornerUI() {
        int startCorner = Main.greenCorner;
        int buttonChoice = Main.buttonChoice;
        do {
            // clear the display
            textLCD.clear();
            textLCD.drawString(" Choose starting", 0, 0);
            textLCD.drawString("     corner:    ", 0, 1);
            textLCD.drawString("start from " + startCorner, 0, 2);
            textLCD.drawString("Left/right to   ", 0, 3);
            textLCD.drawString("change.         ", 0, 4);
            textLCD.drawString("Press center    ", 0, 5);
            textLCD.drawString("button to confirm", 0, 6);

            buttonChoice = Button.waitForAnyPress();

            if ((buttonChoice == Button.ID_RIGHT) && startCorner < 3) {
                startCorner++;
            } else if ((buttonChoice == Button.ID_LEFT) && startCorner > 0) {
                startCorner--;
            }
        } while (buttonChoice != Button.ID_ENTER);
        return startCorner;
    }
    
    /**
     * User interface method to select the zipline approach coordinates
     * @return The zipline coordinates, in the form of an int array {x0, y0}
     */
    public static int[] getXYUI() {
        int buttonChoice, x0 = 0 /*Main.zo_g_x*/, y0 = 0 /*Main.zo_g_y*/; // UI not needed for competition
        do {
            drawSetXYUI();
            buttonChoice = Button.waitForAnyPress();
            if ((buttonChoice == Button.ID_RIGHT) && x0 < 8) { x0++; } 
            else if ((buttonChoice == Button.ID_LEFT) && x0 > 0) { x0--; } 
            else if ((buttonChoice == Button.ID_UP) && y0 < 8) { y0++; }
            else if ((buttonChoice == Button.ID_DOWN) && y0 > 0) { y0--; }
        } while (buttonChoice != Button.ID_ENTER);
        return new int[] {x0, y0};
    }
    
    /**
     * Helper method to draw the <i>x<sub>0</sub></i> and <i>y<sub>0</sub></i> selection UI on the LCD
     */
    public static void drawSetXYUI() {
        int x0 = 0 /*Main.zo_g_x*/, y0 = 0 /*Main.zo_g_y*/; // UI not needed for competition
        textLCD.clear();
        textLCD.drawString("   Choose x,y   ", 0, 0);
        textLCD.drawString("   coordinate:  ", 0, 1);
        textLCD.drawString(" x=" + x0 + "    y=" + y0, 0, 2);
        textLCD.drawString("left/right for x ", 0, 3);
        textLCD.drawString("up/down for y   ", 0, 4);
        textLCD.drawString("press center    ", 0, 5);
        textLCD.drawString("button to confirm", 0, 6);
    }
    
    /**
     * Clears the display after a series of <code>out.print()</code> statements
     */
    public void clear() {
        for (int i = 0; i < 32; i++) {
            System.out.println();
        }
    }
    
    /**
     * Starts the Display Thread, which displays odometry information by default
     */
    public void start() {
        runner = new Thread(this);
        runner.start();
    }

}
