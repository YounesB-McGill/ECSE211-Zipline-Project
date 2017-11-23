package ca.mcgill.ecse211.ziplineproject;

import lejos.robotics.SampleProvider;

// TODO Remove this when timed smplimg is working
import static javax.swing.JOptionPane.showMessageDialog;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller
 * thread. 
 */
public class UltrasonicPoller implements Runnable {
    private SampleProvider us;
    private UltrasonicController cont;
    private float[] usData;
    
    /**Used to run the UltrasonicPoller as a <b><code>Thread</code></b>*/public Thread runner; 

    public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {
        this.us = us;
        this.cont = cont;
        this.usData = usData;
    }

    /**
     * Run the UltrasonicPoller to sample US data, and act on it if necessary
     */
    public void run() {
        /* From Lab 1:  
         * The while loop at the bottom executes in a loop. Assuming that the
         * us.fetchSample, and cont.processUSData methods operate in about 20
         * ms, and that the thread sleeps for 50 ms at the end of each loop,
         * then one cycle through the loop is approximately 70 ms. This
         * corresponds to a sampling rate of 1/70mS or about 14 Hz. */
        
        int distance;
        
        // TODO Add functional timed sampling based on Paarth's code, then remove alert and static import
        
        // Alert tester that feature is not implemented
        showMessageDialog(null, "Not yet implemented! "
                + "Add functional timed sampling based on Paarth's code");
        
        while (true) {
            
        }
    }
    
    /**
     * Starts the Odometer Thread
     */
    public void start() {
        runner = new Thread(this);
        runner.start();
    }

}
