package ca.mcgill.ecse211.ziplineproject;

/**
 * This class is used to test the Thread timing
 */
public class TestThreadTiming implements Runnable {
    
    /**Used to run the TestThreadTiming as a <b><code>Thread</code></b>*/public Thread runner;

    /**
     * Test the Thread timing
     * 
     * The purpose of this test is to determine how many Threads can be safely run on the EV3 platform.
     * The Test is carried out as follows:
     * 
     */
    public static void testThreadTiming() {
        // TODO 
        
    }
    
    /**
     * Run method for thread test
     */
    public void run() {
        double k = 2017;
        int evenNumber = 27726;
        for(int i = 0; i < evenNumber; i++) {
            if(i % 2 == 0) k = Math.tan(k); // k even
            else k = Math.atan(k);
        }
        System.out.println(k);
    }
    
    /**
     * Starts the TestThreadTiming Thread
     */
    public void start() {
        runner = new Thread(this);
        runner.start();
    }

    
}
