<<<<<<< HEAD
/*
 * OdometryDisplay.java
 */

// Nicolas Velastegui 260521419
// Siddiqui Hakim 260564770
// Group 26

package ca.mcgill.ecse211.dreamteamrobot.brick1.display;

import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Driver;

public class LCDDisplay extends Thread {

	private static final long DISPLAY_PERIOD = 250;
	private Driver driver;

	// constructor
	public LCDDisplay(Driver driver) {
		this.driver = driver;
	}

	// run method (required for Thread)
	public void run() {
		long displayStart, displayEnd;
		double[] position = new double[3];

//		// clear the display once
//		t.clear();
//
//		while (true) {
//			displayStart = System.currentTimeMillis();
//
//			// clear the lines for displaying odometry information
//			t.drawString("X:              ", 0, 0);
//			t.drawString("Y:              ", 0, 1);
//			t.drawString("T:              ", 0, 2);
//
//			// get the odometry information
//			odometer.getPosition(position, new boolean[] { true, true, true });
//
//			// display odometry information
//			for (int i = 0; i < 3; i++) {
//				t.drawString(formattedDoubleToString(position[i], 2), 3, i);
//			}
//
//			// throttle the LCDDisplay
//			displayEnd = System.currentTimeMillis();
//			if (displayEnd - displayStart < DISPLAY_PERIOD) {
//				try {
//					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
//				} catch (InterruptedException e) {
//					// there is nothing to be done here because it is not
//					// expected that LCDDisplay will be interrupted
//					// by another thread
//				}
//			}
//		}
	}
	
	private static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;
		
		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";
		
		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long)x;
			if (t < 0)
				t = -t;
			
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
				result += Long.toString((long)x);
			}
		}
		
		return result;
	}

}
=======
package ca.mcgill.ecse211.dreamteamrobot.brick1.display;

import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Driver;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

import java.util.List;

/**
 * This thread automatically samples data from a Driver thread, supplied in the constructor, and
 * displays it on the brick LCD. Classes may interrupt this automated procedure by calling sendToDisplay(),
 * which immediately displays some provided message.
 */
public class LCDDisplay extends Thread {

	private static final long DISPLAY_PERIOD = 250;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;
	private Driver driver;

	/**
	 * Constructor
	 * @param driver Driver thread.
     */
	public LCDDisplay(Driver driver) {
		this.driver = driver;
	}

	/**
	 * Displays message on screen.
	 * @param lines Message to be displayed, in a list where each element is a line.
     */
	public static void sendToDisplay (List<String> lines) {
//		LCD.clear();
//		LCD.drawString("X: ", 0, 0);
//		LCD.drawString("Y: ", 0, 1);
//		LCD.drawString("H: ", 0, 2);
	}

	/**
	 *
	 */
	public void run() {
		long displayStart, displayEnd;
		double[] position = new double[3];

//		// clear the display once
//		t.clear();
//
//		while (true) {
//			displayStart = System.currentTimeMillis();
//
//			// clear the lines for displaying odometry information
//			t.drawString("X:              ", 0, 0);
//			t.drawString("Y:              ", 0, 1);
//			t.drawString("T:              ", 0, 2);
//
//			// get the odometry information
//			odometer.getPosition(position, new boolean[] { true, true, true });
//
//			// display odometry information
//			for (int i = 0; i < 3; i++) {
//				t.drawString(formattedDoubleToString(position[i], 2), 3, i);
//			}
//
//			// throttle the LCDDisplay
//			displayEnd = System.currentTimeMillis();
//			if (displayEnd - displayStart < DISPLAY_PERIOD) {
//				try {
//					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
//				} catch (InterruptedException e) {
//					// there is nothing to be done here because it is not
//					// expected that LCDDisplay will be interrupted
//					// by another thread
//				}
//			}
//		}
	}
	
	private static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;
		
		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";
		
		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long)x;
			if (t < 0)
				t = -t;
			
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
				result += Long.toString((long)x);
			}
		}
		
		return result;
	}

}
>>>>>>> 457d65f4c52d44edce011a61b603db23cfb5bd86
