package ca.mcgill.ecse211.dreamteamrobot.brick1.display;

import ca.mcgill.ecse211.dreamteamrobot.brick1.communication.DriverStatusPacket;
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
	private static TextLCD LCD = LocalEV3.get().getTextLCD();
	private Driver driver;

	/**
	 * Constructor
	 * @param driver Driver thread.
     */
	public LCDDisplay(Driver driver) {
		this.driver = driver;
	}

	/**
	 * Displays message on screen. Logs it to system.out.
	 * @param message Message to be displayed, in a list where each element is a line.
     */
	public static void sendToDisplay (String message, boolean log) {
		// Clear line
		LCD.clear(5);
		// Display message on line.
		LCD.drawString(message, 0, 5);
		// Log
		if (log) System.out.println(message);
	}

	/**
	 *
	 */
	public void run() {
		long displayStart, displayEnd;
		double[] position = new double[3];

		// clear the display once
		LCD.clear();

		while (true) {

			displayStart = System.currentTimeMillis();

			// Clear display
			LCD.clear();

			// Get info from driver.
			DriverStatusPacket stats = driver.getStatus();

			// Display info on screen.
			LCD.drawString("Sts: " + stats.getCurrentState(), 0, 0);
			LCD.drawString("X: " + String.valueOf(stats.getX()), 0, 1);
			LCD.drawString("Y: " + String.valueOf(stats.getY()), 0, 2);
			LCD.drawString("T: " + String.valueOf(stats.getTheta()), 0, 3);
			LCD.drawString("  ", 0, 4);
			// line 5 reserved for messages sent in via sendToDisplay.

			// throttle the LCDDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that LCDDisplay will be interrupted
					// by another thread
				}
			}
		}
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
