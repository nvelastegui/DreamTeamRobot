package ca.mcgill.ecse211.dreamteamrobot.brick1.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.display.LCDDisplay;
import ca.mcgill.ecse211.dreamteamrobot.brick1.localization.Localization;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Central class.
 */
public class Main {

	// /** Constants */
	private static final TextLCD t = LocalEV3.get().getTextLCD();

	// /** Constants: Motor Ports */

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor leftUltrasonicSensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightUltrasonicSensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));


	// /** Constants: Sensor Ports */
	private static final Port leftUltrasonicSensorPort = LocalEV3.get().getPort("S1");
	private static final Port rightUltrasonicSensorPort = LocalEV3.get().getPort("S2");
	private static final Port leftColorSensorPort = LocalEV3.get().getPort("S3");
	private static final Port rightColorSensorPort = LocalEV3.get().getPort("S4");

	/**
	 * Before anything can begin to run on the robot, the robot needs to know if it's playing offensive or defensive.
	 * This Main class awaits the signal with details regarding the match, then runs the appropriate threads.
	 * @param args arggggg matey
	 * @throws InterruptedException
     */
	public static void main(String[] args) throws InterruptedException {

		// Reset the tacho counts in case (for whatever reason) they are not zero.
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();

		// Set initial diplay.
		t.clear();
		t.drawString("<DreamTeamRobot >", 0, 0);
		t.drawString("<               >", 0, 1);
		t.drawString("<  Initializing >", 0, 2);
		t.drawString("<   Classes     >", 0, 3);
		t.drawString("<               >", 0, 4);
		t.drawString("<               >", 0, 5);

		// Wait for signal


		// Process signal


		// Create a new instance of driver thread.
		Driver driver = new Driver(
				leftMotor,
				rightMotor,
				leftUltrasonicSensorMotor,
				rightUltrasonicSensorMotor,
				leftUltrasonicSensorPort,
				rightUltrasonicSensorPort,
				leftColorSensorPort,
				rightColorSensorPort
		);

		// Run pre-execute procedure (starts subthreads).
		driver.performPreExecute();

		// Set up lcdDisplay and run it.
		LCDDisplay lcdDisplay = new LCDDisplay(driver);
		lcdDisplay.start();

		// Perform localization.
		Localization localizer = new Localization(
				leftMotor,
				rightMotor,
				driver.getOdometer(),
				driver.getNavigator(),
				driver.getUltrasonicPollerLeft(),
				driver.getUltrasonicPollerRight(),
				driver.getColourPollerLeft(),
				driver.getColourPollerRight()
		);
		localizer.setupLocalizer();



		localizer.doLocalization(null);




	}

	/**
	 * Pauses thread to allow for wheel motions to finish.
	 * @param timeToStop amount of time to stop, in milliseconds.
	 */
	private static void pause(int timeToStop) {
		try {
			Thread.sleep(timeToStop);
		} catch (InterruptedException e) {
			e.printStackTrace();
			Sound.beep();
			Sound.twoBeeps();
		}
	}

}






















