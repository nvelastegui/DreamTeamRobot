package ca.mcgill.ecse211.dreamteamrobot.brick1.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.display.LCDDisplay;
import ca.mcgill.ecse211.dreamteamrobot.brick1.localization.Localization;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
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
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor leftUltrasonicSensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightRltrasonicSensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

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
		t.drawString("<DreamTeamRobotr>", 0, 0);
		t.drawString("<               >", 0, 1);
		t.drawString("<   Awaiting    >", 0, 2);
		t.drawString("<   Orders      >", 0, 3);
		t.drawString("<               >", 0, 4);
		t.drawString("<               >", 0, 5);

		// Wait for signal


		// Process signal


		// Set up the Driver thread, but do not run it.
		Driver driver = new Driver(leftMotor, rightMotor, rightRltrasonicSensorMotor, leftUltrasonicSensorPort, rightUltrasonicSensorPort);
		driver.performPreExecute();

		// Set up lcdDisplay and run it.
		LCDDisplay lcdDisplay = new LCDDisplay(driver);
		lcdDisplay.start();

		// Perform localization.
		ColourPoller leftColourPoller = new ColourPoller(leftColorSensorPort);
		// leftColourPoller.start();
		ColourPoller rightColourPoller = new ColourPoller(rightColorSensorPort);
		// rightColourPoller.start();
		Localization localizer = new Localization(
				leftMotor,
				rightMotor,
				driver.getOdometer(),
				driver.getNavigator(),
				driver.getUltrasonicPollerLeft(),
				driver.getUltrasonicPollerRight(),
				leftColourPoller,
				rightColourPoller
		);
		localizer.setupLocalizer();
		localizer.doLocalization(null);




	}

}





















