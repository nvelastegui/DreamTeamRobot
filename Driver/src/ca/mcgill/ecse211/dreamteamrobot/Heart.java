// Lab2.java

// Nicolas Velastegui 260521419
// Siddiqui Hakim 260564770
// Group 26

package ca.mcgill.ecse211.dreamteamrobot;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Central class.
 */
public class Heart {
	
	// Static Resources:
	// Motors, TextLCD
	// (We used the LCD for displaying information regarding the system during debugging)
	private static final Port usPort = LocalEV3.get().getPort("S1");
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor ultrasonicSensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final TextLCD t = LocalEV3.get().getTextLCD();

	// Constants
	public static final double WHEEL_RADIUS_L = 2.01; // Kept the wheel radius equal for both wheels.
	public static final double WHEEL_RADIUS_R = 2.01; // 2.05, 2.01
	public static final double TRACK = 14.45;          // Actual wheelbase is around 15 cm, but 14 cm was
													  // used because it resulted in the best turns.

	// was 14

	public static void main(String[] args) throws InterruptedException {

		// Reset the tacho counts in case (for whatever reason) they are not zero.
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();

		int buttonChoice;

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			// text, x coordinate on screen, y coordinate on screen
			t.drawString("<WeTheBestRover>", 0, 0); // DJ Khaled inspired insignia.
			t.drawString("< Left | Right >", 0, 1);
			t.drawString("Dance  |        ", 0, 2);
			t.drawString("Dance  | Move   ", 0, 3);
			t.drawString("Revolu-| with   ", 0, 4);
			t.drawString("tion   | avoid  ", 0, 5);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		/** If left is selected, then do motion as outlined in lab manual. */
		if (buttonChoice == Button.ID_LEFT) {

			// Below is the instantiation for the ultrasonic poller. It runs on it's own,
			// scanning for an approaching obstacle. It stores distances in its distance variable.
			SensorModes usSensor = new EV3UltrasonicSensor(usPort);
			SampleProvider usDistance = usSensor.getMode("Distance");
			float[] usData = new float[usDistance.sampleSize()];
			UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);

			// Instantiate new odometer, new odometryDisplay, and the navigator.
			Odometer odometer = new Odometer();
			OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);
			Navigator navigator = new Navigator(odometer, usPoller);

			odometer.start();
			odometryDisplay.start();
			usPoller.start();
			navigator.start();

			try {
				Thread.sleep(200);
			}
			catch (InterruptedException e) {
				e.printStackTrace();
			}

			completeCourse(navigator);

		}
		// Drive with the odometer, following a procedure.
		else {

			// Below is the instantiation for the ultrasonic poller. It runs on it's own,
			// scanning for an approaching obstacle. It stores distances in its distance variable.
			SensorModes usSensor = new EV3UltrasonicSensor(usPort);
			SampleProvider usDistance = usSensor.getMode("Distance");
			float[] usData = new float[usDistance.sampleSize()];
			UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);

			// Instantiate new odometer, new odometryDisplay, and the navigator.
			Odometer odometer = new Odometer();
			OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);
			Navigator navigator = new Navigator(odometer, usPoller);

			odometer.start();
			odometryDisplay.start();
			usPoller.start();
			navigator.start();

			try {
				Thread.sleep(200);
			}
			catch (InterruptedException e) {
				e.printStackTrace();
			}

			completeCourse2(navigator);

		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}


	private static void completeCourse(Navigator nav) throws InterruptedException {
		double [][] waypoints = {
				{60.0 , 30.0},
				{30.0 , 30.0},
				{30.0 , 60.0},
				{60.0 , 00.0}
		};

		for(double[] point : waypoints) {
			nav.travelTo(point[0], point[1]);
			while (nav.isNavigating()) {
				Thread.sleep(500);
			}
		}
	}

	private static void completeCourse2(Navigator nav) throws InterruptedException {
		double [][] waypoints = {
				{00.0 , 60.0},
				{60.0 , 00.0}
		};

		for(double[] point : waypoints) {
			nav.travelTo(point[0], point[1]);
			while (nav.isNavigating()) {
				Thread.sleep(500);
			}
		}
	}


}






















