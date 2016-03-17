package ca.mcgill.ecse211.dreamteamrobot.brick1.localization;

import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.UltrasonicPoller;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import java.util.ArrayList;
import java.util.List;

/**
 * Performs preliminary localization based on ultrasonic sensor readings.
 */
public class UltrasonicLocalizer {

	/** Constants */
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	private static LocalizationType locType = LocalizationType.FALLING_EDGE;
	private static int ROTATE_SPEED = 50;

	/** Localization Related Constants */
	private static double d = 20;
	private static double k = 2;
	private static double pDTolerance = 50;
	private static double pDToleranceRisingEdge = 70;
	private static double thetaCompensation = 0.09; // amount to add to final theta value to compensate for error.

	/** Instance Variables */
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private Navigator navigator;
	private UltrasonicPoller ultrasonicPoller;

	/**\
	 * @param leftMotor Left wheel motor.
	 * @param rightMotor Right wheel motor.
	 * @param odo Odometer.
	 * @param navigator Navigator.
	 * @param ultrasonicPoller Ultrasonic poller for reading distance values.
     */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odo, Navigator navigator, UltrasonicPoller ultrasonicPoller) {
		this.odometer = odo;
		this.navigator = navigator;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.ultrasonicPoller = ultrasonicPoller;
	}

	/**
	 * Uses ultrasonic localization technique to estimate starting angle of robot. Rotates robot to 0 degrees.
	 */
	public boolean doLocalization() {

		double [] pos = new double [3];
		double angleA, angleB;
		double deltaTheta;

		// Now, depending on the parameter given to locType, we either run the FALLING_EDGE
		// or RISING_EDGE localization technique.

		// The code below (using the ultrasonic sensor) provides a basic
		// approximation to the starting angle theta.

		if (locType == LocalizationType.FALLING_EDGE) {

			double preliminaryDistance = getFilteredData();

			if (preliminaryDistance < pDTolerance) {
				// ie. the robot starts facing the wall.
				// Rotate it 180 degrees and reset theta.

				//navigator.turnToAngle(Math.PI);
				//rotateRight(Math.PI);
				odometer.setTheta(0.00);
			}

			// Now, assuming the robot starts facing away from the wall...

			/** 1. Determine angles A and B */
			angleA = rotateRightUntilWallTooClose();
			angleB = rotateLeftUntilWallTooClose();

			/** 2. Calculate deltaTheta */
			deltaTheta = getDeltaTheta(angleA, angleB);

			/** 3. Add the offset to current theta reading */
			odometer.setTheta(odometer.getTheta() + deltaTheta + thetaCompensation);

			Sound.twoBeeps();

			return true;

		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */

			double preliminaryDistance = getFilteredData();

			if (preliminaryDistance > pDToleranceRisingEdge) {
				// ie. the robot starts facing the wall.
				// Rotate it 180 degrees and reset theta.

				//navigator.turnToAngle(Math.PI);
				//rotateRight(Math.PI);
				odometer.setTheta(0.00);
			}

			// Assuming the robot starts facing the wall.

			/** 1. Determine angles A and B */
			angleA = rotateRightUntilWallTooFar();
			angleB = rotateLeftUntilWallTooFar();

			/** 2. Calculate deltaTheta */
			deltaTheta = getDeltaTheta(angleA, angleB);

			/** 3. Add the offset to current theta reading */
			odometer.setTheta(odometer.getTheta() + deltaTheta);

			Sound.twoBeeps();

			//navigator.turnToAngle(Math.PI);
			//rotateRight(Math.PI);
			odometer.setTheta(0.00);

			return true;
		}

	}

	/**
	 * Rotates robot right until a wall falls below the (d-k) threshhold.
	 * @return angleA at which wall was too close.
	 */
	private double rotateRightUntilWallTooClose () {

		// Make an array list to store all of the values.
		List<Double> listReadings = new ArrayList<Double>();
		List<Double> listThetas = new ArrayList<Double>();

		// Start motors going right.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();

		// Rotate until the distance drops below (d-k).
		// Keep a backlog of all past readings and corresponding thetas.
		double distance = getFilteredData();
		while (distance > (d-k)) {
			distance = getFilteredData();
			listReadings.add(distance);
			listThetas.add(odometer.getTheta());
		}

		// Stop motors and sound beep to signal that wall was found.
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();

		// Do some data analysis to determine angleA.

		// Determine the theta for when values first dropped below (d+k)
		// (might be the same theta as for when values dropped below (d-k))
		int firstdrop = -1; // Start this at -1.
		for (Double listReadingsElement : listReadings) {
			firstdrop += 1; // Goes up to 0 on first run through loop. List index starts at 0.
			if (listReadingsElement < (d+k)) {
				break;
			}
		}

		// Grab theta for when values dropped below (d-k) -> this is the last item in listThetas.
		int seconddrop = listThetas.size() - 1;

		// Return angleA
		return (listThetas.get(seconddrop) + listThetas.get(firstdrop))/2.0;

	}

	/**
	 * Rotates robot left until a wall falls below the (d-k) theshhold.
	 * @return angleB at which wall was too close.
	 */
	private double rotateLeftUntilWallTooClose () {

		// Make an array list to store all of the values.
		List<Double> listReadings = new ArrayList<Double>();
		List<Double> listThetas = new ArrayList<Double>();

		// Start motors going right.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.backward();
		rightMotor.forward();

		// Sleep thread for a bit so that right-side wall doesn't confuse readings.
		pause(2000);

		// Rotate until the distance drops below (d-k).
		// Keep a backlog of all past readings and corresponding thetas.
		double distance = getFilteredData();
		while (distance > (d-k)) {
			distance = getFilteredData();
			listReadings.add(distance);
			listThetas.add(odometer.getTheta());
		}

		// Stop motors and sound beep to signal that wall was found.
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();

		// Do some data analysis to determine angleA.

		// Determine the theta for when values first dropped below (d+k)
		// (might be the same theta as for when values dropped below (d-k))
		int firstdrop = -1; // Start this at -1.
		for (Double listReadingsElement : listReadings) {
			firstdrop += 1; // Goes up to 0 on first run through loop. List index starts at 0.
			if (listReadingsElement < (d+k)) {
				break;
			}
		}

		// Grab theta for when values dropped below (d-k) -> this is the last item in listThetas.
		int seconddrop = listReadings.size() - 1;

		// Return angleB
		return (listThetas.get(seconddrop) + listThetas.get(firstdrop))/2.0;

	}

	/**
	 * Rotates robot right until a wall falls below the (d-k) threshhold.
	 * @return angleA at which wall was too close.
	 */
	private double rotateRightUntilWallTooFar () {

		// Make an array list to store all of the values.
		List<Double> listReadings = new ArrayList<Double>();
		List<Double> listThetas = new ArrayList<Double>();

		// Start motors going right.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();

		// Rotate until the distance drops below (d-k).
		// Keep a backlog of all past readings and corresponding thetas.
		double distance = getFilteredData();
		while (distance < (d+k)) {
			distance = getFilteredData();
			listReadings.add(distance);
			listThetas.add(odometer.getTheta());
		}

		// Stop motors and sound beep to signal that wall was found.
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();

		// Do some data analysis to determine angleA.

		// Determine the theta for when values first dropped below (d+k)
		// (might be the same theta as for when values dropped below (d-k))
		int firstdrop = -1; // Start this at -1.
		for (Double listReadingsElement : listReadings) {
			firstdrop += 1; // Goes up to 0 on first run through loop. List index starts at 0.
			if (listReadingsElement > (d-k)) {
				break;
			}
		}

		// Grab theta for when values dropped below (d-k) -> this is the last item in listThetas.
		int seconddrop = listReadings.size() - 1;

		// Return angleA
		return (listThetas.get(seconddrop) + listThetas.get(firstdrop))/2.0;

	}

	/**
	 * Rotates robot left until a wall falls below the (d-k) theshhold.
	 * @return angleB at which wall was too close.
	 */
	private double rotateLeftUntilWallTooFar () {

		// Make an array list to store all of the values.
		List<Double> listReadings = new ArrayList<Double>();
		List<Double> listThetas = new ArrayList<Double>();

		// Start motors going right.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.backward();
		rightMotor.forward();

		// Sleep thread for a bit so that right-side wall doesn't confuse readings.
		pause(2000);

		// Rotate until the distance drops below (d-k).
		// Keep a backlog of all past readings and corresponding thetas.
		double distance = getFilteredData();
		while (distance < (d+k)) {
			distance = getFilteredData();
			listReadings.add(distance);
			listThetas.add(odometer.getTheta());
		}

		// Stop motors and sound beep to signal that wall was found.
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();

		// Do some data analysis to determine angleA.

		// Determine the theta for when values first dropped below (d+k)
		// (might be the same theta as for when values dropped below (d-k))
		int firstdrop = -1; // Start this at -1.
		for (Double listReadingsElement : listReadings) {
			firstdrop += 1; // Goes up to 0 on first run through loop. List index starts at 0.
			if (listReadingsElement > (d-k)) {
				break;
			}
		}

		// Grab theta for when values dropped below (d-k) -> this is the last item in listThetas.
		int seconddrop = listReadings.size() - 1;

		// Return angleB
		return (listThetas.get(seconddrop) + listThetas.get(firstdrop))/2.0;

	}


	/**
	 * @return Value of delta theta for corresponding angles A and B.
	 */
	private double getDeltaTheta (double angleA, double angleB) {

		double deltaTheta;
		// Delta theta is calculated using method defined in lab tutorial
		if (Math.abs(angleA) > Math.abs(angleB)) {
			deltaTheta = (Math.PI/4.0) - (angleA + angleB)/2.0;
		}
		else {
			deltaTheta = (Math.PI/4.0 + Math.PI) - (angleA + angleB)/2.0;
		}

		return deltaTheta;

	}


	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}


	/**
	 * @return Returns data reading from ultrasonic sensor, filtered.
	 */
	private double getFilteredData() {
		double distance = ultrasonicPoller.getDistance();
		if (distance > 255) distance = 255;
		return distance;
	}

	/**
	 * Pauses thread to allow for wheel motions to finish.
	 * @param timeToStop amount of time to stop, in milliseconds.
	 */
	private void pause(int timeToStop) {
		try {
			Thread.sleep(timeToStop);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

}
