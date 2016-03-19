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

	// /** Constants */
	private static int ROTATE_SPEED = 50;

	// /** Localization Related Constants */
	private static double d = 20;
	private static double k = 2;
	private static double pDTolerance = 50;
	private static double tolDistanceToWall = 2;
	private static double pDToleranceRisingEdge = 70;
	private static double thetaCompensation = 0.09; // amount to add to final theta value to compensate for error.

	// /** Instance Variables */
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private Navigator navigator;
	private UltrasonicPoller ultrasonicPollerLeft;
	private UltrasonicPoller ultrasonicPollerRight;

	/**\
	 * @param leftMotor Left wheel motor.
	 * @param rightMotor Right wheel motor.
	 * @param odo Odometer.
	 * @param navigator Navigator.
	 * @param ultrasonicPollerLeft Ultrasonic poller for reading left distance value.
	 * @param ultrasonicPollerRight Ultrasonic poller for reading right distance value.
     */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odo, Navigator navigator, UltrasonicPoller ultrasonicPollerLeft, UltrasonicPoller ultrasonicPollerRight) {
		this.odometer = odo;
		this.navigator = navigator;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.ultrasonicPollerLeft = ultrasonicPollerLeft;
		this.ultrasonicPollerRight = ultrasonicPollerRight;
	}

	/**
	 * Uses ultrasonic localization technique to estimate starting angle of robot. Rotates robot to 0 degrees.
	 */
	public boolean doLocalization() {

		double angleA, angleB;
		double deltaTheta;

		// The code below (using the ultrasonic sensor) provides a basic
		// approximation to the starting angle theta.

		if (isFacingWall()) {
			// If condition is true, then robot is facing a wall,
			// and we need to rotate it 180 degrees to start the localization.
			navigator.turnToAngle(Math.PI);
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

	}

	/**
	 * @return True if robot is facing a wall within distance pDTolerance. False if else.
     */
	private boolean isFacingWall () {

		// Grab the distances from each sensor.
		List<Double> listDistances = getFilteredData();
		double distanceLeft = listDistances.get(0);
		double distanceRight = listDistances.get(1);

		// If either one is reading less than pDTolerance, then return true;
		return (distanceLeft < pDTolerance) || (distanceRight < pDTolerance);

	}

	/**
	 * Rotates robot right until a wall falls below the (d-k) threshhold.
	 * @return angleA at which wall was too close.
	 */
	private double rotateRightUntilWallTooClose () {

		// Make an array list to store all of the values.
		List<List<Double>> listDistances = new ArrayList<>();
		List<Double> listThetas = new ArrayList<Double>();

		// Start motors going right.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.forward();
		rightMotor.backward();

		// Rotate until the distances read by both sensors are roughly the same.
		//		-> ie. robot is essentially perpendicular to wall.
		// Keep a backlog of all past readings and corresponding thetas.

		// Start by grabbing an initial reading and setting the conditional variables.
		List<Double> currentDistances = getFilteredData();
		double distanceLeft = currentDistances.get(0);
		double distanceRight = currentDistances.get(1);

		// Log the current distances.
		listDistances.add(currentDistances);
		listThetas.add(odometer.getTheta());

		// Rotate until within tolerance.
		// TODO: Incorporate d-k mechanic like before?
		while (Math.abs(distanceLeft - distanceRight) > (tolDistanceToWall)) {

			// Grab the current distances and theta and record them.
			currentDistances = getFilteredData();
			listThetas.add(odometer.getTheta());
			listDistances.add(currentDistances);

			// Update distances for conditional variables.
			distanceLeft = currentDistances.get(0);
			distanceRight = currentDistances.get(1);

		}

		// Stop motors and sound beep to signal that wall was found.
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();

		// Return angleA
		return listThetas.get(listThetas.size() - 1);

	}

	/**
	 * Rotates robot left until a wall falls below the (d-k) theshhold.
	 * @return angleB at which wall was too close.
	 */
	private double rotateLeftUntilWallTooClose () {

		// Make an array list to store all of the values.
		List<List<Double>> listDistances = new ArrayList<>();
		List<Double> listThetas = new ArrayList<Double>();

		// Start motors going right.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.backward();
		rightMotor.forward();

		// Sleep thread for a bit so that right-side wall doesn't confuse readings.
		pause(2000);

		// Start by grabbing an initial reading and setting the conditional variables.
		List<Double> currentDistances = getFilteredData();
		double distanceLeft = currentDistances.get(0);
		double distanceRight = currentDistances.get(1);

		// Log the current distances.
		listDistances.add(currentDistances);
		listThetas.add(odometer.getTheta());

		// Rotate until within tolerance.
		// TODO: Incorporate d-k mechanic like before?
		while (Math.abs(distanceLeft - distanceRight) > (tolDistanceToWall)) {

			// Grab the current distances and theta and record them.
			currentDistances = getFilteredData();
			listThetas.add(odometer.getTheta());
			listDistances.add(currentDistances);

			// Update distances for conditional variables.
			distanceLeft = currentDistances.get(0);
			distanceRight = currentDistances.get(1);

		}

		// Stop motors and sound beep to signal that wall was found.
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();

		// Return angleB
		return (listThetas.get(listThetas.size() - 1));

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
	 * @return Returns data reading from ultrasonic sensors, filtered.
	 */
	private List<Double> getFilteredData() {

		// Grab distance reading from left sensor.
		double distanceLeft = ultrasonicPollerLeft.getDistance();
		if (distanceLeft > 255) distanceLeft = 255;

		// Grab distance reading from right sensor.
		double distanceRight = ultrasonicPollerRight.getDistance();
		if (distanceRight > 255) distanceRight = 255;

		// Add both to return list.
		List<Double> newList = new ArrayList<>();
		newList.add(distanceLeft);
		newList.add(distanceRight);
		return newList;
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
