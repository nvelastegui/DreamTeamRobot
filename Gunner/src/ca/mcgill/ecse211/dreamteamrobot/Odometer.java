/*
 * Odometer.java
 */

// Nicolas Velastegui 260521419
// Siddiqui Hakim 260564770
// Group 26

package ca.mcgill.ecse211.dreamteamrobot;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;
	private double distancePerDegreeRotationLeft;
	private double distancePerDegreeRotationRight;

	private int leftMotorTachoCount;
	private double leftMotorDistance;
	private int rightMotorTachoCount;
	private double rightMotorDistance;
	private double newX;
	private double newY;
	private double newTheta;

	private double deltaTheta;
	private double deltaC;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer() {
		x = 0.0;
		y = 0.0;
		theta = 00.0; // Start theta at 90 so that x and y values are positive.
		// stop at example2?
		lock = new Object();
		// Initialize the distancePerDegreeRotation using the parameters in Lab2.
		distancePerDegreeRotationLeft = Math.PI * (Heart.WHEEL_RADIUS_L) / 180.00;
		distancePerDegreeRotationRight = Math.PI * (Heart.WHEEL_RADIUS_R) / 180.00;
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here

			// We want to grab the tacho count from the motors and immediately
			// reset them so that we don't lose any rotations.
			leftMotorTachoCount = Heart.leftMotor.getTachoCount();
			rightMotorTachoCount = Heart.rightMotor.getTachoCount();
			Heart.rightMotor.resetTachoCount();
			Heart.leftMotor.resetTachoCount();

			// theta goes from vertical axis (0 degrees) in a CW rotation.

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!

				// Determine deltaL and deltaR for calculation below.
				leftMotorDistance = distancePerDegreeRotationLeft * ((double) leftMotorTachoCount);
				rightMotorDistance = distancePerDegreeRotationRight * ((double) rightMotorTachoCount);

				// Determine (approximate) distances deltaC and deltaTheta
				// deltaC is just the average of the two individual motor distance values.
				deltaC = (leftMotorDistance + rightMotorDistance) / (2.0);
				// deltaTheta is more complicated - derived in the lab tutorial.
				// Heavily relies upon the time interval of readings being small (sufficiently small)
				// (But not infinitesimally small!)
				deltaTheta = (leftMotorDistance - rightMotorDistance) / Heart.TRACK;

				// Now we can calculate the new x, y, and theta values based on
				// the previous ones, using the recursive relationship derived
				// in the lab tutorial.
				newX = x + (deltaC) * Math.sin(theta + (deltaTheta)/2.0); // Used sin for x because of definition of angle
				newY = y + (deltaC) * Math.cos(theta + (deltaTheta)/2.0); // Used cos for y because of definition of angle
				newTheta = theta + deltaTheta;

				// New values have been finalized. Save them.
				x = newX;
				y = newY;
				theta = newTheta;

				// Theta upper bound
				if (theta > 6.28) {
					theta = theta - 6.28;
				}

				// Theta lower bound
				if (theta < 0) {
					theta = 6.28 + theta;
				}


			}

			// To ensure that the odometer only runs once every period...
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	/**
	 *
	 * @param position array of positions to update
	 * @param update boolean array dictating which elements of position array to update
	 */
	public void getPosition(double[] position, boolean[] update) {
		// Again, we ensure here that we are synchronized with lock
		// so that the values don't change while the odometer is running.
		synchronized (lock) {

			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	/**
	 * @return Current x position.
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/**
	 * @return Current y position.
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	/**
	 * @return Current theta.
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	/**
	 * @param position Array of position parameters
	 * @param update Boolean array to dictate which elements to update.
	 */
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * Set x to given value.
	 * @param x
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	/**
	 * Set y to given value.
	 * @param y
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	/**
	 * Set theta to given value.
	 * @param theta
	 */
	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
}