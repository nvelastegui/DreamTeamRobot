package ca.mcgill.ecse211.dreamteamrobot.brick1.navigation;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Odometer is a thread which uses the parameters in KinematicModel and changes in the tachometers
 * of the left and right motors to estimate the robot's current position.
 */
public class Odometer extends Thread {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	// robot position
	private double x, y, theta;
	private double distancePerDegreeRotationLeft;
	private double distancePerDegreeRotationRight;


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
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		x = 0.0;
		y = 0.0;
		theta = 00.0; // Start theta at 90 so that x and y values are positive.
		// stop at example2?
		lock = new Object();
		// Initialize the distancePerDegreeRotation using the parameters in Lab2.
		distancePerDegreeRotationLeft = Math.PI * (KinematicModel.WHEEL_RADIUS_L) / 180.00;
		distancePerDegreeRotationRight = Math.PI * (KinematicModel.WHEEL_RADIUS_R) / 180.00;
	}

	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	// run method (required for Thread)
	public void run() {

		// Variables: Timing
		long updateStart, updateEnd;

		// Reset the tacho counts.
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();

		// Variables: Keep track of last cycle's tacho reading
		long previousTachoLeft = 0;
		long previousTachoRight = 0;

		// Variables: Keep track of difference in this cycle's tacho reading
		// 			  and last cycle's tacho reading.
		long leftMotorTachoCount;
		long rightMotorTachoCount;

		// Variables: Distance travelled based on tacho counts.
		double leftMotorDistance;
		double rightMotorDistance;

		while (true) {
			updateStart = System.currentTimeMillis();

			// Grab the tacho count from the motors
			long currentTachoLeft = leftMotor.getTachoCount();
			long currentTachoRight = rightMotor.getTachoCount();

			// Calculate the rotations from the last cycle until now
			leftMotorTachoCount = currentTachoLeft - previousTachoLeft;
			rightMotorTachoCount = currentTachoRight - previousTachoRight;

			// Save the current tachos into previous slots.
			previousTachoLeft = currentTachoLeft;
			previousTachoRight = currentTachoRight;

			// >>> Theta goes from vertical axis (0 degrees) in a CW rotation.

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
				deltaTheta = (leftMotorDistance - rightMotorDistance) / KinematicModel.WHEELBASE;

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
				if (theta > 2.0*Math.PI) {
					theta = theta - 2.0*Math.PI;
				}

				// Theta lower bound
				if (theta < 0) {
					theta = 2.0*Math.PI + theta;
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