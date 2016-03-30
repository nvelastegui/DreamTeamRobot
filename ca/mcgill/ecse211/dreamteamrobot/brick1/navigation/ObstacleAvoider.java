package ca.mcgill.ecse211.dreamteamrobot.brick1.navigation;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import java.util.ArrayList;
import java.util.List;

/**
 * This is a sub-thread for a Navigator thread, which is run when Navigator determines that an obstacle
 * is ahead. ObstacleAvoider runs a wall following algorithm to drive around the obstacle.
 */
public class ObstacleAvoider extends Thread {

	enum AvoidanceDirection {LEFT, RIGHT}

	// /** enum: bangbang or p controller */
	enum Controller {BANGBANG, PCONT};
	private Controller chosenController = Controller.BANGBANG;
	// BangBang
	private static final int bandCenterBB = 38;			// Offset from the wall (cm) adjusted for 45 degree angle of sensor
	private static final int bandWidthBB = 6;			// Width of dead band (cm) adjusted for 45 degree angle of sensor
	private static final int motorLowBB = 100;			// Speed of slower rotating wheel (deg/sec)
	private static final int motorHighBB = 175;			// Speed of the faster rotating wheel (deg/seec)
	// P Controller
	int bandCenterPC = 40; // was 32
	int bandwidthPC = 5;
	int motorStraightPC = 200; // was 200

	/** Constants */
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 80;

	/** Variables: Simultaneous Threads */
	private Navigator nav;
	private Odometer odometer;
	private UltrasonicPoller ultrasonicPollerLeft;
	private UltrasonicPoller ultrasonicPollerRight;

	/** Variables: Motors */
	private EV3LargeRegulatedMotor ultrasonicSensorMotorLeft;
	private EV3LargeRegulatedMotor ultrasonicSensorMotorRight;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	/** */
	private Location finalPosition;
	private Location startingPosition;
	private double slope;
	boolean safe;

	/** tolerances */
	double tolSlope = 45;
	double tolFinalTheta = 0.40;

	/**
	 * Constructor.
	 * @param nav
	 * @param ultrasonicSensorMotorLeft
	 * @param ultrasonicSensorMotorRight
	 * @param leftMotor
     * @param rightMotor
     */
	public ObstacleAvoider(Navigator nav, EV3LargeRegulatedMotor ultrasonicSensorMotorLeft, EV3LargeRegulatedMotor ultrasonicSensorMotorRight, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.nav = nav;
		this.odometer = nav.getOdometer();
		this.ultrasonicPollerLeft = nav.getUsPollerLeft();
		this.ultrasonicPollerRight = nav.getUsPollerRight();
		this.finalPosition = nav.getCurrentDestination();
		this.ultrasonicSensorMotorLeft = ultrasonicSensorMotorLeft;
		this.ultrasonicSensorMotorRight = ultrasonicSensorMotorRight;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.safe = false;
	}

	/**
	 * Rotates given ultrasonic sensor from North to East or West and measures distances along the way.
	 * @param rotationDirectionMultiplier Multiplier (1 or -1) that determines the direction of rotation.
	 * @param ultrasonicSensorMotor Motor for ultrasonic sensor.
	 * @param poller Ultrasonic sensor poller.
	 * @return Percentage of distances measured which are a comfortable distance away.
     */
	private double getPercentageViewIsClear (int rotationDirectionMultiplier, EV3LargeRegulatedMotor ultrasonicSensorMotor, UltrasonicPoller poller) {

		// Set rotation speed really low.
		ultrasonicSensorMotor.setSpeed(KinematicModel.obstacleAvoider_ultrasonicSensorRotationSpeedLow);
		ultrasonicSensorMotor.setAcceleration(3000);

		// Begin rotation until horizontal.
		ultrasonicSensorMotor.rotate(rotationDirectionMultiplier*90, true);

		// Meanwhile, take distance measurements (while the motor is rotating).
		List<Integer> listMeasurements = new ArrayList<>();
		while (ultrasonicSensorMotor.isMoving()) {
			// (Keep in mind that ultrasonic poller takes new reading every 50 ms.)
			listMeasurements.add(poller.getDistance());
			try {
				Thread.sleep(60);
			} catch (Exception e) {
			}
		}

		// Count the number of measurements that are comfortable distance away from robot.
		int numberClearMeasurements = 0;
		for (Integer current : listMeasurements) {
			if (current > KinematicModel.obstacleAvoider_tolCloseness) numberClearMeasurements++;
		}

		// Return percentage of measurements that are comfortable measurements.
		return (double) numberClearMeasurements / listMeasurements.size();

	}


	@Override
	public void run() {

		// First things first. We need to check which path around the obstacle is best.

		// Scan left and right directions to determine which is safer.
		// TODO: check the multipliers here
		double percentageLeftClear = getPercentageViewIsClear(-1, ultrasonicSensorMotorLeft, ultrasonicPollerLeft);
		double percentageRightClear = getPercentageViewIsClear(-1, ultrasonicSensorMotorRight, ultrasonicPollerRight);

		AvoidanceDirection direction;
		if (percentageLeftClear > percentageRightClear) direction = AvoidanceDirection.LEFT;
		else direction = AvoidanceDirection.RIGHT;

		// Turn both ultrasonic sensors to 45 degrees.
		ultrasonicSensorMotorLeft.rotate(45, true);    // TODO: determine appropriate angle
		ultrasonicSensorMotorRight.rotate(45, false);

		// Record the starting position for the wall-following.
		startingPosition = nav.getPositionFromOdometer();

		// Move back a little.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 60), true);    // Return immediately
		rightMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 60), false); // Return when motion is completed.

		// Wall-follow, baby!
		switch (direction) {

			/** Wall-Following Left */
			case LEFT:

				// 3. Rotate the robot 90 degrees left.
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 90), true);    // Return immediately
				rightMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 90), false); // Return when motion is completed.

				// Grab the starting angle.
				double startingThetaForLeftDirection = odometer.getTheta();
				// Use this angle to determine the IDEAL termination angle
				// (when the robot is on the opposite side of the obstacle).
				double finalThetaForLeftDirection    = startingThetaForLeftDirection + Math.PI;
				// If the angle wraps around too far, bring it back to [0, 6.28] range
				if (finalThetaForLeftDirection > 2.0*Math.PI) finalThetaForLeftDirection = finalThetaForLeftDirection - 2.0*Math.PI;

				// Now we can begin to wall-follow around the object.
				while (true) {

					// Grab distance from the ultrasonicPoller.
					int distance = ultrasonicPollerRight.getDistance();

					// Do correction based on chosen wall-following controller.
					if ( Math.abs(distance - bandCenterBB) < bandWidthBB/2) {
						// If the robot is within the bandwidth, then go straight.
						leftMotor.setSpeed(motorLowBB);
						leftMotor.forward();
						rightMotor.setSpeed(motorLowBB);
						rightMotor.forward();
					}
					else if (distance < bandCenterBB) {
						// If the robot is too close, turn out.

						// Speed up right motor
						rightMotor.setSpeed(motorHighBB);
						rightMotor.forward();

						// Remain reg speed left motor
						leftMotor.setSpeed(motorLowBB);
						leftMotor.forward();
					}
					else if (distance > bandCenterBB) {
						// If robot is too far, turn in.

						// Speed up left motor
						leftMotor.setSpeed(motorHighBB);
						leftMotor.forward();

						// Remain reg speed right motor
						rightMotor.setSpeed(motorLowBB);
						rightMotor.forward();
					}
					else {
						// Do nothing. (Stay forward)
						// This is run very rarely.
						leftMotor.setSpeed(motorLowBB);
						leftMotor.forward();
						rightMotor.setSpeed(motorLowBB);
						rightMotor.forward();
					}

					// The termination condition for the obstacle avoidance is theta.
					// When the theta is roughly around the opposite of where it started, then we know
					// that the robot is more or less around the object and we can resume trajectory.
					double currentTheta = odometer.getTheta();
					if ((finalThetaForLeftDirection - tolFinalTheta < currentTheta) && (currentTheta < tolFinalTheta + finalThetaForLeftDirection)) {
						// If it has, then break;
						break;
					}

					try {
						Thread.sleep(50);
					} catch (Exception e) {
					} // Poor man's timed sampling
				}

				break;

			/** Wall-Following Right */
			case RIGHT:

				// 3. Rotate the robot 90 degrees right.
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 90), true);    // Return immediately
				rightMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 90), false); // Return when motion is completed.

				// Grab the starting angle.
				double startingTheta = odometer.getTheta();
				// Use this angle to determine the IDEAL termination angle
				// (when the robot is on the opposite side of the obstacle).
				double finalTheta    = startingTheta + Math.PI;
				// If the angle wraps around too far, bring it back to [0, 6.28] range
				if (finalTheta > 2.0*Math.PI) finalTheta = finalTheta - 2.0*Math.PI;

				// Now we can begin to wall-follow around the object.
				while (true) {

					// Grab distance from the ultrasonicPoller.
					int distance = ultrasonicPollerLeft.getDistance();

					// Do correction based on chosen wall-following controller.
					if ( Math.abs(distance - bandCenterBB) < bandWidthBB/2) {
						// If the robot is within the bandwidth, then go straight.
						leftMotor.setSpeed(motorLowBB);
						leftMotor.forward();
						rightMotor.setSpeed(motorLowBB);
						rightMotor.forward();
					}
					else if (distance < bandCenterBB) {
						// If the robot is too close, turn out.

						// Speed up left motor
						leftMotor.setSpeed(motorHighBB);
						leftMotor.forward();

						// Remain reg speed right motor
						rightMotor.setSpeed(motorLowBB);
						rightMotor.forward();
					}
					else if (distance > bandCenterBB) {
						// If robot is too far, turn in.

						// Speed up right motor
						rightMotor.setSpeed(motorHighBB);
						rightMotor.forward();

						// Remain reg speed left motor
						leftMotor.setSpeed(motorLowBB);
						leftMotor.forward();
					}
					else {
						// Do nothing. (Stay forward)
						// This is run very rarely.
						leftMotor.setSpeed(motorLowBB);
						leftMotor.forward();
						rightMotor.setSpeed(motorLowBB);
						rightMotor.forward();
					}

					// The termination condition for the obstacle avoidance is theta.
					// When the theta is roughly around the opposite of where it started, then we know
					// that the robot is more or less around the object and we can resume trajectory.
					double currentTheta = odometer.getTheta();
					if ((finalTheta - tolFinalTheta < currentTheta) && (currentTheta < tolFinalTheta + finalTheta)) {
						// If it has, then break;
						break;
					}

					try {
						Thread.sleep(50);
					} catch (Exception e) {
					} // Poor man's timed sampling
				}

				break;

		}

		// Return ultrasonic sensors to forward position.
		ultrasonicSensorMotorLeft.rotate(45, false);
		ultrasonicSensorMotorRight.rotate(45, false);
		// Set safe to true, to signal that obstacle issue has been resolved.
		safe = true;
		return;


	}

	/**
	 * This method is called by a parent thread to check if the obstacle has been surpassed.
	 * @return True if obstacle surpassed, false if not.
     */
	public boolean isResolved() {
		return safe;
	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
