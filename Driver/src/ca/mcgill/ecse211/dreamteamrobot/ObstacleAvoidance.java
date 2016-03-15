package ca.mcgill.ecse211.dreamteamrobot;

// Nicolas Velastegui 260521419
// Siddiqui Hakim 260564770
// Group 26

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Created by nicolas on 2016-02-07.
 */
public class ObstacleAvoidance extends Thread {

	/** enum: bangbang or p controller */
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

	/** Variables: threads */
	private Navigator nav;
	private Odometer odometer;
	private UltrasonicPoller ultrasonicPoller;

	/** Variables */
	private EV3LargeRegulatedMotor ultrasonicSensorMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Location finalPosition;
	private Location startingPosition;
	private double slope;
	boolean safe;

	/** tolerances */
	double tolSlope = 45;
	double tolFinalTheta = 0.40;

	/** Constructor */
	public ObstacleAvoidance (Navigator nav) {
		this.nav = nav;
		this.odometer = nav.getOdometer();
		this.ultrasonicPoller = nav.getUsPoller();
		this.finalPosition = nav.getCurrentDestination();
		this.ultrasonicSensorMotor = Heart.ultrasonicSensorMotor;
		this.leftMotor = Heart.leftMotor;
		this.rightMotor = Heart.rightMotor;
		this.safe = false;
	}

	@Override
	public void run() {

		// Some stuff needs to be done before obstacle avoidance can begin.

		// 0. Record the starting position for the wall-following.
		//    and determine the slope of the line joining the two.
		startingPosition = nav.getPositionFromOdometer();
		slope = (finalPosition.getY() - startingPosition.getY())/(finalPosition.getX() - startingPosition.getX());

		// 1. Turn the ultrasonic sensor to the appropriate angle.
		ultrasonicSensorMotor.rotate(-45, false);    // TODO: determine appropriate angle

		// 2. Move back a little.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(Heart.WHEEL_RADIUS_L, Heart.TRACK, 60), true);    // Return immediately
		rightMotor.rotate(-convertAngle(Heart.WHEEL_RADIUS_R, Heart.TRACK, 60), false); // Return when motion is completed.

		// 3. Rotate the robot 90 degrees right.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(Heart.WHEEL_RADIUS_L, Heart.TRACK, 90), true);    // Return immediately
		rightMotor.rotate(-convertAngle(Heart.WHEEL_RADIUS_R, Heart.TRACK, 90), false); // Return when motion is completed.

		// Grab the starting angle.
		double startingTheta = odometer.getTheta();
		// Use this angle to determine the IDEAL termination angle
		// (when the robot is on the opposite side of the obstacle).
		double finalTheta    = startingTheta + Math.PI;
		// If the angle wraps around too far, bring it back to [0, 6.28] range
		if (finalTheta > 2.0*Math.PI) finalTheta = finalTheta - 2.0*Math.PI;

		// Now we can begin to wall-follow around the object.
		while (true) {

			// Grab distance from ultrasonicPoller.
			int distance = ultrasonicPoller.getDistance();

			// Do correction based on chosen wall-following controller.
			switch (chosenController) {
				case BANGBANG:

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

					break;
				case PCONT:
					if (distance > 51) distance = 51;

					// Set error appropriately.
					int error = bandCenterPC - distance;

					// Our wall-following code from lab1 is below. It is completely unchanged
					// from lab1, except that I have removed the comments.

					// Wall follow.
					if (Math.abs(error) <= bandwidthPC) {

						leftMotor.forward();
						rightMotor.forward();

					} else if (error < 0 && error >= -20) {

						leftMotor.setSpeed(50 + error);
						rightMotor.setSpeed(120 - error);
						leftMotor.forward();
						rightMotor.forward();
						leftMotor.setSpeed(motorStraightPC);
						rightMotor.setSpeed(2 * motorStraightPC);
						leftMotor.forward();
						rightMotor.forward();

					} else if (error > 0 && error <= 20) {

						leftMotor.setSpeed(motorStraightPC + (2 * error));
						rightMotor.setSpeed(100 - (4 * error));
						leftMotor.forward();
						rightMotor.forward();

					} else if (error > 20) {

						leftMotor.setSpeed(motorStraightPC);
						rightMotor.setSpeed(0);
						leftMotor.forward();
						rightMotor.forward();

					}
					break;
			}

			// The termination condition for the obstacle avoidance is theta.
			// When the theta is roughly around the opposite of where it started, then we know
			// that the robot is more or less around the object and we can resume trajectory.
			double currentTheta = odometer.getTheta();
			if ((finalTheta - tolFinalTheta < currentTheta) && (currentTheta < tolFinalTheta + finalTheta)) {
				// If it has, then break;
				// Return ultrasonic sensor to forward position.
				ultrasonicSensorMotor.rotate(45, false);
				// Set safe to true, to signal that obstacle issue has been resolved.
				safe = true;
				return;
			}

			try {
				Thread.sleep(50);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}

	}

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
