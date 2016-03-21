package ca.mcgill.ecse211.dreamteamrobot.brick1.localization;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Performs final localization assuming robot forward and sideways axes are approximately
 * perpendicular to corner walls. Moves robot to relative (0,0) and faces forward.
 */
public class LightLocalizer {

	private static int FORWARD_SPEED = 30;
	private static int ROTATE_SPEED = 30;
	private static double lengthToLightSensor = 12.00; // 8.00

	// Instance Variables
	private Odometer odo;
	private Navigator nav;
	private ColourPoller colourPollerLeft;
	private ColourPoller colourPollerRight;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	/**
	 * Constructor.
	 * @param leftMotor Left wheel motor.
	 * @param rightMotor Right wheel motor.
	 * @param odo Odometer.
	 * @param nav Navigator.
     * @param colourPollerLeft Thread poller for left colour sensor.
	 * @param colourPollerRight Thread poller for right colour sensor.
     */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odo, Navigator nav, ColourPoller colourPollerLeft, ColourPoller colourPollerRight) {
		this.nav = nav;
		this.odo = odo;
		this.colourPollerLeft = colourPollerLeft;
		this.colourPollerRight = colourPollerRight;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	/**
	 * Corrects theta and brings robot to relative (0,0,0).
	 */
	public void doLocalization() {

		// Set odometer values to (-30, -30)
		odo.setX(-30.00);
		odo.setY(-30.00);

		// Drive up to cross y = 0 line
		// assume correction by odometry correction
		nav.travelTo(-30.00, 10.00);

		// Move back to vertical middle of block
		nav.travelTo(-30.00, -15.00);

		//
		nav.travelTo(10, -15.00);

		//
		nav.travelTo(0,0);

		// Move forward




		// First we need to bring the robot somewhere near the intersection (0,0).
		getNearTopRightIntersection();

		// Now we can proceed with the light localization process.
		// Rotate in a circle going right, marking the theta for each line crossed.
		double firstTheta = rotateRightUntilLine();
		Sound.beep();
		leftMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 15), true);    // Return immediately
		rightMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 15), false); // Return when motion is completed.
		double secondTheta = rotateRightUntilLine();
		Sound.beep();
		leftMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 15), true);    // Return immediately
		rightMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 15), false); // Return when motion is completed.
		double thirdTheta = rotateRightUntilLine();
		Sound.beep();
		leftMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 15), true);    // Return immediately
		rightMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 15), false); // Return when motion is completed.
		double fourthTheta = rotateRightUntilLine();
		Sound.beep();

		// Calculate offsets from x-axis and y-axis.
		double deltaY = thirdTheta - firstTheta;
		double offsetX = (-1.0) * (lengthToLightSensor) * Math.cos(deltaY/2.0);

		double deltaX = secondTheta + (2*Math.PI - fourthTheta);
		double offsetY = (-1.0) * (lengthToLightSensor) * Math.cos(deltaX/2.0);

		// Set the odometer with new x and y values.
		odo.setX(offsetX);
		odo.setY(offsetY);

		// Travel to (0,0)
		nav.travelTo(0.0,0.0);
		while (nav.isNavigating()) {
			pause(500);
		}

		// Experimentally determined average offset - correct for it.
		odo.setX(odo.getX() + 3.00);
		odo.setY(odo.getY() + 2.00);

		// Turn to 0 degrees.
		nav.turnToAngle(0.0);
		while (nav.isNavigating()) {
			pause(500);
		}

	}

	/**
	 * Rotates the robot right until a line is detected.
	 */
	private double rotateRightUntilLine () {

		// Turn right until a line is crossed.
		while (!isHoveringOverLine()) {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.forward();
			rightMotor.backward();
		}

		leftMotor.stop();
		rightMotor.stop();

		return odo.getTheta();

	}

	/**
	 * Assuming that the robot has been approximately oriented using
	 * ultrasonic localization, this procedure moves it somewhat close
	 * to the top right intersection of the block it's in.
	 */
	private void getNearTopRightIntersection () {

		// Wait for a button press to move to next procedure.
		Button.waitForAnyPress();

		// Move forward SLOWLY until a line is crossed.
		while (!isHoveringOverLine()) {
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();
		}

		leftMotor.stop();
		rightMotor.stop();

		// Move Forward a little.
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 10), true);    // Return immediately
		rightMotor.rotate(-convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 10), false); // Return when motion is completed.

		// Turn to horizontal.
		nav.turnToAngle(Math.PI/2.0);

		// Move forward SLOWLY until a line is crossed.
		while (!isHoveringOverLine()) {
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();
		}

		leftMotor.stop();
		rightMotor.stop();

		// Move forward a little.
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 25), true);    // Return immediately
		rightMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 25), false); // Return when motion is completed.

		// Turn back to vertical.
		nav.turnToAngle(0.0);

		// Move forward a little.
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_L, KinematicModel.WHEELBASE, 40), true);    // Return immediately
		rightMotor.rotate(convertAngle(KinematicModel.WHEEL_RADIUS_R, KinematicModel.WHEELBASE, 40), false); // Return when motion is completed.


	}

	/**
	 * @return True if robot is hovering over line. False if not.
	 */
	private boolean isHoveringOverLine () {
		double sensorValueLeft = colourPollerLeft.getSensorValue();
		double sensorValueRight = colourPollerRight.getSensorValue();
		return (sensorValueLeft < 50.00) || (sensorValueRight < 50.00);
	}


	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
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
			Sound.beep();
			Sound.twoBeeps();
		}
	}

}







