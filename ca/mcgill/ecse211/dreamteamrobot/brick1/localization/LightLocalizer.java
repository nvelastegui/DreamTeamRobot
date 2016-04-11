package ca.mcgill.ecse211.dreamteamrobot.brick1.localization;

import ca.mcgill.ecse211.dreamteamrobot.brick1.display.LCDDisplay;
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

	enum firstSensorToHit {NONE, LEFT, RIGHT}

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
	 * Corrects theta and brings robot to relative (0,0,0). Relies upon correction in odometerCorrection.
	 */
	public void doLocalization() {

		LCDDisplay.sendToDisplay("Exec: LightLocalization", true);

		// Drive forward until both sensors have gone over a line.
		boolean leftSensorHasSeenLine = false;
		boolean rightSensorHasSeenLine = false;
		firstSensorToHit firstSensor = firstSensorToHit.NONE;

		// Parameters Needed For Calculating Theta Correction
		int leftSensorSeesLine_LeftTachoCount = 0;
		int leftSensorSeesLine_RightTachoCount = 0;
		int rightSensorSeesLine_LeftTachoCount = 0;
		int rightSensorSeesLine_RightTachoCount = 0;

		// Set appropriate speed and set wheels to start moving.
		leftMotor.setSpeed(KinematicModel.lightLocalization_forwardSpeed);
		rightMotor.setSpeed(KinematicModel.lightLocalization_forwardSpeed);
		leftMotor.forward();
		rightMotor.forward();
		while (true) {

			double leftVal = colourPollerLeft.getSensorValue();
			double rightVal = colourPollerRight.getSensorValue();

			if (!leftSensorHasSeenLine) if (leftVal < KinematicModel.lightLocalization_lineThreshold) {
				leftSensorSeesLine_LeftTachoCount = leftMotor.getTachoCount();
				leftSensorSeesLine_RightTachoCount = rightMotor.getTachoCount();
				Sound.twoBeeps();
				leftSensorHasSeenLine = true;
				if (firstSensor == firstSensorToHit.NONE) firstSensor = firstSensorToHit.LEFT;
			}
			if (!rightSensorHasSeenLine) if (rightVal < KinematicModel.lightLocalization_lineThreshold) {
				rightSensorSeesLine_LeftTachoCount = leftMotor.getTachoCount();
				rightSensorSeesLine_RightTachoCount = rightMotor.getTachoCount();
				Sound.twoBeeps();
				rightSensorHasSeenLine = true;
				if (firstSensor == firstSensorToHit.NONE) firstSensor = firstSensorToHit.RIGHT;
			}

			if (leftSensorHasSeenLine && rightSensorHasSeenLine) {
				leftMotor.stop();
				rightMotor.stop();
				break;
			}

		}

		// Do theta correction
		switch (firstSensor) {

			case LEFT:
				// In the case that we crossed the line from the South West direction, calculate the angle between
				// wheelbase and the line and set theta to the appropriate angle as a result of the calculation.
				int deltaTachoRight = Math.abs(leftSensorSeesLine_RightTachoCount - rightSensorSeesLine_RightTachoCount);
				odo.setTheta(
						Math.atan((((double)deltaTachoRight/360)*Math.PI*2*KinematicModel.WHEEL_RADIUS_L)/KinematicModel.lightLocalization_colourSensorSeparation)
				);
				System.out.println("Passed Left First - Theta: " + odo.getTheta());
				// If we're already roughly straight, then just keep theta at 0.00.
				if (deltaTachoRight < 3) odo.setTheta(0.00);
				break;

			case RIGHT:
				// In the case that we crossed the line from the South East direction, calculate the angle between
				// wheelbase and the line and set theta to the appropriate angle as a result of the calculation.
				int deltaTachoLeft = Math.abs(leftSensorSeesLine_LeftTachoCount - rightSensorSeesLine_LeftTachoCount);
				odo.setTheta(
						2.0*Math.PI - Math.atan((((double)deltaTachoLeft/360)*Math.PI*2*KinematicModel.WHEEL_RADIUS_L)/KinematicModel.lightLocalization_colourSensorSeparation)
				);
				System.out.println("Passed Right First - Theta: " + odo.getTheta());
				// If we're already roughly straight, then just keep theta at 0.00.
				if (deltaTachoLeft < 3) odo.setTheta(0.00);
				break;

		}

		System.out.println("Final Theta: " + odo.getTheta());
		//nav.turnToAngle(0.00);

		// Turn to 180 degrees.
		nav.turnToAngle(Math.PI);
		// Go in reverse for a bit to make sure line is IN FRONT of robot.
		leftMotor.backward();
		rightMotor.backward();
		pause(2500);
		rightMotor.stop();
		leftMotor.stop();


		// !!!!!! >>>>> EVERYTHING ABOVE THIS IS WORKING <<<<<<<<<<< !!!!!!!!!

		// Drive until robot sees line.
		leftMotor.setSpeed(KinematicModel.lightLocalization_backwardSpeed);
		rightMotor.setSpeed(KinematicModel.lightLocalization_backwardSpeed);
		leftMotor.forward();
		rightMotor.forward();

		// Robot is *theoretically* straight at this point, so check for "first"
		// sensor to detect line.
		while (true) {
			if ((colourPollerLeft.getSensorValue() < KinematicModel.lightLocalization_lineThreshold) || (colourPollerRight.getSensorValue() < KinematicModel.lightLocalization_lineThreshold)) {
				odo.setY(KinematicModel.lightLocalization_colourSensorOffsetWheelBase);
				Sound.twoBeeps();
				pause(4000); // Let robot go past line a little.
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
		}

		// With y-coordinate and theta known, we just turn to the right and drive until we
		// see a black line. That's x = 0!
		nav.turnToAngle(Math.PI/2.0);

		leftMotor.setSpeed(KinematicModel.lightLocalization_forwardSpeed);
		rightMotor.setSpeed(KinematicModel.lightLocalization_forwardSpeed);
		leftMotor.forward();
		rightMotor.forward();
		while (true) {
			if ((colourPollerRight.getSensorValue() < KinematicModel.lightLocalization_lineThreshold)) {
				// Only checking right sensor since left sensor is typically too close to intersection.
				odo.setX(-KinematicModel.lightLocalization_colourSensorOffsetWheelBase);
				Sound.twoBeeps();
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
		}

		// All values calibrated. Go to (0,0).
		nav.travelTo(0.00, 0.00);
		while(nav.isNavigating()){
			pause(50);
		}
		nav.turnToAngle(0.00);

		LCDDisplay.sendToDisplay("Compl: LightLocalization", true);

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







