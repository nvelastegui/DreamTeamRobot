package ca.mcgill.ecse211.dreamteamrobot.brick1.navigation;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Navigator is a state machine running on a thread. It manages movement of the robot in the horizontal plane,
 * watching for obstacles and driving around them.
 */
public class Navigator extends Thread {

	/** Constants */
	private static final int FORWARD_SPEED = KinematicModel.navigator_forwardSpeed;
	private static final int ROTATE_SPEED = KinematicModel.navigator_rotateSpeed;
	private static boolean[] getAllValues = {true, true, true};

	/** Variables: Sub Threads */
	private Odometer odometer;
	private UltrasonicPoller usPollerLeft;
	private UltrasonicPoller usPollerRight;
	private ObstacleAvoider obstacleAvoider;

	/** Variables: Status */
	enum State {INIT, TURNING, TRAVELLING, EMERGENCY};
	enum StateObstacleAvoidance {OFF, ON};
	private boolean status;
	private State navState;
	private StateObstacleAvoidance stateOA;

	/** Variables: Motors */
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftUltrasonicSensorMotor;
	private EV3LargeRegulatedMotor rightUltrasonicSensorMotor;

	/** Variables: Positions */
	private Location currentDestination;
	private double destinationAngle;
	private double[] currentPosition = new double[3];

	/** Tolerances */
	private double tolTheta;
	private static double tolEuclideanDistance = KinematicModel.navigator_tolEuclideanDistance;
	private static int    tolCloseness = KinematicModel.navigator_obstacleDistanceTolerance;
	private static double wallDetectionTolerance = 10.0;

	/**
	 * Constructor.
	 * @param odometer Robot odometer.
	 * @param usPollerLeft Left ultrasonic poller.
	 * @param usPollerRight Right ultrasonic poller.
	 * @param leftMotor Left motor.
	 * @param rightMotor Right motor.
	 * @param leftUltrasonicSensorMotor Left ultrasonic sensor motor.
	 * @param rightUltrasonicSensorMotor Right ultrasonic sensor motor.
     */
	public Navigator(Odometer odometer, UltrasonicPoller usPollerLeft, UltrasonicPoller usPollerRight, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftUltrasonicSensorMotor, EV3LargeRegulatedMotor rightUltrasonicSensorMotor) {
		this.odometer = odometer;
		this.usPollerLeft = usPollerLeft;
		this.usPollerRight = usPollerRight;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftUltrasonicSensorMotor = leftUltrasonicSensorMotor;
		this.rightUltrasonicSensorMotor = rightUltrasonicSensorMotor;
		this.status = false;

		// Set obstacle avoidance to off initially.
		this.stateOA = StateObstacleAvoidance.OFF;

		// Set theta tolerance to low initially.
		this.tolTheta = KinematicModel.navigator_tolThetaLow;
	}

	/**
	 * Sets tolerance on theta (for turnToAngle/turnTo/isFacing/etc.) to lower value, as defined
	 * in KinematicModel.
	 */
	public void setThetaToleranceLow () {
		this.tolTheta = KinematicModel.navigator_tolThetaLow;
	}

	/**
	 * Sets tolerance on theta (for turnToAngle/turnTo/isFacing/etc.) to higher value, as defined
	 * in KinematicModel.
	 */
	public void setThetaToleranceHigh () {
		this.tolTheta = KinematicModel.navigator_tolThetaHigh;
	}

	/**
	 * Turns on Obstacle Avoidance.
	 */
	public void setObstacleAvoidanceOn () {
		stateOA = StateObstacleAvoidance.ON;
	}

	/**
	 * @return Navigator's odometer.
	 */
	public Odometer getOdometer() {
		return odometer;
	}

	/**
	 * @return Navigator's ultrasonic poller.
	 */
	public UltrasonicPoller getUsPoller() {
		return usPollerLeft;
	}

	/**
	 * @return Navigator's left ultrasonic poller.
     */
	public UltrasonicPoller getUsPollerLeft() {
		return usPollerLeft;
	}

	/**
	 * @return Navigator's right ultrasonic poller.
     */
	public UltrasonicPoller getUsPollerRight() {
		return usPollerRight;
	}

	/**
	 * @return Navigator's current destination.
	 */
	public Location getCurrentDestination() {
		return currentDestination;
	}

	/**
	 * Sets new travel point for robot.
	 * @param x x coordinate of travel point
	 * @param y y coordinate of travel point
     */
	public void travelTo (double x, double y) {
		//System.out.println("run travelTo");

		// Set the destination.
		currentDestination = new Location(x, y);

		// Set the angle required to reach destination.
		destinationAngle = getDestinationAngle();

		// Set travel status to true.
		status = true;
	}

	/**
	 * Cancels current travel.
	 */
	public void cancelTravel () {
		// do stuff here
		// what if it's currently in an emergency?
	}

	/**
	 * @return The angle at which the robot must be to reach the destination by
	 * driving in a straight forward path.
	 */
	private double getDestinationAngle() {
		//System.out.println("run getDestinationAngle");

		// Get current position stuff
		odometer.getPosition(currentPosition, getAllValues);
		double currentX = currentPosition[0];
		double currentY = currentPosition[1];

		// Get current destination stuff
		double destinationX = currentDestination.getX();
		double destinationY = currentDestination.getY();

		// Calculate differences
		double deltaX = destinationX - currentX;
		double deltaY = destinationY - currentY;

		// Determine destination angle
		// (relative to positive y axis, ie. what the robot navigates in)
		// We are using the atan2 function with the deltaX and deltaY in
		// the opposite parameters to achieve this.
		double number = Math.atan2(deltaX, deltaY);

		if (number < 0) {
			number = 2.0*Math.PI + number;
		}

		return number;

	}

	/**
	 * Turns robot to destination angle using minimal angle.
	 * @param desiredAngle
	 */
	private void turnTo (double desiredAngle) {

		// Grab the current angle and determine the
		// heading error based on desiredAngle
		double currentAngle = odometer.getTheta();
		double headingError = desiredAngle - currentAngle;

		// Filter headingError so that minimum angle is used.
		if (headingError < -Math.PI) {
			// Error is below -180.
			headingError = headingError + 2*Math.PI;
		}
		else if (headingError > Math.PI) {
			// Error is above +180.
			headingError = headingError - 2*Math.PI;
		}
		else {
			// Error is within (-180,180)
			// Already minimum angle.
		}

		// With the heading error rightly identified,
		// we need to rotate to the required angle.
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		if (headingError >= 0.0) {
			leftMotor.forward();
			rightMotor.backward();
		}
		else {
			leftMotor.backward();
			rightMotor.forward();
		}

	}

	/**
	 * Allows external object to turn robot to a specific angle.
	 * @param desiredAngle
	 */
	public boolean turnToAngle (double desiredAngle) {

		// Continue to turn until proper angle is reached.
		while (!isFacingDestination(desiredAngle)) {
			turnTo(desiredAngle);
		}

		// Stop motors and set navigating status to false.
		leftMotor.stop();
		rightMotor.stop();

		// Return a value here to ensure that full method runs
		// before next line is executed in parent call.
		return true;
	}

	/**
	 * @param destinationAngle Required angle for robot to be at to reach destination.
	 * @return True if robot is at proper angle. False if robot is not.
	 */
	private boolean isFacingDestination (double destinationAngle) {
		double currentTheta = odometer.getTheta();
		double err = Math.min(Math.abs(destinationAngle-currentTheta), Math.abs(Math.PI*2-currentTheta - destinationAngle));
		return err < tolTheta;
		//return ((destinationAngle - tolTheta) < currentTheta) && (currentTheta < (destinationAngle + tolTheta));
	}

	/**
	 * @return Location object with current position of robot.
	 */
	public Location getPositionFromOdometer () {
		odometer.getPosition(currentPosition, getAllValues);
		return new Location(currentPosition[0], currentPosition[1]);
	}

	/**
	 * @param currentLocation
	 * @return True if at current destination, within tolerance. False if not at current destination.
	 */
	private boolean checkIfAtDestination (Location currentLocation) {
		// TODO: Do this with vectors instead of Euclidean distance?

		// Calculate euclidean distance
		double distance = Math.sqrt(
				Math.pow(currentDestination.getX() - currentLocation.getX(), 2.0)
						+ Math.pow(currentDestination.getY() - currentLocation.getY(), 2.0)
			);

		return (distance < tolEuclideanDistance);
	}

	/**
	 * Updates wheels to ensure they are moving forward at FORWARD_SPEED.
	 */
	private void updateTravel () {
		// Ensure speeds are set to FORWARD_SPEED.
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		// Ensure wheels are moving forward.
		leftMotor.forward();
		rightMotor.forward();
	}

	/**
	 * @return True if robot is navigating. False if it is not.
	 */
	public boolean isNavigating () {
		return status;
	}

	public String getNavState () {
		return this.navState.toString();
	}
	/**
	 * @return True if in state of emergency. False if not.
	 */
	private boolean checkEmergency() {
		// Return true if either sensor is reading a value below tolerance. (ie. an object might be nearby).
		double leftP = usPollerLeft.getDistance();
		double rightP = usPollerRight.getDistance();
		if((leftP < tolCloseness) || (rightP < tolCloseness)){
			System.out.println("leftP:" + leftP + ", isDetectingWall:"+isDetectingWall(leftP)+" - rightP:"+rightP + ", isDetectingWall:"+isDetectingWall(rightP));
		}
		return ((leftP < tolCloseness) || (rightP < tolCloseness)) && !isDetectingWall(leftP) && !isDetectingWall(rightP);
	}

	private boolean isDetectingWall(double pollerDist){
		double detectionX = this.odometer.getX() + pollerDist * Math.sin(this.odometer.getTheta());
		double detectionY = this.odometer.getY() + pollerDist * Math.cos(this.odometer.getTheta());

		boolean xWall = Math.min(Math.abs(detectionX - (-30)), Math.abs(detectionX - (330))) < wallDetectionTolerance;
		boolean yWall = Math.min(Math.abs(detectionY - (-30)), Math.abs(detectionY - (330))) < wallDetectionTolerance;

		return xWall || yWall;
	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This is the main run method for the driver. Structure is heavily based on material
	 * from threadTutorial.pdf (very useful document).
	 */
	@Override
	public void run() {

		State state = State.INIT;
		ObstacleAvoider avoidance = new ObstacleAvoider(this, leftUltrasonicSensorMotor, rightUltrasonicSensorMotor, leftMotor, rightMotor);

		while (true) {
			this.navState = state;

			switch (state) {
				/** */
				case INIT:
					if (status) {
						state = State.TURNING;
					}
					break;
				/** */
				case TURNING:
					// If robot is facing the right way, set state to travelling.
					if (isFacingDestination(getDestinationAngle())) {
						state = State.TRAVELLING;
					}
					// Otherwise, turn until facing the right way.
					else {
						// This method returns only when the motion is complete.
						turnToAngle(getDestinationAngle());
						state = State.TRAVELLING;
					}
					break;
				/** */
				case TRAVELLING:
					Location currentPosition = getPositionFromOdometer();
					if (checkEmergency()) {
						if (stateOA == StateObstacleAvoidance.ON) {
							leftMotor.stop();
							rightMotor.stop();
							state = State.EMERGENCY;
							avoidance = new ObstacleAvoider(this, leftUltrasonicSensorMotor, rightUltrasonicSensorMotor, leftMotor, rightMotor);
							avoidance.start();
						}
					}
					else if (!checkIfAtDestination(currentPosition)) {
						if (isFacingDestination(getDestinationAngle())) {
							updateTravel();
						} else {
							System.out.println("Not facing destination");
							state = State.TURNING;
						}
					}
					else {
						leftMotor.stop();
						rightMotor.stop();
						// Operation complete. Set navigating status to false
						// and reset state to INIT.
						status = false;
						state = State.INIT;
					}
					break;
				case EMERGENCY:
					if (avoidance.isResolved()) {
						state = State.TURNING;
						// Reset destination angle...
						destinationAngle = getDestinationAngle();
					}
					break;
				default:
					leftMotor.stop();
					rightMotor.stop();
					break;
			}

			try {
				Thread.sleep(30);
			}
			catch (InterruptedException e) {
				e.printStackTrace();
			}

		}

	}

}