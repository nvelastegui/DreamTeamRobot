package ca.mcgill.ecse211.dreamteamrobot.brick2;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.internal.io.SystemSettings;

public class Navigator extends Thread {

//	/** Constants */
//	private static final int FORWARD_SPEED = 100;
//	private static final int ROTATE_SPEED = 80;
//	private static boolean[] getAllValues = {true, true, true};
//
//	private Odometer odometer;
//	private UltrasonicPoller usPoller;
//	private boolean status;
//	enum State {INIT, TURNING, TRAVELLING, EMERGENCY};
//
//	/** Variables: motors */
//	private EV3LargeRegulatedMotor leftMotor = Heart.leftMotor;
//	private EV3LargeRegulatedMotor rightMotor = Heart.rightMotor;
//
//	/** Variables: positions */
//	private Location currentDestination;
//	private double destinationAngle;
//	private double[] currentPosition = new double[3];
//
//	/** Tolerances */
//	private static double tolTheta = 0.04; // 0.06
//	private static double tolEuclideanDistance = 5.0;
//	private static int    tolCloseness = 15;
//
//	/**
//	 * Constructor
//	 * @param odometer Robot odometer.
//	 * @param usPoller Robot ultrasonic poller used for checkEmergency()
//	 */
//	public Navigator(Odometer odometer, UltrasonicPoller usPoller) {
//		this.odometer = odometer;
//		this.usPoller = usPoller;
//		this.status = false;
//	}
//
//	/**
//	 * @return Navigator's odometer.
//	 */
//	public Odometer getOdometer() {
//		return odometer;
//	}
//
//	/**
//	 * @return Navigator's ultrasonic poller.
//	 */
//	public UltrasonicPoller getUsPoller() {
//		return usPoller;
//	}
//
//	/**
//	 * @return Navigator's current destination.
//	 */
//	public Location getCurrentDestination() {
//		return currentDestination;
//	}
//
//	public void travelTo (double x, double y) {
//		//System.out.println("run travelTo");
//
//		// Set the destination.
//		currentDestination = new Location(x, y);
//
//		// Set the angle required to reach destination.
//		destinationAngle = getDestinationAngle();
//
//		// Set travel status to true.
//		status = true;
//	}
//
//	/**
//	 * @return The angle at which the robot must be to reach the destination by
//	 * driving in a straight forward path.
//	 */
//	private double getDestinationAngle() {
//		//System.out.println("run getDestinationAngle");
//
//		// Get current position stuff
//		odometer.getPosition(currentPosition, getAllValues);
//		double currentX = currentPosition[0];
//		double currentY = currentPosition[1];
//
//		// Get current destination stuff
//		double destinationX = currentDestination.getX();
//		double destinationY = currentDestination.getY();
//
//		// Calculate differences
//		double deltaX = destinationX - currentX;
//		double deltaY = destinationY - currentY;
//
//		// Determine destination angle
//		// (relative to positive y axis, ie. what the robot navigates in)
//		// We are using the atan2 function with the deltaX and deltaY in
//		// the opposite parameters to achieve this.
//		double number = Math.atan2(deltaX, deltaY);
//
//		if (number < 0) {
//			number = 2.0*Math.PI + number;
//		}
//
//		return number;
//
//	}
//
//	/**
//	 * Turns robot to destination angle using minimal angle.
//	 * @param desiredAngle
//	 */
//	private void turnTo (double desiredAngle) {
//
//		// Grab the current angle and determine the
//		// heading error based on desiredAngle
//		double currentAngle = odometer.getTheta();
//		double headingError = desiredAngle - currentAngle;
//
//		// Filter headingError so that minimum angle is used.
//		if (headingError < -Math.PI) {
//			// Error is below -180.
//			headingError = headingError + 2*Math.PI;
//		}
//		else if (headingError > Math.PI) {
//			// Error is above +180.
//			headingError = headingError - 2*Math.PI;
//		}
//		else {
//			// Error is within (-180,180)
//			// Already minimum angle.
//		}
//
//		// With the heading error rightly identified,
//		// we need to rotate to the required angle.
//		leftMotor.setSpeed(ROTATE_SPEED);
//		rightMotor.setSpeed(ROTATE_SPEED);
//		if (headingError >= 0.0) {
//			leftMotor.forward();
//			rightMotor.backward();
//		}
//		else {
//			leftMotor.backward();
//			rightMotor.forward();
//		}
//
//	}
//
//	/**
//	 * @param destinationAngle Required angle for robot to be at to reach destination.
//	 * @return True if robot is at proper angle. False if robot is not.
//	 */
//	private boolean isFacingDestination (double destinationAngle) {
//		double currentTheta = odometer.getTheta();
//		return ((destinationAngle - tolTheta) < currentTheta) && (currentTheta < (destinationAngle + tolTheta));
//	}
//
//	/**
//	 * @return Location object with current position of robot.
//	 */
//	public Location getPositionFromOdometer () {
//		odometer.getPosition(currentPosition, getAllValues);
//		return new Location(currentPosition[0], currentPosition[1]);
//	}
//
//	/**
//	 * @param currentLocation
//	 * @return True if at current destination, within tolerance. False if not at current destination.
//	 */
//	private boolean checkIfAtDestination (Location currentLocation) {
//		// TODO: Do this with vectors instead of Euclidean distance?
//
//		// Calculate euclidean distance
//		double distance = Math.sqrt(
//				Math.pow(currentDestination.getX() - currentLocation.getX(), 2.0)
//						+ Math.pow(currentDestination.getY() - currentLocation.getY(), 2.0)
//			);
//
//		return (distance < tolEuclideanDistance);
//	}
//
//	/**
//	 * Updates wheels to ensure they are moving forward at FORWARD_SPEED.
//	 */
//	private void updateTravel () {
//		// Ensure speeds are set to FORWARD_SPEED.
//		leftMotor.setSpeed(FORWARD_SPEED);
//		rightMotor.setSpeed(FORWARD_SPEED);
//		// Ensure wheels are moving forward.
//		leftMotor.forward();
//		rightMotor.forward();
//	}
//
//	/**
//	 * @return True if robot is navigating. False if it is not.
//	 */
//	public boolean isNavigating () {
//		return status;
//	}
//
//	/**
//	 * @return True if in state of emergency. False if not.
//	 */
//	private boolean checkEmergency() {
//		return usPoller.getDistance() < tolCloseness;
//	}
//
//	public static int convertDistance(double radius, double distance) {
//		return (int) ((180.0 * distance) / (Math.PI * radius));
//	}
//
//	public static int convertAngle(double radius, double width, double angle) {
//		return convertDistance(radius, Math.PI * width * angle / 360.0);
//	}
//
//	/**
//	 * This is the main run method for the driver. Structure is heavily based on material
//	 * from threadTutorial.pdf (very useful document).
//	 */
//	@Override
//	public void run() {
//
//		State state = State.INIT;
//		ObstacleAvoidance avoidance = new ObstacleAvoidance(this);
//
//		while (true) {
////			System.out.println("State: " + state);
//
//			switch (state) {
//				/** */
//				case INIT:
//					if (status) {
//						state = State.TURNING;
//					}
//					break;
//				/** */
//				case TURNING:
//					// First order of business is to turn to the required angle.
//					// This method returns only when the motion is complete.
//					turnTo(destinationAngle);
//					// Then if the angle is correct, set the state to TRAVELLING.
//					if (isFacingDestination(destinationAngle)) {
//						state = State.TRAVELLING;
//					}
//					break;
//				/** */
//				case TRAVELLING:
//					Location currentPosition = getPositionFromOdometer();
//					if (checkEmergency()) {
//						state = State.EMERGENCY;
//						avoidance = new ObstacleAvoidance(this);
//						avoidance.start();
//					}
//					else if (!checkIfAtDestination(currentPosition)) {
//						updateTravel();
//					}
//					else {
//						leftMotor.stop();
//						rightMotor.stop();
//						// Operation complete. Set navigating status to false
//						// and reset state to INIT.
//						status = false;
//						state = State.INIT;
//					}
//					break;
//				case EMERGENCY:
//					if (avoidance.isResolved()) {
//						state = State.TURNING;
//						// Reset destination angle...
//						destinationAngle = getDestinationAngle();
//					}
//					break;
//				default:
//					leftMotor.stop();
//					rightMotor.stop();
//					break;
//			}
//
//			try {
//				Thread.sleep(30);
//			}
//			catch (InterruptedException e) {
//				e.printStackTrace();
//			}
//
//		}
//
//	}

}