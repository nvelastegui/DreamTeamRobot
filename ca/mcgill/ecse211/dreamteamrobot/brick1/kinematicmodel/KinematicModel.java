package ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel;

/**
 * Class holding constants for kinematic model of robot.
 */
public class KinematicModel {

    /** Robot Physical Constants */
    public static final double WHEEL_RADIUS_L = 1.95; // Might change if we increase/reduce weight of the robot.
    public static final double WHEEL_RADIUS_R = 1.95;
    public static final double WHEELBASE = 14.25; // DON'T CHANGE THIS.
    public static final double COLOR_SENSOR_FORWARD_OFFSET = 3.5;


    /** Ultrasonic Localization Constants
     *
     *  Ultrasonic localization works like in the labs (ie. that angleA/angleB stuff.) Except, since we have two ultrasonic
     *  sensors, it does things a bit different. It rotates right until it's facing the wall roughly straight on: (1) difference
     *  in values read by ultrasonic sensors is below tolDistanceToWall and (2) the average value read by both sensors is below
     *  d. Note that our method also incorporates the whole band of k distance above and below d (see lab tutorial).
     *
     *  thetaCompensation is the constant value added to theta at the end (to correct for systematic error - ie. consistent errors of x amount).
     */
    public static final int ultrasonicLocalizationRotateSpeed = 50;
    public static final double d = 20;
    public static final double k = 6;
    public static final double pDTolerance = 50;
    public static final double tolDistanceToWall = 4;
    public static final double thetaCompensation = 0.00; // amount to add to final theta value to compensate for error.


    /** Light Localization Constants
     *
     */
    public static final int    lightLocalization_forwardSpeed = 80;
    public static final int    lightLocalization_backwardSpeed = 60;
    public static final double lightLocalization_colourSensorSeparation = 8.00;
    public static final int    lightLocalization_lineThreshold = 40;
    public static final double lightLocalization_colourSensorOffsetWheelBase = 3.5;

    /** Navigator Constants
     *
     *  Constants related to navigator. There are two tolerances on the theta. The tight tolerance is too low for
     *  the odometry correction, but it's perfect for the ultrasonic localization. So during the first half of the localization
     *  the low tolerance is used (makes rotations at low speed MUCH more accurate), and for the light localization
     *  it's pumped up to 0.16.
     *
     */
    public static final double navigator_tolThetaLow = 0.06;
    public static final double navigator_tolThetaHigh = 0.16;
    public static final double navigator_tolEuclideanDistance = 1.0;
    public static final int    navigator_obstacleDistanceTolerance = 15;
    public static final int    navigator_forwardSpeed = 100;
    public static final int    navigator_rotateSpeed = 70;

    /** Obstacle Avoider Constants
     *
     */
    public static final int    obstacleAvoider_tolCloseness = 15;
    public static final int    obstacleAvoider_ultrasonicSensorRotationSpeedLow = 20;

    /** Odometry Correction
     *
     */
    public static final double COLOUR_SENSOR_SPACING = 8.0;
}
