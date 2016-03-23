package ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel;

/**
 * Class holding constants for kinematic model of robot.
 */
public class KinematicModel {

    /** Robot Physical Constants */
    public static final double WHEEL_RADIUS_L = 2.01;
    public static final double WHEEL_RADIUS_R = 2.01;
    public static final double WHEELBASE = 13.00;
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
    public static final double k = 2;
    public static final double pDTolerance = 50;
    public static final double tolDistanceToWall = 4;
    public static final double thetaCompensation = 0.09; // amount to add to final theta value to compensate for error.



    /** Odometry Correction
     *
     */
    public static final double COLOUR_SENSOR_SPACING = 8.0;
}
