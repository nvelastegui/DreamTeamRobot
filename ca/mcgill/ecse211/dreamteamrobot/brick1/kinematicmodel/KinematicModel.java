package ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel;

import java.util.HashMap;

/**
 * Class holding constants for kinematic model of robot.
 */
public class KinematicModel {

    /** Current Round Data */
    public static HashMap<String, Integer> roundData;

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
    public static final int ultrasonicLocalizationRotateSpeedFast = 150;
    public static final double d = 20;
    public static final double k = 6;
    public static final double pDTolerance = 50;
    public static final double tolDistanceToWall = 4;
    public static final double thetaCompensation = 0.00; // amount to add to final theta value to compensate for error.


    /** Light Localization Constants
     *
     */
    public static final int    lightLocalization_forwardSpeed = 50;
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
    public static final double navigator_tolThetaHigh = 0.15;
    public static final double navigator_tolEuclideanDistance = 1.0;
    public static final int    navigator_obstacleDistanceTolerance = 15;
    public static final int    navigator_forwardSpeed = 150;
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

    /** Wifi Connection Constants
     *  Constants related to wifi connection.
     */
    public static final String WIFI_SERVER_IP = "192.168.0.101"; //"localhost";
    public static final int    TEAM_NUMBER = 12;

    /**
     * Communication Constants
     */
    public static final int BRICK_PORT = 8080;
    public static final int COMP_PORT = 8081;

    //public static final String BRICK1_HOST = "10.0.1.1";
    //public static final String BRICK2_HOST = "10.0.1.4";
    public static final String BRICK1_HOST = "192.168.43.67";
    public static final String BRICK2_HOST = "192.168.43.111";
    public static final String COMP_HOST = "192.168.43.104";

    public static final int BRICK_TIMEOUT = 10000;      // 10 seconds

    public static final String ROUTE_PROPERTY = "route";
    public enum ROUTES{HEARTBEAT_B1, CLAWS_CLOSED, CLAWS_MOVE, CLAWS_MOVED, READ_BALL, BALL_COLOUR, EXECUTE_SHOOT, FINISHED_SHOOT, NAV_TO, TURN_TO, UPDATE_ODO};

    /**
     * Gripper and Shooter Brick1 Constants
     */
    public static final double GRIPPER_CLOSED_ANGLE = 0.0;
    public static final double GRIPPER_OPEN_ANGLE = 180.0;

    /** Constants: Clasp Motor */
    public static final int CLASP_MOTOR_ROTATION_ANGLE_FOR_OPEN_OR_CLOSE = 180; // Angle of rotation required to open or close
    // the clasp exactly ONCE.
    public static final int CLASP_MOTOR_SPEED = 200;
    public static final int CLASP_MOTOR_ACCELERATION = 3000;

    /** Constants: Shooting */
    public static final int SHOOTER_ROTATION_ANGLE = 90; // in degrees
    public static final int SHOOTER_ARM_ROTATION_SPEED = 900; // in deg/sec -> theoretical maximum of 960 deg/s.
    public static final int SHOOTER_ARM_ACCELERATION   = 3000; // Used the same as previously used.

    /** Constants: Shooting Arm Localization */
    // Assuming the resting position for the arm is zero
    // degrees, this is the angle of the arm in the top
    // position.
    public static final int SHOOTER_ARM_TOP_POSITION_ANGLE = 120;
    public static final int SHOOTER_ARM_LOCALIZATION_ACCELERATION = 3000;
    public static final int SHOOTER_ARM_LOCALIZATION_SPEED = 250;

    /**
     * Loader Constants (approach distance, etc)
     */
    public static final double APPROACH_DISTANCE = 30.0;
    public static final double GRAB_DISTANCE = 0.5;
    public static final double[] SHOOTING_POS = {5*30, 7*30};
    public static final int SHOOTING_HEADING = 90;
    public static final int THROWAWAY_HEADING = 270;


    /**
     * Ball Color constants
     */
    public enum BALL_COLORS{RED, BLUE, ANY};
    public static final double[] BLUE_RGB = {0};
    public static final double[] RED_RGB = {0.5};
}
