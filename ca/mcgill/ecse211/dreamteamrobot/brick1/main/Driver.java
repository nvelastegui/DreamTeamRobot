package ca.mcgill.ecse211.dreamteamrobot.brick1.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.communication.DriverStatusPacket;
import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Location;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.OdometerCorrection;
import ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding.Graph;
import ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding.PathFinder;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;

import java.util.List;

/**
 * This is the main offensive thread for the navigational brick. It runs as a state machine.
 */
public class Driver extends Thread {

    /** Constants: State */
    enum State {OFF, INIT, IDLE, DRIVING_TO_BALLS, LOADING, DRIVING_TO_SHOOT, SHOOTING};
    private State state;

    /** Variables: Sub Threads */
    private Odometer odometer;
    private OdometerCorrection odometerCorrection;
    private UltrasonicPoller ultrasonicPollerLeft;
    private UltrasonicPoller ultrasonicPollerRight;
    private ColourPoller colourPollerLeft;
    private ColourPoller colourPollerRight;
    private Navigator navigator;

    /**
     * Constructor.
     * @param leftMotor motor for left wheel
     * @param rightMotor motor for right wheel
     * @param leftUltrasonicSensorMotor motor for left ultrasonic sensor's rotation about vertical axis
     * @param rightUltrasonicSensorMotor motor for right ultrasonic sensor's rotation about vertical axis
     * @param ultrasonicSensorPortLeft port for left ultrasonic sensor
     * @param ultrasonicSensorPortRight port for right ultrasonic sensor
     */
    public Driver (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftUltrasonicSensorMotor, EV3LargeRegulatedMotor rightUltrasonicSensorMotor, Port ultrasonicSensorPortLeft, Port ultrasonicSensorPortRight, Port colourSensorLeftPort, Port colourSensorRightPort) {

        // Create thread instances.
        this.odometer = new Odometer(leftMotor, rightMotor);
        this.colourPollerLeft = new ColourPoller(colourSensorLeftPort);
        this.colourPollerRight = new ColourPoller(colourSensorRightPort);
        this.odometerCorrection = new OdometerCorrection(odometer, colourPollerLeft, colourPollerRight);
        this.ultrasonicPollerLeft = new UltrasonicPoller(ultrasonicSensorPortLeft);
        this.ultrasonicPollerRight = new UltrasonicPoller(ultrasonicSensorPortRight);
        this.navigator = new Navigator(odometer, ultrasonicPollerLeft, ultrasonicPollerRight, leftMotor, rightMotor, leftUltrasonicSensorMotor, rightUltrasonicSensorMotor);

        // Set state to OFF.
        state = State.OFF;
    }

    /**
     * @return Current status of driver, for display on LCD.
     */
    public DriverStatusPacket getStatus () {

        return new DriverStatusPacket(
                state.name(),
                odometer.getX(),
                odometer.getY(),
                odometer.getTheta()
        );

    }

    public Odometer getOdometer () {
        return odometer;
    }

    public OdometerCorrection getOdometerCorrection() {
        return odometerCorrection;
    }

    public Navigator getNavigator() {
        return navigator;
    }

    public UltrasonicPoller getUltrasonicPollerLeft() {
        return ultrasonicPollerLeft;
    }

    public UltrasonicPoller getUltrasonicPollerRight() {
        return ultrasonicPollerRight;
    }

    public ColourPoller getColourPollerLeft() {
        return colourPollerLeft;
    }

    public ColourPoller getColourPollerRight() {
        return colourPollerRight;
    }

    /**
     * Set up sub threads before running.
     */
    public void performPreExecute () {

        // Start odometer.
        odometer.start();

        // Start colour pollers.
        colourPollerLeft.start();
        colourPollerRight.start();

        // Don't start odometry correction yet.

        // Start ultrasonic pollers.
        ultrasonicPollerLeft.start();
        ultrasonicPollerRight.start();

        // Start navigator, in no-obstacle-avoidance mode.
        navigator.start();

    }

    /**
     * Runs odometry correction.
     */
    public void startOdometryCorrection () {
        odometerCorrection.start();
    }

    /**
     * Sets state to INIT and starts thread.
     */
    public void turnOn () {
        state = State.INIT;
        PathFinder.board = new Graph(12*12);
        PathFinder.setupFullyConnectedBoard();
        start();
    }

    /**
     * Thread Run.
     * This is the main run method for the driver. Structure is heavily based on material
     * from threadTutorial.pdf and code from Lab 3.
     */
    @Override
    public void run() {

        navigator.setObstacleAvoidanceOn();

        while (true) {

            switch (state) {

                /** CASE: INIT
                 *  Checks round data to determine starting point. Drives there.
                 */
                case INIT:

                    // Localize robot relative to board, given starting corner.
                    // 1: BL, 2: BR, 3: TR, 4: TL
                    int startingCorner = KinematicModel.roundData.get("SC");
                    switch (startingCorner) {
                        case 1:
                            // Do nothing.
                            // Localization was done assuming robot started in this corner.
                            break;
                        case 2:
                            // Set x and y.
                            odometer.setX(300.00);
                            odometer.setY(0.00);
                            // Set theta.
                            odometer.setTheta(3 * Math.PI / 2); // robot currently facing West
                            break;
                        case 3:
                            // Set x and y.
                            odometer.setX(300.00);
                            odometer.setY(300.00);
                            // Set theta.
                            odometer.setTheta(Math.PI); // robot currently facing South
                            break;
                        case 4:
                            // Set x and y.
                            odometer.setX(0.00);
                            odometer.setY(300.00);
                            // Set theta.
                            odometer.setTheta(Math.PI / 2); // robot currently facing East
                            break;
                    }


                    // Process roundData to determine initial obstacles (defender zone, ball zone)
                    PathFinder.blockOutDefenseZone();
                    PathFinder.blockOutBallBox();

                    // Generate path to offensive zone.
                    List<Location> pathToOffensiveZone = PathFinder.generatePath(
                            new Location(odometer.getX(), odometer.getY()),
                            new Location(135.00, 15.00)
                    );

                    // Drive to offense zone. (standard: to block 17)
                    for (Location loc : pathToOffensiveZone) {
                        navigator.travelTo(loc.getX(), loc.getY());
                        while (navigator.isNavigating()) {
                            try {Thread.sleep(30);}
                            catch (InterruptedException e) {e.printStackTrace();}
                        }
                    }

                    // Once located at offensive zone, switch status to idle.
                    state = State.IDLE;
                    break;

                /** CASE: IDLE
                 *  Waits for signal to go!
                 */
                case IDLE:
                    state = State.DRIVING_TO_BALLS;
                    break;

                case DRIVING_TO_BALLS:
                    
                    break;

                case LOADING:
                    break;

                case DRIVING_TO_SHOOT:
                    break;

                case SHOOTING:
                    break;

                default:
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
