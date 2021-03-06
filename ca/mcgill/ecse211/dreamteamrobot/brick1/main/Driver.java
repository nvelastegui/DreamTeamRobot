package ca.mcgill.ecse211.dreamteamrobot.brick1.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.ballloader.BallLoader;
import ca.mcgill.ecse211.dreamteamrobot.brick1.communication.DriverStatusPacket;
import ca.mcgill.ecse211.dreamteamrobot.brick1.display.LCDDisplay;
import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Location;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.OdometerCorrection;
import ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding.Graph;
import ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding.PathFinder;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.UltrasonicPoller;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the main offensive thread for the navigational brick. It runs as a state machine.
 */
public class Driver extends Thread {

    /** Constants: State */
    enum State {OFF, INIT, IDLE, DRIVING_TO_BALLS, LOADING, DRIVING_TO_THROW_AWAY, DRIVING_TO_SHOOT, SHOOTING};
    private State state;

    /** Variables: Sub Threads */
    private Odometer odometer;
    private OdometerCorrection odometerCorrection;
    private UltrasonicPoller ultrasonicPollerLeft;
    private UltrasonicPoller ultrasonicPollerRight;
    private ColourPoller colourPollerLeft;
    private ColourPoller colourPollerRight;
    private Navigator navigator;
    private BallLoader ballLoader;

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
     *
     * @param brick2
     * @param comp
     */
    public void initializeBallLoader (Connection brick2, Connection comp) {
        ballLoader = new BallLoader(brick2, comp, navigator);
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
        // Set the state to the one-time INIT state.
        state = State.INIT;
        // Reset the board to a blank 12 by 12.
        PathFinder.board = new Graph(12*12);
        // Fully connect the board.
        PathFinder.setupFullyConnectedBoard();
        // Ready?! Fight! (cue Mortal Kombat theme)
        start();
    }

    /**
     * Thread Run.
     * This is the main run method for the driver. Structure is heavily based on material
     * from threadTutorial.pdf and code from Lab 3.
     */
    @Override
    public void run() {

        //navigator.setObstacleAvoidanceOn();

        while (true) {

            switch (state) {

                /** CASE: INIT
                 *  DEPENDENCIES: Robot has localized relative to a corner.
                 *  Checks round data to determine starting point. Drives there.
                 */
                case INIT:

                    switch (KinematicModel.roundData.get("Role")) {

                        /** FORWARD */
                        case 0:

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

                            // Process roundData to log initial obstacles
                            PathFinder.blockOutDefenseZone();   // Defense Zone must be blocked off.
                            PathFinder.blockOutBallBox();       // Ball Box must be blocked off.

                            // Generate path to offensive zone. Always move to same point in offensive zone (independent of how tall
                            // the zone is.
                            List<Location> pathToOffensiveZone = null;
                            switch (startingCorner) {
                                case 1:
                                    List<Location> random = PathFinder.generatePath(
                                            new Location(odometer.getX(), odometer.getY()),
                                            new Location(15.00, 15.00)
                                    );
                                    if (random != null) {
                                        List<Location> secondPortion = PathFinder.generatePath(
                                                new Location(15.00, 15.00),
                                                new Location(135.00, 15.00)
                                        );
                                        if (secondPortion != null) random.addAll(secondPortion);
                                        pathToOffensiveZone = new ArrayList<>();
                                        pathToOffensiveZone.addAll(random);
                                    }
                                    break;
                                case 2:
                                    List<Location> initialPointCase2 = PathFinder.generatePath(
                                            new Location(odometer.getX(), odometer.getY()),
                                            new Location(285.00, 15.00)
                                    );
                                    if (initialPointCase2 != null) {
                                        List<Location> secondPortion = PathFinder.generatePath(
                                                new Location(285.00, 15.00),
                                                new Location(135.00, 15.00)
                                        );
                                        if (secondPortion != null) initialPointCase2.addAll(secondPortion);
                                        pathToOffensiveZone = new ArrayList<>();
                                        pathToOffensiveZone.addAll(initialPointCase2);
                                    }
                                    break;
                                case 3:
                                    List<Location> initialPointCase3 = PathFinder.generatePath(
                                            new Location(odometer.getX(), odometer.getY()),
                                            new Location(285.00, 285.00)
                                    );
                                    if (initialPointCase3 != null) {
                                        List<Location> secondPortion = PathFinder.generatePath(
                                                new Location(285.00, 285.00),
                                                new Location(135.00, 15.00)
                                        );
                                        if (secondPortion != null) initialPointCase3.addAll(secondPortion);
                                        pathToOffensiveZone = new ArrayList<>();
                                        pathToOffensiveZone.addAll(initialPointCase3);
                                    }
                                    break;
                                case 4:
                                    List<Location> initialPointCase4 = PathFinder.generatePath(
                                            new Location(odometer.getX(), odometer.getY()),
                                            new Location(15.00, 285.00)
                                    );
                                    if (initialPointCase4 != null) {
                                        List<Location> secondPortion = PathFinder.generatePath(
                                                new Location(15.00, 285.00),
                                                new Location(135.00, 15.00)
                                        );
                                        if (secondPortion != null) initialPointCase4.addAll(secondPortion);
                                        pathToOffensiveZone = new ArrayList<>();
                                        pathToOffensiveZone.addAll(initialPointCase4);
                                    }
                                    break;
                            }

                            // If a path was made, drive to offense zone along that path.
                            if (pathToOffensiveZone != null) {

                                // DEBUG
//                        System.out.println("\n\nINITIAL PATH\n");
//                        for (Location current : pathToOffensiveZone) {
//                            System.out.println("Location: (" + current.getX() + "," + current.getY() + ")");
//                        }
//                        System.out.println("\n");

                                pathToOffensiveZone.remove(0);
                                Sound.twoBeeps();
                                // Basically: cycle through the locations in the path, telling nav to go each one.
                                for (Location loc : pathToOffensiveZone) {
                                    navigator.travelTo(loc.getX(), loc.getY());
                                    while (navigator.isNavigating()) {
                                        try {Thread.sleep(30);}
                                        catch (InterruptedException e) {e.printStackTrace();}
                                    }
                                }
                            }
                            else {
                                // If for whatever reason a path could not be made, notify and beep.
                                // Basically, just cop the freak out.
                                Sound.beep();
                                Button.ESCAPE.waitForPress();
                                LCDDisplay.sendToDisplay("PathGen Failed", true);
                                return;
                            }

                            // Once located at offensive zone, switch status to idle.
                            state = State.IDLE;
                            Sound.twoBeeps();
                            break;

                        /** DEFENSE */
                        case 1:

                            // Localize robot relative to board, given starting corner.
                            // 1: BL, 2: BR, 3: TR, 4: TL
                            int startingCornerDefense = KinematicModel.roundData.get("SC");
                            switch (startingCornerDefense) {
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

                            // Process roundData to log initial obstacles
                            PathFinder.blockOutBallBox();       // Ball Box must be blocked off.

                            // Generate path to defensive position.
                            List<Location> pathToDefensiveZone = null;
                            switch (KinematicModel.roundData.get("SC")) {
                                case 1:
                                    List<Location> random = PathFinder.generatePath(
                                            new Location(odometer.getX(), odometer.getY()),
                                            new Location(15.00, 15.00)
                                    );
                                    if (random != null) {
                                        List<Location> secondPortion = PathFinder.generatePath(
                                                new Location(15.00, 15.00),
                                                new Location(135.00, 280.00)
                                        );
                                        if (secondPortion != null) random.addAll(secondPortion);
                                        pathToDefensiveZone = new ArrayList<>();
                                        pathToDefensiveZone.addAll(random);
                                    }
                                    break;
                                case 2:
                                    List<Location> initialPointCase2 = PathFinder.generatePath(
                                            new Location(odometer.getX(), odometer.getY()),
                                            new Location(285.00, 15.00)
                                    );
                                    if (initialPointCase2 != null) {
                                        List<Location> secondPortion = PathFinder.generatePath(
                                                new Location(285.00, 15.00),
                                                new Location(135.00, 280.00)
                                        );
                                        if (secondPortion != null) initialPointCase2.addAll(secondPortion);
                                        pathToDefensiveZone = new ArrayList<>();
                                        pathToDefensiveZone.addAll(initialPointCase2);
                                    }
                                    break;
                                case 3:
                                    List<Location> initialPointCase3 = PathFinder.generatePath(
                                            new Location(odometer.getX(), odometer.getY()),
                                            new Location(285.00, 285.00)
                                    );
                                    if (initialPointCase3 != null) {
                                        List<Location> secondPortion = PathFinder.generatePath(
                                                new Location(285.00, 285.00),
                                                new Location(135.00, 280.00)
                                        );
                                        if (secondPortion != null) initialPointCase3.addAll(secondPortion);
                                        pathToDefensiveZone = new ArrayList<>();
                                        pathToDefensiveZone.addAll(initialPointCase3);
                                    }
                                    break;
                                case 4:
                                    List<Location> initialPointCase4 = PathFinder.generatePath(
                                            new Location(odometer.getX(), odometer.getY()),
                                            new Location(15.00, 285.00)
                                    );
                                    if (initialPointCase4 != null) {
                                        List<Location> secondPortion = PathFinder.generatePath(
                                                new Location(15.00, 285.00),
                                                new Location(135.00, 280.00)
                                        );
                                        if (secondPortion != null) initialPointCase4.addAll(secondPortion);
                                        pathToDefensiveZone = new ArrayList<>();
                                        pathToDefensiveZone.addAll(initialPointCase4);
                                    }
                                    break;
                            }

                            // If a path was made, drive to offense zone along that path.
                            if (pathToDefensiveZone != null) {

                                pathToDefensiveZone.remove(0);
                                Sound.twoBeeps();
                                // Basically: cycle through the locations in the path, telling nav to go each one.
                                for (Location loc : pathToDefensiveZone) {
                                    navigator.travelTo(loc.getX(), loc.getY());
                                    while (navigator.isNavigating()) {
                                        try {Thread.sleep(100);}
                                        catch (InterruptedException e) {e.printStackTrace();}
                                    }
                                }
                            }
                            else {
                                // If for whatever reason a path could not be made, notify and beep.
                                // Basically, just cop the freak out.
                                Sound.beep();
                                Button.ESCAPE.waitForPress();
                                LCDDisplay.sendToDisplay("PathGen Failed", true);
                                return;
                            }

                            // Don't move to next state.
                            Sound.twoBeeps();
                            Button.ESCAPE.waitForPress();
                            break;

                    }

                    break;

                /** CASE: IDLE
                 *  Waits for signal to go!
                 */
                case IDLE:
                    state = State.DRIVING_TO_BALLS;
                    break;

                /** CASE: DRIVING_TO_BALLS
                 *  Creates path to
                 */
                case DRIVING_TO_BALLS:

                    // Grab ball coordinates
                    double[] ll = {KinematicModel.roundData.get("ll-x")*30, KinematicModel.roundData.get("ll-y")*30};
                    double[] ur = {KinematicModel.roundData.get("ur-x")*30, KinematicModel.roundData.get("ur-y")*30};
                    double[][] ballCoordinates = ballLoader.computeCoordsOfBalls(ll, ur);

                    // Calculate the point we want to hit
                    double x = ballCoordinates[0][0] + 3.81;
                    double y = ballCoordinates[0][1] - 11.00;

                    // Drive to the middle of the block below the block with the ball platform.
                    List<Location> pathToBalls = PathFinder.generatePath(
                            new Location(odometer.getX(), odometer.getY()),
                            new Location(x, y)
                    );
                    for (Location loc : pathToBalls) {
                        navigator.travelTo(loc.getX(), loc.getY());
                        while (navigator.isNavigating()) {
                            try {Thread.sleep(300);}
                            catch (InterruptedException e) {e.printStackTrace();}
                        }
                    }

                    // Now drive a little bit below the platform.
                    navigator.travelTo(x, y - 10.0);
                    while (navigator.isNavigating()) {
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                            Sound.beep();
                            Sound.twoBeeps();
                        }
                    }
                    navigator.turnToAngle(0.00);

                    // Now drive up parallel to it.
                    navigator.travelTo(x, y);
                    while (navigator.isNavigating()) {
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                            Sound.beep();
                            Sound.twoBeeps();
                        }
                    }

                    state = State.LOADING;
                    break;

                /** CASE: LOADING
                 *  Basically does nothing while waiting for brick2 to load balls.
                 */
                case LOADING:
                    // Grab the ball.
                    ballLoader.fetchBall();

                    // Drive back with the ball.
                    Main.leftMotor.backward();
                    Main.rightMotor.backward();
                    try {
                        Thread.sleep(3500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                        Sound.beep();
                        Sound.twoBeeps();
                    }
                    Main.leftMotor.stop();
                    Main.rightMotor.stop();

                    // Switch to appropriate state.
                    if (ballLoader.isValidBall()) {
                        state = State.DRIVING_TO_SHOOT;
                    } else {
                        state = State.DRIVING_TO_THROW_AWAY;
                    }
                    break;

                /** CASE: DRIVING_TO_THROW_AWAY
                 *
                 */
                case DRIVING_TO_THROW_AWAY:

                    // Turn around.
                    navigator.turnToAngle(Math.PI);

                    // Go to next state.
                    state = State.SHOOTING;
                    break;

                /** CASE: DRIVING_TO_SHOOT
                 *
                 */
                case DRIVING_TO_SHOOT:

                    // Drive to #datShootingLocation
                    List<Location> pathToShootingLocation = PathFinder.generatePath(
                            new Location(odometer.getX(), odometer.getY()),
                            new Location(165.00, 165.00)
                    );
                    for (Location loc : pathToShootingLocation) {
                        navigator.travelTo(loc.getX(), loc.getY());
                        while (navigator.isNavigating()) {
                            try {Thread.sleep(300);}
                            catch (InterruptedException e) {e.printStackTrace();}
                        }
                    }

                    // Rotate to face goal.
                    navigator.turnToAngle(0.00);

                    // Go to shooting state.
                    state = State.SHOOTING;
                    break;

                /** CASE: SHOOTING
                 *
                 */
                case SHOOTING:
                    ballLoader.shootBall();
                    state = State.IDLE;
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
