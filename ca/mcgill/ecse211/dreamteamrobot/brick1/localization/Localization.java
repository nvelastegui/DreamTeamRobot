package ca.mcgill.ecse211.dreamteamrobot.brick1.localization;

import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Static class with methods for localizing robot before commencing central thread.
 * @PlayerType Offense or Defense
 */
public class Localization {

    public enum PlayerType {Offense, Defense};

    /** Variables: Localizers */
    UltrasonicLocalizer ultrasonicLocalizer;
    LightLocalizer lightLocalizer;

    /** Variables: Dependencies */
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private Navigator navigator;
    private UltrasonicPoller ultrasonicPollerLeft;
    private UltrasonicPoller ultrasonicPollerRight;
    private ColourPoller colourPollerLeft;
    private ColourPoller colourPollerRight;

    public Localization (EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor, Odometer odometer, Navigator navigator, UltrasonicPoller ultrasonicPollerLeft, UltrasonicPoller ultrasonicPollerRight, ColourPoller colourPollerLeft, ColourPoller colourPollerRight) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.navigator = navigator;
        this.ultrasonicPollerLeft = ultrasonicPollerLeft;
        this.ultrasonicPollerRight = ultrasonicPollerRight;
        this.colourPollerLeft = colourPollerLeft;
        this.colourPollerRight = colourPollerRight;
    }

    /**
     * Sets up dependencies for localization (sub threads, values, etc.)
     * @return True if set up successful. False if not success (ie. a sub-thread is not running).
     */
    public boolean setupLocalizer () {

        // Set up the ultrasonic localizer.
        ultrasonicLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, navigator, ultrasonicPollerLeft, ultrasonicPollerRight);

        // Set up the light localizer.
        lightLocalizer = new LightLocalizer(leftMotor, rightMotor, odometer, navigator, colourPollerLeft, colourPollerRight);

        // TODO: Check that all threads are running
        return true;
    }


    /**
     * Localize robot, given player type.
     * @param playerType Player type.
     */
    public void doLocalization(PlayerType playerType) {

        // Perform #datLocalizationDoe
        ultrasonicLocalizer.doLocalization();

        // Perform colour sensor localization.
        // lightLocalizer.doLocalization();

        // Move to proper starting location.
        // TODO: start navigator here?
        // navigator.start();

    }


}
