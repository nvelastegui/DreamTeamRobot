package ca.mcgill.ecse211.dreamteamrobot.main;

import ca.mcgill.ecse211.dreamteamrobot.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.navigation.OdometerCorrection;
import ca.mcgill.ecse211.dreamteamrobot.navigation.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;

/**
 * This is the main thread for the navigational brick. It runs as state machine.
 */
public class Driver extends Thread {

    /** Constants: State */
    enum State {INIT};
    private State state;

    /** Variables: Sub Threads */
    private Odometer odometer;
    private OdometerCorrection odometerCorrection;
    private UltrasonicPoller ultrasonicPoller;
    private Navigator navigator;


    /**
     * Constructor.
     * @param leftMotor motor for left wheel
     * @param rightMotor motor for right wheel
     * @param ultrasonicSensorMotor motor for ultrasonic sensor's rotation about vertical axis
     * @param ultrasonicSensorPort port for ultrasonic sensor
     */
    public Driver (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor ultrasonicSensorMotor, Port ultrasonicSensorPort) {

        // Create thread instances.
        this.odometer = new Odometer(leftMotor, rightMotor);
        this.odometerCorrection = new OdometerCorrection(odometer);
        this.ultrasonicPoller = new UltrasonicPoller(ultrasonicSensorPort);
        this.navigator = new Navigator(odometer, ultrasonicPoller, leftMotor, rightMotor, ultrasonicSensorMotor);

        // Set state to init.
        state = State.INIT;
    }

    /**
     * @return Current status of driver, for display on LCD.
     */
    public String getStatus () {
        // add stuff here
        return null;
    }

    /**
     * Set up sub threads before running.
     */
    public void performPreExecute () {
        // add stuff here
    }

    /**
     * Thread Run.
     * This is the main run method for the driver. Structure is heavily based on material
     * from threadTutorial.pdf and code from Lab 3.
     */
    @Override
    public void run() {

        //ObstacleAvoider avoidance = new ObstacleAvoider(this);

//        while (true) {
//
//            switch (state) {
//                /** */
//                case INIT:
//                    if (status) {
//                        state = State.TURNING;
//                    }
//                    break;
//
//                default:
//                    leftMotor.stop();
//                    rightMotor.stop();
//                    break;
//            }
//
//            try {
//                Thread.sleep(30);
//            }
//            catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//
//        }

    }


}
