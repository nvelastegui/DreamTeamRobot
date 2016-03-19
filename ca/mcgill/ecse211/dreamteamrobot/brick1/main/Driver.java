package ca.mcgill.ecse211.dreamteamrobot.brick1.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.communication.DriverStatusPacket;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.OdometerCorrection;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;

/**
 * This is the main offensive thread for the navigational brick. It runs as a state machine.
 */
public class Driver extends Thread {

    /** Constants: State */
    enum State {OFF, INIT};
    private State state;

    /** Variables: Sub Threads */
    private Odometer odometer;
    private OdometerCorrection odometerCorrection;
    private UltrasonicPoller ultrasonicPollerLeft;
    private UltrasonicPoller ultrasonicPollerRight;
    private Navigator navigator;

    /**
     * Constructor.
     * @param leftMotor motor for left wheel
     * @param rightMotor motor for right wheel
     * @param ultrasonicSensorMotor motor for ultrasonic sensor's rotation about vertical axis
     * @param ultrasonicSensorPortLeft port for left ultrasonic sensor
     * @param ultrasonicSensorPortRight port for right ultrasonic sensor
     */
    public Driver (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor ultrasonicSensorMotor, Port ultrasonicSensorPortLeft, Port ultrasonicSensorPortRight) {

        // Create thread instances.
        this.odometer = new Odometer(leftMotor, rightMotor);
        this.odometerCorrection = new OdometerCorrection(odometer);
        this.ultrasonicPollerLeft = new UltrasonicPoller(ultrasonicSensorPortLeft);
        this.ultrasonicPollerRight = new UltrasonicPoller(ultrasonicSensorPortRight);
        this.navigator = new Navigator(odometer, ultrasonicPollerLeft, leftMotor, rightMotor, ultrasonicSensorMotor);

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

    public Navigator getNavigator() {
        return navigator;
    }

    public UltrasonicPoller getUltrasonicPollerLeft() {
        return ultrasonicPollerLeft;
    }

    public UltrasonicPoller getUltrasonicPollerRight() {
        return ultrasonicPollerRight;
    }

    /**
     * Set up sub threads before running.
     */
    public void performPreExecute () {
        // add stuff here
        odometer.start();
        //odometerCorrection.start();
        ultrasonicPollerLeft.start();
        ultrasonicPollerRight.start();
        // navigator.start(); TODO: needs to be started later so that obstacle avoidance doesn't fuck localization.

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
