package ca.mcgill.ecse211.dreamteamrobot.brick2.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;

/**
 * Main thread on brick 2, in charge of loading and shooting balls.
 */
public class Gunner extends Thread {

    /** Possible States */
    enum State {IDLE, LOADING, SHOOTING};
    private State state;

    /** Constants: Clasp Motor */
    private static final int CLASP_MOTOR_ROTATION_ANGLE_FOR_OPEN_OR_CLOSE = 10; // Angle of rotation required to open or close
                                                                                // the clasp exactly ONCE.
    private static final int CLASP_MOTOR_SPEED = 200;
    private static final int CLASP_MOTOR_ACCELERATION = 3000;

    /** Constants: Shooting */
    private static final int SHOOTER_ROTATION_ANGLE = 17; // in degrees
    private static final int SHOOTER_ARM_ROTATION_SPEED = 900; // in deg/sec -> theoretical maximum of 960 deg/s.
    private static final int SHOOTER_ARM_ACCELERATION   = 3000; // Used the same as previously used.

    /** Constants: Shooting Arm Localization */
    private static final int SHOOTER_ARM_TOP_POSITION_ANGLE = 120; // Assuming the resting position for the arm is zero
                                                                   // degrees, this is the angle of the arm in the top
                                                                   // position.
    private static final int SHOOTER_ARM_LOCALIZATION_ACCELERATION = 3000;
    private static final int SHOOTER_ARM_LOCALIZATION_SPEED = 250;


    /** Instance Variables */
    private EV3LargeRegulatedMotor shootMotor;
    private EV3LargeRegulatedMotor claspMotor;
    private ColourPoller colourPoller;

    /**
     * Constructor
     * @param shootMotor Motor for shooter.
     * @param claspMotor Motor for clasp mechanism.
     * @param colourSensorPort Colour sensor for detecting ball.
     */
    public Gunner (EV3LargeRegulatedMotor shootMotor, EV3LargeRegulatedMotor claspMotor, Port colourSensorPort) {
        this.shootMotor = shootMotor;
        this.claspMotor = claspMotor;
        this.colourPoller = new ColourPoller(colourSensorPort);
    }

    /**
     * Prepares thread for execution.
     * @return True if nothing went wrong, false if else.
     */
    public boolean performPreExecute () {

        // Start colour poller.
        //colourPoller.start();

        // Reset tacho count on clasp motor.
        claspMotor.resetTachoCount();

        // Open clasp
        openClasp();

        // Drop shooter arm to bottom position (in case it is not)
        // This also resets the tacho count to 0 at the bottom position.
        dropArmToBottomPosition();

        // Move the motor to the top position
        moveArmToTopPosition();

        // Close clasp
        closeClasp();

        return true;
    }

    /**
     * Opens front clasp.
     */
    public void openClasp () {
        claspMotor.setAcceleration(CLASP_MOTOR_ACCELERATION);
        claspMotor.setSpeed(CLASP_MOTOR_SPEED);
        claspMotor.rotate(CLASP_MOTOR_ROTATION_ANGLE_FOR_OPEN_OR_CLOSE);
        while (claspMotor.isMoving()){}
    }

    /**
     * Closes front clasp.
     */
    public void closeClasp () {
        claspMotor.setAcceleration(CLASP_MOTOR_ACCELERATION);
        claspMotor.setSpeed(CLASP_MOTOR_SPEED);
        claspMotor.rotate(-CLASP_MOTOR_ROTATION_ANGLE_FOR_OPEN_OR_CLOSE);
        while (claspMotor.isMoving()){}
    }

    /**
     * Executes a ball shoot given relevant parameters.
     * @param acceleration Motor acceleration to use.
     * @param speed Motor speed to use.
     * @param cutOffAngleCandidate Angle after which to stop motor (ie. rotate for 'cutOffAngleCandidate' degrees).
     *                             Can be passed as -1 to use default.
     * @return True if executed. False if Gunner is currently doing something else and cannot shoot.
     */
    public boolean executeShoot (int acceleration, int speed, int cutOffAngleCandidate) {

        // If no cutOffAngle is passed, then use default.
        int cutOffAngle;
        if (cutOffAngleCandidate == -1) cutOffAngle = 17;
        else cutOffAngle = cutOffAngleCandidate;

        if (state == State.IDLE) {

            // Because the method
            // catapultMotor.rotate(CATAPULT_ROTATION_ANGLE, false);
            // apparently doesn't work - ie. it returns immediately no matter what - I wrote this method
            // to ensure that the motion is allowed to finish.

            // Set the acceleration and speed.
            shootMotor.setAcceleration(acceleration);
            shootMotor.setSpeed(speed);

            // Go forward until arm passes threshold angle.
            shootMotor.forward();
            while (true) {
                // When the arm passes the threshold ange - ie. it's done its swing -
                // then stop the motor.
                if (shootMotor.getTachoCount() > cutOffAngle) {
                    shootMotor.stop();
                    return dropArmToBottomPosition();
                }
            }
        }
        else return false;

    }

    /**
     * Performs shoot with default parameters.
     * @return True if successful. False if shooting not available.
     */
    public boolean executeShoot () {
        return executeShoot(SHOOTER_ARM_ACCELERATION, SHOOTER_ARM_ROTATION_SPEED, SHOOTER_ROTATION_ANGLE);
    }

    /**
     * Returns shoot motor to resting position.
     * @return True if successful.
     */
    public boolean moveArmToBottomPosition () {
        // Grab the current tacho count. This is considered to be from bottom position.
        // (ie. before shooting the tachometer is always reset to zero at the bottom resting position).
        int currentTacho = shootMotor.getTachoCount(); // Is this needed?
        shootMotor.rotateTo(0); // TODO: Figure out what parameter should be here?
        while (shootMotor.isMoving()) {}
        shootMotor.resetTachoCount();
        return true;
    }

    /**
     * Drops shooting arm to bottom position. Resets tacho count.
     * @return True if successful. False if not.
     */
    public boolean dropArmToBottomPosition () {
        shootMotor.flt();
        pause(1500);
        shootMotor.resetTachoCount();
        return true;
    }

    /**
     * Rotates shooter arm to top position.
     * @return True if successful.
     */
    public boolean moveArmToTopPosition () {
        shootMotor.setAcceleration(SHOOTER_ARM_LOCALIZATION_ACCELERATION);
        shootMotor.setSpeed(SHOOTER_ARM_LOCALIZATION_SPEED);
        shootMotor.rotateTo(SHOOTER_ARM_TOP_POSITION_ANGLE);
        while (shootMotor.isMoving()) {}
        return true;
    }

    @Override
    public void run() {
        super.run();

        // Start thread at idle.
        state = State.IDLE;

        // Loop indefinitely.
        while (true) {

            switch (state) {

                case IDLE:


                    break;
                case LOADING:


                    break;
                case SHOOTING:


                    break;

            }

        }


    }

    /**
     * Pauses thread.
     * @param timeToStop amount of time to stop, in milliseconds.
     */
    private void pause(int timeToStop) {
        try {
            Thread.sleep(timeToStop);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
