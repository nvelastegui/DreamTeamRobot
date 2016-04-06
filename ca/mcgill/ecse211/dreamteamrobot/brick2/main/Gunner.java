package ca.mcgill.ecse211.dreamteamrobot.brick2.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import org.json.simple.JSONObject;

/**
 * Main thread on brick 2, in charge of loading and shooting balls.
 */
public class Gunner extends Thread {

    /** Possible States */
    enum State {IDLE, LOADING, SHOOTING};
    private State state;




    /** Instance Variables */
    private EV3LargeRegulatedMotor shootMotor;
    private EV3LargeRegulatedMotor claspMotor;
    private ColourPoller colourPoller;

    /** Communication */
    private Connection brick1;
    private Connection comp;

    /**
     * Constructor
     * @param shootMotor Motor for shooter.
     * @param claspMotor Motor for clasp mechanism.
     * @param colourSensorPort Colour sensor for detecting ball.
     */
    public Gunner (EV3LargeRegulatedMotor shootMotor, EV3LargeRegulatedMotor claspMotor, Port colourSensorPort, Connection brick1, Connection comp) {
        this.shootMotor = shootMotor;
        this.claspMotor = claspMotor;
        this.colourPoller = new ColourPoller(colourSensorPort);
        this.brick1 = brick1;
        this.comp = comp;

        comp.queue.registerQueue(KinematicModel.ROUTES.CLAWS_MOVE.toString());
        comp.queue.registerQueue(KinematicModel.ROUTES.READ_BALL.toString());
    }

    /**
     * Prepares thread for execution.
     * @return True if nothing went wrong, false if else.
     */
    public boolean performPreExecute () {

        // Start colour poller.
        colourPoller.start();

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
    public void moveClasp (int destAngle) {
        claspMotor.setAcceleration(KinematicModel.CLASP_MOTOR_ACCELERATION);
        claspMotor.setSpeed(KinematicModel.CLASP_MOTOR_SPEED);
        claspMotor.rotate(destAngle);
        while (claspMotor.isMoving()){}
    }

    /**
     * Opens front clasp.
     */
    public void openClasp () {
        claspMotor.setAcceleration(KinematicModel.CLASP_MOTOR_ACCELERATION);
        claspMotor.setSpeed(KinematicModel.CLASP_MOTOR_SPEED);
        claspMotor.rotate(KinematicModel.CLASP_MOTOR_ROTATION_ANGLE_FOR_OPEN_OR_CLOSE);
        while (claspMotor.isMoving()){}
    }

    /**
     * Closes front clasp.
     */
    public void closeClasp () {
        claspMotor.setAcceleration(KinematicModel.CLASP_MOTOR_ACCELERATION);
        claspMotor.setSpeed(KinematicModel.CLASP_MOTOR_SPEED);
        claspMotor.rotate(-KinematicModel.CLASP_MOTOR_ROTATION_ANGLE_FOR_OPEN_OR_CLOSE);
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
        return executeShoot(KinematicModel.SHOOTER_ARM_ACCELERATION, KinematicModel.SHOOTER_ARM_ROTATION_SPEED, KinematicModel.SHOOTER_ROTATION_ANGLE);
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
        shootMotor.setAcceleration(KinematicModel.SHOOTER_ARM_LOCALIZATION_ACCELERATION);
        shootMotor.setSpeed(KinematicModel.SHOOTER_ARM_LOCALIZATION_SPEED);
        shootMotor.rotateTo(KinematicModel.SHOOTER_ARM_TOP_POSITION_ANGLE);
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
            // handle various Queues
            handleClaws(this.brick1);
            handleClaws(this.comp);
            handleColourRead(this.brick1);
            handleColourRead(this.comp);

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

    private void handleClaws(Connection con){
        if(con == null) return;

        String routeName = KinematicModel.ROUTES.CLAWS_MOVE.toString();
        JSONObject incomingMsg = con.queue.popJSON(routeName);

        if(incomingMsg != null){
            System.out.println("CLAWS_MOVE to : "+incomingMsg.get("claws_angle"));
            int angle = ((Long)incomingMsg.get("claws_angle")).intValue();
            moveClasp(angle);     // waits until motion finished

            // send brick1 confirmation that closing the clasps is done
            con.out.sendJSONObj(KinematicModel.ROUTES.CLAWS_CLOSED.toString(), null);
        }
    }

    private void handleColourRead(Connection con){
        if(con == null) return;

        JSONObject incomingMessage = con.queue.popJSON(KinematicModel.ROUTES.READ_BALL.toString());
        if(incomingMessage != null){

            float[] RGB_val = colourPoller.getRGB();

            // send brick1 confirmation that closing the clasps is done
            JSONObject colorReading = new JSONObject();
            colorReading.put("ball_colour", colorReading);
            con.out.sendJSONObj(KinematicModel.ROUTES.BALL_COLOUR.toString(), colorReading);
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
