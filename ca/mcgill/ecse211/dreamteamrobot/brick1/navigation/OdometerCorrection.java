package ca.mcgill.ecse211.dreamteamrobot.brick1.navigation;

import lejos.hardware.Sound;
import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import lejos.internal.io.SystemSettings;

/**
 * Thread simultaneous to Odometer. This thread reads values from the environment (ie. color sensor values)
 * and compares them with current values in the Odometer so that it can perform any necessary corrections
 * for improved navigational precision.
 */
public class OdometerCorrection extends Thread {

    private int LONG_SLEEP = 100;
    private int SHORT_SLEEP = 50;
    private double R_THRESHOLD = 50.00;
    private double MAX_CORRECTION_DETECT_DIST = 10;

    // HEADING_ERROR is how closely the robot's heading must be to perpendicular to the line in order to initiate correction
    // ie : HEADING_ERROR of 0 would mean the robots heading would have to be a perfect 90deg from the line it is crossing
    private double HEADING_ERROR = 15 * Math.PI / 180;
    private int POSITION_ERROR = 5;
    // MAX_DELTA_TACHO : amount of error between the right and left wheel tachos when they each detect their lines.
    // if the difference in distance travelled between the 2 wheels exceeds this # the correction attempt is cancelled and tachos are reset.
    private double MAX_DELTA_TACHO = (HEADING_ERROR * KinematicModel.WHEELBASE / KinematicModel.WHEEL_RADIUS_L) * 180 / Math.PI;

    private boolean[] aUpdateArr = {true, true, true};
    private boolean[] xUpdateArr = {true, false, false};
    private boolean[] yUpdateArr = {false, true, false};

    private Odometer odometer;

    private ColourPoller leftColourPoller;
    private ColourPoller rightColourPoller;

    private double rightTacho, leftTacho;
    private double[] leftLoc = new double[3];
    private double[] rightLoc = new double[3];

    public OdometerCorrection (Odometer odometer, ColourPoller leftP, ColourPoller rightP) {
        this.leftTacho = 0;
        this.rightTacho = 0;
        this.leftColourPoller = leftP;
        this.rightColourPoller = rightP;
        this.odometer = odometer;
    }

    private static double calcLineInterceptAngle(double left, double right){
        double deltaTheta = (left - right) * Math.PI / 180;
        double sideLength = deltaTheta * KinematicModel.WHEEL_RADIUS_L;
        return -1 * Math.atan(sideLength / KinematicModel.COLOUR_SENSOR_SPACING);
    }

    /**
     * Corrects x,y on odometer.
     */
    private void correctCoords(){

        // Variables
        double heading = odometer.getTheta();
        double xDir = 0;
        double yDir = 0;
        double absAdjust, updateAmount;
        double[] updateArr = new double[3];

        // Determine direction
        if(Math.abs(heading - 0) <= HEADING_ERROR){
            yDir = 1; // heading positive y direction
        } else if (Math.abs(heading - Math.PI/2.0) <= HEADING_ERROR){
            xDir = 1; // heading positive x direction
        } else if (Math.abs(heading - Math.PI) <= HEADING_ERROR){
            yDir = -1; // heading negative y direction
        } else if (Math.abs(heading - (Math.PI + Math.PI/2.0)) <= HEADING_ERROR){
            xDir = -1; // heading negative x direction
        }

        // Perform correction based on direction

        // Heading positive x or negative x
        if(xDir != 0) {
            // Grab coordinates from odometer.
            double[] doublexPos = new double[3];
            odometer.getPosition(doublexPos, xUpdateArr);

            // Send x coordinate into getDistanceAdjust
            // to get required update amount.
            absAdjust = getDistanceAdjust(doublexPos[0]);

            // Switch sign of adjustment depending on direction.
            updateAmount = xDir * absAdjust;

            // Update position in odometer.
            updateArr[0] = updateAmount + doublexPos[0];
            odometer.setPosition(updateArr, xUpdateArr);
        }
        // Heading positive y or negative y
        else {
            // Grab coordinates from odometer.
            double[] doubleyPos = new double[3];
            odometer.getPosition(doubleyPos, yUpdateArr);

            // Send y coordinate into getDistanceAdjust
            // to get required update amount
            absAdjust = getDistanceAdjust(doubleyPos[1]);

            // Then switch sign of adjustment depending on direction
            updateAmount = yDir * absAdjust;

            // Update position in odometer.
            updateArr[1] = updateAmount + doubleyPos[1];
            odometer.setPosition(updateArr, yUpdateArr);
        }

        System.out.println("updated Coord to x: "+updateArr[0] + ", y: "+updateArr[1]);
    }

    /**
     * Given either an x position or a y position, determines whether to adjust to next multiple of 30
     * or previous multiple of 30.
     * @param odoPosition x or y position
     * @return Adjustment to be added to parameter.
     */
    private static double getDistanceAdjust(double odoPosition) {

        // In the case that we are localizing, and the robot is reading less than 0,
        // just return 0.0.
        if (odoPosition < 0) return 0.0;

        // Variables
        double previous30, distFromPrev30, closest30;

        // Determine how far the current odoPosition is from the last line.
        previous30 = ((int) (odoPosition / 30.0)) * 30;
        distFromPrev30 = odoPosition - previous30;

        // Correct position.
        if(distFromPrev30 > 15) {
            // odoPosition is shorter than actual
            closest30 = previous30 + 30;
        } else {
            // odoPosition is farther than actual (bot thinks its past the line
            closest30 = previous30;
        }

        // Return required adjustment.
        return (closest30 - odoPosition);
    }

    /**
     * Checks if robot is travelling vertically or horizontally, approximately.
     * Checks if coordinates are around a known line (multiple of 30).
     * @return True if robot is travelling vertically horizontally and around line. False if not.
     */
    private boolean checkCorrectionCandidate(){

        // Check if heading is within accepted heading error:
        double headingErr = Math.abs(odometer.getTheta() % (Math.PI/2.0));
        boolean headingValid = headingErr < HEADING_ERROR;
            // double headingErr = Math.PI/4 - Math.abs(this.odometer.getTheta() % Math.PI/2 - Math.PI/4);

        // Check if coords are within acceptable distances
        //double xErr = Math.abs(odometer.getX())%30);
        double xErr = 15 - Math.abs(Math.abs(odometer.getX())%30 - 15);
        double yErr = 15 - Math.abs(Math.abs(odometer.getY())%30 - 15);

        boolean coordValid = xErr < POSITION_ERROR || yErr < POSITION_ERROR;
        //System.out.println("" + (coordValid && headingValid) + " --- head:"+this.odometer.getTheta()+",headingErr:"+headingErr+ " - x:"+this.odometer.getX()+",xErr:"+xErr+" - y:"+this.odometer.getY()+":yErr : "+yErr);
        return coordValid && headingValid;

    }

    /**
     * Checks if a colour sensor is reading a line.
     * @param cp Colour poller for colour sensor.
     * @return True if there is a line. False if not.
     */
    private boolean colourSensorHitLine(ColourPoller cp){
        boolean hit = cp.getSensorValue() < R_THRESHOLD;
        if (hit) Sound.beep();
        return hit;
    }

    /**
     * Manages odometer correction.
     */
    private void beginOdoCorrection(){

        // poll left color sensor
        boolean leftSensorHitLine = colourSensorHitLine(leftColourPoller);
        if(leftSensorHitLine && leftTacho == 0){
            leftTacho = odometer.getLeftMotor().getTachoCount();
            odometer.getPosition(leftLoc, aUpdateArr);
        }
        // poll right color sensor
        boolean rightSensorHitLine = colourSensorHitLine(rightColourPoller);
        if(rightSensorHitLine && rightTacho == 0){
            rightTacho = odometer.getRightMotor().getTachoCount();
            odometer.getPosition(rightLoc, aUpdateArr);
        }

        if(leftTacho != 0 && rightTacho != 0){
            // account for possibility that the bot was turning or something and the tachos are very different
            System.out.println("OdoCorrecting - leftTacho:"+leftTacho + ", rightTacho:"+rightTacho);
            double detectDist = Math.abs(Math.pow(rightLoc[0]-leftLoc[0],2) + Math.pow(rightLoc[1]-leftLoc[1],2));
            if(Math.abs(leftTacho - rightTacho) > MAX_DELTA_TACHO && detectDist < MAX_CORRECTION_DETECT_DIST){
                leftTacho = 0;
                rightTacho = 0;
                return;
            } else {


                // correct for coordinates
                correctCoords();

                // reset line tacho counts
                leftTacho = 0;
                rightTacho = 0;
            }
        }
    }


    public void correctTheta(double rightTacho, double leftTacho){
        // calculate for heading
        double lineInterceptAngle = calcLineInterceptAngle(leftTacho, rightTacho);
        double correctedHeading;

        // robot is driving parallel to yAxis.. therefore angle is near 0 or 180
        boolean yDir = (Math.PI/4 - Math.abs(this.odometer.getTheta() % Math.PI - Math.PI/4)) < HEADING_ERROR;
        if(yDir){
            correctedHeading = lineInterceptAngle;
        } else {
            correctedHeading = lineInterceptAngle + Math.PI / 2;
        }

        // update heading in the odometer..
        System.out.println("UpdatingTheta from : "+odometer.getTheta() + " to : "+correctedHeading);
        this.odometer.setTheta(correctedHeading);
    }

    @Override
    public void run() {
        boolean nearCandidate;

        // We don't want to waste processing power or anything, so if
        // we determine that we're close to a spot where we have to correct,
        // then we do so. Otherwise, we rest for a longer period.
        while(true){

            nearCandidate = checkCorrectionCandidate();

            if(nearCandidate){
                beginOdoCorrection();
                try {
                    Thread.sleep(SHORT_SLEEP);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {

                try {
                    Thread.sleep(LONG_SLEEP);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
