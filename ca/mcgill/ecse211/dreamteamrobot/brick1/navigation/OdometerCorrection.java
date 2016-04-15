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
    private double MAX_CORRECTION_DETECT_DIST = 5;

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

    // tacho counts of both wheels at left colour sensor line detection event
    private double leftDetectrightTacho, leftDetectleftTacho;
    // tacho counts of both wheels at right colour sensor line detection event
    private double rightDetectrightTacho, rightDetectleftTacho;

    private double[] leftLoc = new double[3];
    private double[] rightLoc = new double[3];

    /**
     * Parent class for odometryCorrection Object. Stores Odometer object, detectionTachoCounts and Left / Right Color Pollers.
     * This class operates as its own thread and constantly polls the color sensors. Performs calculations when lines are hit and updates the odometer object.
     * @param odometer
     * @param leftP
     * @param rightP
     */
    public OdometerCorrection (Odometer odometer, ColourPoller leftP, ColourPoller rightP) {
        this.leftDetectleftTacho = 0;
        this.leftDetectrightTacho = 0;
        this.rightDetectleftTacho = 0;
        this.rightDetectrightTacho = 0;

        this.leftColourPoller = leftP;
        this.rightColourPoller = rightP;
        this.odometer = odometer;
    }

    /**
     * Corrects x,y on odometer.
     */
    private void correctCoords(){

        // Variables
        double heading = odometer.getTheta();
        double xDir = 0;
        double yDir = 0;
        double nearestLine, updateAmount;

        // Determine direction
        if(Math.abs(heading - 0) <= HEADING_ERROR || Math.abs(heading - Math.PI*2) <= HEADING_ERROR){
            // account for small negative # and / or positive #s close to 360
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
            nearestLine = getDistanceAdjust(doublexPos[0]);

            // Switch sign of adjustment depending on direction.
            updateAmount = nearestLine - xDir * KinematicModel.COLOR_SENSOR_FORWARD_OFFSET;

            odometer.setX(updateAmount);
            System.out.println("Update X from :"+doublexPos[0]+", to :"+updateAmount);
        }
        // Heading positive y or negative y
        else {
            // Grab coordinates from odometer.
            double[] doubleyPos = new double[3];
            odometer.getPosition(doubleyPos, yUpdateArr);

            // Send y coordinate into getDistanceAdjust
            // to get required update amount
            nearestLine = getDistanceAdjust(doubleyPos[1]);

            // Then switch sign of adjustment depending on direction
            updateAmount = nearestLine - yDir * KinematicModel.COLOR_SENSOR_FORWARD_OFFSET;

            odometer.setY(updateAmount);
            System.out.println("Update Y from :"+doubleyPos[1]+", to :"+updateAmount);
        }
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
        return closest30;
    }

    /**
     * Checks if robot is travelling vertically or horizontally, approximately.
     * Checks if coordinates are around a known line (multiple of 30).
     * @return True if robot is travelling vertically horizontally and around line. False if not.
     */
    private boolean checkCorrectionCandidate(){

        // Check if heading is within accepted heading error:
        boolean headingValid = (Math.PI/4 - Math.abs((this.odometer.getTheta() % (Math.PI/2)) - Math.PI/4)) < HEADING_ERROR;

        // Check if coords are within acceptable distances
        double xErr = 15 - Math.abs(Math.abs(odometer.getX() % 30) - 15);
        double yErr = 15 - Math.abs(Math.abs(odometer.getY() % 30) - 15);

        // if in the first box check for lines..
        boolean coordValid = xErr < POSITION_ERROR || yErr < POSITION_ERROR;
        // Note: I took out the xCoord and yCoord thing since it was causing problems on some runs.
        //       Basically, it would be in "checkCorrectionCandidate" mode continuously when in the bottom
        //       left square. Sometimes, it would pick up a false positive early and subsequently grab the
        //       other positive (actually on the line on top) and due to the huge tacho count difference
        //       overturn a shit ton.
        // boolean coordValid = xErr < POSITION_ERROR || yErr < POSITION_ERROR || (xCoord<0 && yCoord<0);
        return coordValid && headingValid;

    }

    /**
     * Checks if a colour sensor is reading a line.
     * @param cp Colour poller for colour sensor.
     * @return True if there is a line. False if not.
     */
    private boolean colourSensorHitLine(ColourPoller cp){
        boolean hit = cp.getSensorValue() < R_THRESHOLD;
        return hit;
    }

    /**
     * Manages odometer correction.
     */
    private void beginOdoCorrection(){

        // Poll left color sensor
        boolean leftSensorHitLine = colourSensorHitLine(leftColourPoller);
        if(leftSensorHitLine && (leftDetectleftTacho == 0 && leftDetectrightTacho == 0)){
            leftDetectleftTacho = odometer.getLeftMotor().getTachoCount();
            leftDetectrightTacho = odometer.getRightMotor().getTachoCount();
            odometer.getPosition(leftLoc, aUpdateArr);
        }
        // Poll right color sensor
        boolean rightSensorHitLine = colourSensorHitLine(rightColourPoller);
        if(rightSensorHitLine && (rightDetectleftTacho == 0 && rightDetectrightTacho == 0)){
            rightDetectleftTacho = odometer.getLeftMotor().getTachoCount();
            rightDetectrightTacho = odometer.getRightMotor().getTachoCount();
            odometer.getPosition(rightLoc, aUpdateArr);
        }

        //System.out.println("lDlT:"+leftDetectleftTacho+",lDrT:"+leftDetectrightTacho+",rDlT:"+rightDetectleftTacho+", rDrT:"+rightDetectrightTacho);

        if(leftDetectleftTacho != 0 && leftDetectrightTacho != 0 && rightDetectleftTacho != 0 && rightDetectrightTacho != 0){
            // account for possibility that the bot was turning or something and the tachos are very different
            System.out.println("beginOdoCorrection -- leftDetectleftTacho:"+leftDetectleftTacho+", leftDetectrightTacho:"+leftDetectrightTacho+", rightDetectleftTacho:"+rightDetectleftTacho+", rightDetectrightTacho:"+rightDetectrightTacho);
            double detectDist = Math.sqrt(Math.pow(rightLoc[0]-leftLoc[0],2) + Math.pow(rightLoc[1]-leftLoc[1],2));

            // check to make sure the left and right detection events were close together and hence one line
            if(Math.abs(leftDetectleftTacho - rightDetectleftTacho) > MAX_DELTA_TACHO || Math.abs(leftDetectrightTacho - rightDetectrightTacho) > MAX_DELTA_TACHO || detectDist > MAX_CORRECTION_DETECT_DIST){
                leftDetectleftTacho = 0;
                leftDetectrightTacho = 0;
                rightDetectleftTacho = 0;
                rightDetectrightTacho = 0;
            } else {
                // correct for coordinates
                correctCoords();

                // correct heading
                correctTheta(leftDetectleftTacho, leftDetectrightTacho, rightDetectleftTacho, rightDetectrightTacho);

                Sound.twoBeeps();

                // reset line tacho counts
                leftDetectleftTacho = 0;
                leftDetectrightTacho = 0;
                rightDetectleftTacho = 0;
                rightDetectrightTacho = 0;
            }
        }
    }

    /**
     * Reads in various tachoCounts and performs trig calculations to calculate the intecept theta and correct the odometer object accordingly.
     * @param leftDetectleftTacho
     * @param leftDetectrightTacho
     * @param rightDetectleftTacho
     * @param rigthDetectrightTacho
     */
    public void correctTheta(double leftDetectleftTacho, double leftDetectrightTacho, double rightDetectleftTacho, double rigthDetectrightTacho){
        // calculate for heading
        double lineInterceptAngle;

        double heading = odometer.getTheta();
        double xDir = 0;
        double yDir = 0;

        // Determine direction
        if(Math.abs(heading - 0) <= HEADING_ERROR || Math.abs(heading - Math.PI*2) <= HEADING_ERROR){
            yDir = 1; // heading positive y direction
        } else if (Math.abs(heading - Math.PI/2.0) <= HEADING_ERROR){
            xDir = 1; // heading positive x direction
        } else if (Math.abs(heading - Math.PI) <= HEADING_ERROR){
            yDir = -1; // heading negative y direction
        } else if (Math.abs(heading - (Math.PI + Math.PI/2.0)) <= HEADING_ERROR){
            xDir = -1; // heading negative x direction
        }

        // check which detection event happened first
        if(leftDetectleftTacho < rightDetectleftTacho){
            // left detection occured first.. ie bot is heading slightly right from perpendicular
            // Therefore side line of triangle is on right side so use rigthTachos for calculating this side distance
            lineInterceptAngle = calcLineInterceptAngle(leftDetectrightTacho, rigthDetectrightTacho);
        } else {
            // right detection occurred first.. ie bot is heading slightly left from perpendicular
            // Use leftTachos to measure side of triangle
            // multiply by -1 becuase the bot is heading slightly
            lineInterceptAngle = -1 * calcLineInterceptAngle(rightDetectleftTacho, leftDetectleftTacho);
        }

        //double lineInterceptAngle = calcLineInterceptAngle(leftDetectleftTacho, leftDetectrightTacho);
        double correctedHeading = 0;

        // robot is driving parallel to yAxis.. therefore angle is near 0 or 180
        //boolean yDir = (Math.PI/4 - Math.abs(this.odometer.getTheta() % Math.PI - Math.PI/4)) < HEADING_ERROR;
        if(yDir == 1){
            correctedHeading = lineInterceptAngle;
        } else if(yDir == -1){
            correctedHeading = lineInterceptAngle + Math.PI;
        } else if(xDir == 1){
            correctedHeading = lineInterceptAngle + Math.PI/2;
        } else if(xDir == -1){
            correctedHeading = lineInterceptAngle + 3 * Math.PI/2;
        }

        // update heading in the odometer..
        System.out.println("UpdatingTheta from : "+((int)(odometer.getTheta()*180/Math.PI*100))/100.0 + " to : "+((int)(correctedHeading*180/Math.PI*100))/100.0);
        this.odometer.setTheta(correctedHeading);
    }

    /**
     * Given distances (side lengths) this method executes the trig operation to return the angle enclosed in the triangle.
     * @param first
     * @param second
     * @return
     */
    private static double calcLineInterceptAngle(double first, double second){
        double deltaTheta = (second - first) * Math.PI / 180;
        double sideLength = deltaTheta * KinematicModel.WHEEL_RADIUS_L;
        return Math.atan(sideLength / KinematicModel.COLOUR_SENSOR_SPACING);
    }

    @Override
    public void run() {

        // We don't want to waste processing power or anything, so if
        // we determine that we're close to a spot where we have to correct,
        // then we do so. Otherwise, we rest for a longer period.
        while(true){

            if(checkCorrectionCandidate()){
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
