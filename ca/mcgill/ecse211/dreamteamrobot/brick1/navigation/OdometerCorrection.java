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

    private void correctCoords(){
        double heading = odometer.getTheta();
        double xDir = 0;
        double yDir = 0;
        double absAdjust, updateAmount;
        double[] updateArr = new double[3];
        if(Math.abs(heading - 0) <= HEADING_ERROR){
            xDir = 1;
        } else if (Math.abs(heading - 90) <= HEADING_ERROR){
            yDir = 1;
        } else if (Math.abs(heading - 180) <= HEADING_ERROR){
            xDir = -1;
        } else if (Math.abs(heading - 270) <= HEADING_ERROR){
            yDir = -1;
        }

        if(xDir != 0) {
            double[] doublexPos = new double[3];
            odometer.getPosition(doublexPos, xUpdateArr);
            absAdjust = getDistanceAdjust(doublexPos[0]);
            updateAmount = xDir * absAdjust;
            updateArr[0] = updateAmount + doublexPos[0];

            odometer.setPosition(updateArr, xUpdateArr);
        } else {
            double[] doubleyPos = new double[3];
            odometer.getPosition(doubleyPos, yUpdateArr);
            absAdjust = getDistanceAdjust(doubleyPos[1]);
            updateAmount = yDir * absAdjust;
            updateArr[1] = updateAmount + doubleyPos[1];

            odometer.setPosition(updateArr, yUpdateArr);
        }
        System.out.println("updated Coord to x:"+updateArr[0] + ", y:"+updateArr[1]);
    }

    private static double getDistanceAdjust(double odoPosition) {
        if(odoPosition < 0){
            return 0.0;
        }
        double previous30, distFromPrev30, closest30, totalAdjust;
        double startAdjusted = odoPosition;
        // hasn't passed a line yet.. set to first line

        previous30 = ((int) (startAdjusted / 30.0)) * 30;
        distFromPrev30 = startAdjusted - previous30;
        if(distFromPrev30 > 15) {
            // odoPosition is shorter than actual
            closest30 = previous30 + 30;
        } else {
            // odoPosition is farther than actual (bot thinks its past the line
            closest30 = previous30;
        }

        totalAdjust = closest30 - startAdjusted;
        return totalAdjust;
    }

    private boolean checkCorrectionCandidate(){
        // check if heading is within accepted heading error;
        double headingErr = Math.PI/4 - Math.abs(this.odometer.getTheta() % Math.PI/2 - Math.PI/4);
        boolean headingValid = headingErr < HEADING_ERROR;

        // check if coords are within acceptable distances
        double xErr = 15 - Math.abs(Math.abs(this.odometer.getX())%30 - 15);
        double yErr = 15 - Math.abs(Math.abs(this.odometer.getY())%30 - 15);

        boolean coordValid = xErr < POSITION_ERROR || yErr < POSITION_ERROR;
        //System.out.println("" + (coordValid && headingValid) + " --- head:"+this.odometer.getTheta()+",headingErr:"+headingErr+ " - x:"+this.odometer.getX()+",xErr:"+xErr+" - y:"+this.odometer.getY()+":yErr : "+yErr);
        return coordValid && headingValid;
    }

    private boolean colourSensorHitLine(ColourPoller cp){
        boolean hit = cp.getSensorValue() < R_THRESHOLD;
        if(hit){
            Sound.beep();
        }
        return hit;
    }

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
