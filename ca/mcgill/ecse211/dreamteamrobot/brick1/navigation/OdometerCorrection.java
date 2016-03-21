package ca.mcgill.ecse211.dreamteamrobot.brick1.navigation;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;

/**
 * Thread simultaneous to Odometer. This thread reads values from the environment (ie. color sensor values)
 * and compares them with current values in the Odometer so that it can perform any necessary corrections
 * for improved navigational precision.
 */
public class OdometerCorrection extends Thread {

    private int LONG_SLEEP = 200;
    private int SHORT_SLEEP = 50;

    // HEADING_ERROR is how closely the robot's heading must be to perpendicular to the line in order to initiate correction
    // ie : HEADING_ERROR of 0 would mean the robots heading would have to be a perfect 90deg from the line it is crossing
    private double HEADING_ERROR = 15 * Math.PI / 180;
    private int POSITION_ERROR = 5;
    // MAX_DELTA_TACHO : amount of error between the right and left wheel tachos when they each detect their lines.
    // if the difference in distance travelled between the 2 wheels exceeds this # the correction attempt is cancelled and tachos are reset.
    private double MAX_DELTA_TACHO = HEADING_ERROR * KinematicModel.WHEELBASE / KinematicModel.WHEEL_RADIUS_L;

    private boolean[] xUpdateArr = {true, false, false};
    private boolean[] yUpdateArr = {false, true, false};

    private Odometer odometer;

    private double rightTacho;
    private double leftTacho;

    public OdometerCorrection (Odometer odometer) {
        this.leftTacho = 0;
        this.rightTacho = 0;

        this.odometer = odometer;
    }

    private static double calcDeltaHeading(double left, double right){
        double deltaTheta = left - right;
        double sideLength = deltaTheta * KinematicModel.WHEEL_RADIUS_L;
        return Math.atan(sideLength / KinematicModel.WHEELBASE);
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
    }

    private static double getDistanceAdjust(double odoPosition) {
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
        double headingErr = 45 - Math.abs(this.odometer.getTheta() % 90 - 45);
        boolean headingValid = headingErr < HEADING_ERROR;

        // check if coords are within acceptable distances
        double xErr = 15 - Math.abs(this.odometer.getX()%30 - 15);
        double yErr = 15 - Math.abs(this.odometer.getX()%30 - 15);

        boolean coordValid = xErr < POSITION_ERROR || yErr < POSITION_ERROR;

        return coordValid && headingValid;
    }

    private void beginOdoCorrection(){

        // poll left color sensor
        boolean leftSensorHitLine = false;
        if(leftSensorHitLine){
            this.leftTacho = odometer.getLeftMotor().getTachoCount();
        }
        // poll right color sensor
        boolean rightSensorHitLine = false;
        if(rightSensorHitLine){
            this.rightTacho = odometer.getRightMotor().getTachoCount();
        }

        if(leftTacho != 0 && rightTacho != 0){
            // account for possibility that the bot was turning or something and the tachos are very different
            if(Math.abs(leftTacho - rightTacho) > MAX_DELTA_TACHO){
                leftTacho = 0;
                rightTacho = 0;
                return;
            }

            // calculate for heading
            double deltaHeading = calcDeltaHeading(leftTacho, rightTacho);

            // correct for coordinates
            correctCoords();

            // reset line tacho counts
            leftTacho = 0;
            rightTacho = 0;
        }
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
