package ca.mcgill.ecse211.dreamteamrobot.brick1.ballloader;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Driver;
import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Main;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import com.sun.beans.util.Cache;
import lejos.robotics.mapping.NavigationModel;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

/**
 * Created by Angus on 4/5/2016.
 */
public class BallLoader {

    public enum STATES{OPEN, CLOSED, HOLDING_RED, HOLDING_BLUE, MANEUVERING};

    private static final double inchToCm = 2.54;

    private Connection brick2;
    private Connection comp;
    private Driver driver;

    private boolean[] availableBalls;
    private double[][] ballCoordinates;
    private STATES state;

    private int TargetBall;
    private KinematicModel.BALL_COLORS TargetColor;

    /**
     *
     * @param brick2
     * @param comp
     * @param driver
     */
    public BallLoader(Connection brick2, Connection comp, Driver driver, KinematicModel.BALL_COLORS targColor, double[][] ballPlatformCorners){
        // initialize the state
        this.state = STATES.CLOSED;
        this.availableBalls = new boolean[]{true, true, true, true};
        this.TargetBall = 0;
        this.TargetColor = targColor;
        this.ballCoordinates = computeCoordsOfBalls(ballPlatformCorners[0], ballPlatformCorners[1]);


        this.brick2 = brick2;
        this.comp = comp;
        this.driver = driver;

        // initialize the queues as needed :
        this.brick2.queue.registerQueue(KinematicModel.ROUTES.CLAWS_CLOSED.toString());
        this.brick2.queue.registerQueue(KinematicModel.ROUTES.CLAWS_MOVED.toString());
        this.brick2.queue.registerQueue(KinematicModel.ROUTES.BALL_COLOUR.toString());
        this.brick2.queue.registerQueue(KinematicModel.ROUTES.FINISHED_SHOOT.toString());
    }

    /**
     * Iterates the navigating, loading and shooting operations for all 4 balls
     */
    public void shootAllBalls(){
        while(this.TargetBall <= 3){
            System.out.println("fetching ball : "+this.TargetBall);
            fetchBall();

            // validate if the ball is the right color
            boolean blue = (this.state == STATES.HOLDING_BLUE && this.TargetColor == KinematicModel.BALL_COLORS.BLUE);
            boolean red = (this.state == STATES.HOLDING_RED && this.TargetColor == KinematicModel.BALL_COLORS.RED);
            if(red || blue){
                // holding a valid ball
                System.out.println("shooting ball : "+this.TargetBall);
                moveToShoot();
            } else {
                // throw away the ball
                System.out.println("throwing ball : "+this.TargetBall);
                moveToThrowAway();
            }
            shootBall();
        }
    }

    /**
     * Given the current target ball, this method will
     *  - navigate based on the best approach
     *  - close the clasps
     *  - read and determine the ball color
     *  - updates class variables
     */
    public void fetchBall(){
        // move to closing position in front of target ball
        moveToTargetBall();

        // close the clasp
        closeGripsSync();

        this.state = STATES.CLOSED;

        //@TODO : back away from platform

        // read ball color
        KinematicModel.BALL_COLORS ballC = readBallColour(true);

        // update constants and states etc
        if(ballC == KinematicModel.BALL_COLORS.BLUE){
            this.state = STATES.HOLDING_BLUE;
        } else {
            this.state = STATES.HOLDING_RED;
        }
        this.TargetBall ++;
    }

    /**
     * Moves robot to shooting position
     */
    public void moveToShoot(){
        Navigator tempNav = driver.getNavigator();

        // move to firing position..
        tempNav.travelTo(KinematicModel.SHOOTING_POS[0], KinematicModel.SHOOTING_POS[1]);
        while (tempNav.isNavigating()){
            Main.pause(500);
        }

        // aim the robot..
        tempNav.turnToAngle(((double)KinematicModel.SHOOTING_HEADING) * Math.PI / 180.0);
        while (tempNav.isNavigating()){
            Main.pause(500);
        }
    }

    /**
     * Turns robot so that it can throw away an invalid ball
     */
    public void moveToThrowAway(){
        Navigator tempNav = driver.getNavigator();

        // aim the robot..
        tempNav.turnToAngle(((double)KinematicModel.THROWAWAY_HEADING) * Math.PI / 180.0);
        while (tempNav.isNavigating()){
            Main.pause(500);
        }
    }

    /**
     * Executes the shooting of the ball operation
     */
    public void shootBall(){
        JSONObject shootMsg = new JSONObject();
        this.brick2.out.sendJSONObj(KinematicModel.ROUTES.EXECUTE_SHOOT.toString(), shootMsg);

        this.brick2.queue.get(KinematicModel.ROUTES.FINISHED_SHOOT.toString()).clear();
        while(this.brick2.queue.get(KinematicModel.ROUTES.FINISHED_SHOOT.toString()).pollLast() == null){
            Main.pause(200);
        }
    }

    /**
     * Given the lowerLeft and UpperRight Corners, will return the coordinates of the centers of all the balls
     * @param lowerLeft double[] = {xCoord, yCoord}
     * @param upperRight double[] = {xCoord, yCoord}
     * @return double[][] = {{coordsOfBall1}, {coordsOfBall2}}
     */
    public double[][] computeCoordsOfBalls(double[] lowerLeft, double[] upperRight){

        double xDist = upperRight[0] - lowerLeft[0];
        double yDist = upperRight[1] - lowerLeft[1];

        if(xDist > yDist){
            // balls are along the xAxis
            double[][] x = {
                    {lowerLeft[0] + 1.5 * inchToCm, lowerLeft[1] + 1.5 * inchToCm},
                    {lowerLeft[0] + (3 + 1.5) * inchToCm, lowerLeft[1] + 1.5 * inchToCm},
                    {lowerLeft[0] + (6 + 1.5) * inchToCm, lowerLeft[1] + 1.5 * inchToCm},
                    {lowerLeft[0] + (9 + 1.5) * inchToCm, lowerLeft[1] + 1.5 * inchToCm}
            };
            return x;

        } else {
            // balls are along the yAxis
            double[][] y = {
                    {lowerLeft[0] + 1.5 * inchToCm, lowerLeft[1] + 1.5 * inchToCm},
                    {lowerLeft[0] + 1.5 * inchToCm, lowerLeft[1] + (3 + 1.5) * inchToCm},
                    {lowerLeft[0] + 1.5 * inchToCm, lowerLeft[1] + (6 + 1.5) * inchToCm},
                    {lowerLeft[0] + 1.5 * inchToCm, lowerLeft[1] + (9 + 1.5) * inchToCm}
            };
            return y;
        }
    }

    /**
     *
     * @param ballIndex int
     * @param ballLocationsArr double[][] = {{coordsOfBall1}, {coordsOfBall2}}
     * @return
     */
    public double[][] getBestApproach(int ballIndex, double[][] ballLocationsArr){
        // y coordinates are the same for balls 1 and 2.. Therefore the platform is lying along the xAxis
        boolean isXAxis = (ballLocationsArr[0][1] == ballLocationsArr[1][1]);

        if(isXAxis){
            // Line up with the ball's xCoordinate and approach from below
            double[] first = {ballLocationsArr[ballIndex][0], ballLocationsArr[ballIndex][1] - KinematicModel.APPROACH_DISTANCE};
            double[] end = {ballLocationsArr[ballIndex][0], ballLocationsArr[ballIndex][1] - KinematicModel.GRAB_DISTANCE};

            double[][] a = {first, end};
            return a;
        } else {
            // Line up with the ball's yCoordinate and approach from the left
            double[] first = {ballLocationsArr[ballIndex][0] - KinematicModel.APPROACH_DISTANCE, ballLocationsArr[ballIndex][1]};
            double[] end = {ballLocationsArr[ballIndex][0] - KinematicModel.GRAB_DISTANCE, ballLocationsArr[ballIndex][1]};

            double[][] a = {first, end};
            return a;
        }
    }

    public void closeGripsSync(){
        JSONObject closeAngle = new JSONObject();
        closeAngle.put("claws_angle", 0);
        this.brick2.out.sendJSONObj(KinematicModel.ROUTES.CLAWS_MOVE.toString(), closeAngle);

        this.brick2.queue.get(KinematicModel.ROUTES.CLAWS_MOVED.toString()).clear();
        while(this.brick2.queue.get(KinematicModel.ROUTES.CLAWS_MOVED.toString()).pollLast() == null){
            Main.pause(200);
        }
    }

    public void incrementTargetBall(){
        try{
            this.availableBalls[this.TargetBall] = false;
        } catch (ArrayIndexOutOfBoundsException e){
            e.printStackTrace();
        }

        this.TargetBall ++;
    }

    public KinematicModel.BALL_COLORS readBallColour(boolean waitUntilDone){

        //this.brick2.queue.get("BALL_COLOUR").clear();
        this.brick2.out.sendJSONObj("READ_BALL", null);

        if(waitUntilDone){
            JSONObject response = (JSONObject)this.brick2.queue.popJSON(KinematicModel.ROUTES.BALL_COLOUR.toString());
            while(response == null){
                Main.pause(200);
                response = this.brick2.queue.popJSON(KinematicModel.ROUTES.BALL_COLOUR.toString());
            }
            JSONArray jsonArr = (JSONArray)response.get("ball_colour");
            double[] javaArr = new double[jsonArr.size()];
            for(int i=0;i<jsonArr.size();i++){
                javaArr[i] = Double.parseDouble(jsonArr.get(i).toString());
            }
            double blueDist = euclidDist(KinematicModel.BLUE_RGB, javaArr);
            double redDist = euclidDist(KinematicModel.RED_RGB, javaArr);

            if(blueDist < redDist){
                this.state = STATES.HOLDING_BLUE;
                return KinematicModel.BALL_COLORS.BLUE;
            } else {
                this.state = STATES.HOLDING_RED;
                return KinematicModel.BALL_COLORS.RED;
            }
        }

        // if not synchronous
        return null;
    }

    private double euclidDist(double[] a, double[] b){
        double squaresSum = 0;
        for (int i=0;(i<a.length && i<b.length);i++){
            squaresSum += Math.pow(a[i]-b[i], 2);
        }
        return Math.sqrt(squaresSum);
    }

    public void moveToTargetBall(){
        Navigator tempNav = driver.getNavigator();

        //@TODO : need to set up better approach than driving right in to the ball..
        double[][] wayPoints = getBestApproach(this.TargetBall, this.ballCoordinates);

        // line up with target ball..
        tempNav.travelTo(wayPoints[0][0], wayPoints[0][1]);
        while (tempNav.isNavigating()){
            Main.pause(500);
        }

        // slowly approach the ball
        tempNav.travelTo(wayPoints[1][0], wayPoints[1][1]);
        while (tempNav.isNavigating()){
            Main.pause(500);
        }
    }

    public STATES getState() {
        return this.state;
    }

    public void setState(STATES state) {
        this.state = state;
    }

    public int getTargetBall() {
        return TargetBall;
    }

    public void setTargetBall(int TargetBall) {
        this.TargetBall = TargetBall;
    }

}
