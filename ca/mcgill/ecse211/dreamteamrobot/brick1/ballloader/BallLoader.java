package ca.mcgill.ecse211.dreamteamrobot.brick1.ballloader;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.localization.Localization;
import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Driver;
import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Main;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Location;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.pathfinding.PathFinder;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import java.util.List;

/**
 * Created by Angus on 4/5/2016.
 */
public class BallLoader {

    public enum STATES{OPEN, CLOSED, HOLDING_RED, HOLDING_BLUE, MANEUVERING};

    private static final double inchToCm = 2.54;

    private Connection brick2;
    private Connection comp;
    private Navigator navigator;

    private boolean[] availableBalls;
    private double[][] ballCoordinates;
    private STATES state;

    private int TargetBall;
    private KinematicModel.BALL_COLORS TargetColor;

    /**
     *
     * @param brick2
     * @param comp
     * @param navigator
     */
    public BallLoader(Connection brick2, Connection comp, Navigator navigator){
        // initialize the state
        this.state = STATES.CLOSED;
        //this.availableBalls = new boolean[]{true, true, true, true};
        this.TargetBall = 0;
        if(KinematicModel.roundData.get("BC") == 0){
            this.TargetColor = KinematicModel.BALL_COLORS.RED;
        } else if(KinematicModel.roundData.get("BC") == 1){
            this.TargetColor = KinematicModel.BALL_COLORS.BLUE;
        } else {
            this.TargetColor = KinematicModel.BALL_COLORS.ANY;
        }

        int ll_x = KinematicModel.roundData.get("ll-x"); // [-1, 11] -> if it's 10, corresponds to box 11 -> [0, 11]
        int ll_y = KinematicModel.roundData.get("ll-y"); // [-1, 11] -> if it's 10, corresponds to box 11 -> [0, 11]
        int ur_x = KinematicModel.roundData.get("ur-x"); // [-1, 11] -> [0, 11]
        int ur_y = KinematicModel.roundData.get("ur-y"); // [-1, 11] -> [0, 11]

        double[] ll = {ll_x*30, ll_y*30};
        double[] ur = {ur_x*30, ur_y*30};

        this.ballCoordinates = computeCoordsOfBalls(ll, ur);


        this.brick2 = brick2;
        this.comp = comp;
        this.navigator = navigator;

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

    public boolean isValidBall(){
        // validate if the ball is the right color
        boolean blue = (this.state == STATES.HOLDING_BLUE && this.TargetColor == KinematicModel.BALL_COLORS.BLUE);
        boolean red = (this.state == STATES.HOLDING_RED && this.TargetColor == KinematicModel.BALL_COLORS.RED);
        boolean any = this.TargetColor == KinematicModel.BALL_COLORS.ANY;
        return (red || blue || any);
    }

    /**
     * Given the current target ball, this method will
     *  - navigate based on the best approach
     *  - close the clasps
     *  - read and determine the ball color
     *  - updates class variables
     */
    public void fetchBall(){

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
        Navigator tempNav = navigator;

        // move to firing position..
        travelToPath(KinematicModel.SHOOTING_POS[0], KinematicModel.SHOOTING_POS[1]);
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
        Navigator tempNav = navigator;

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
     * Given the index of the target ball and the locations of all balls this method will calculate the best approach inorder to maximize success rate when grabbing the balls
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

    /**
     * Closes the clasp on brick2 in a synchronous fasion.
     */
    public void closeGripsSync(){
        JSONObject closeAngle = new JSONObject();
        closeAngle.put("claws_angle", 0);
        this.brick2.out.sendJSONObj(KinematicModel.ROUTES.CLAWS_MOVE.toString(), closeAngle);

        this.brick2.queue.get(KinematicModel.ROUTES.CLAWS_MOVED.toString()).clear();
        while(this.brick2.queue.get(KinematicModel.ROUTES.CLAWS_MOVED.toString()).pollLast() == null){
            Main.pause(200);
        }
    }

    /**
     * Increments the target ball
     */
    public void incrementTargetBall(){
        try{
            this.availableBalls[this.TargetBall] = false;
        } catch (ArrayIndexOutOfBoundsException e){
            e.printStackTrace();
        }

        this.TargetBall ++;
    }

    /**
     * Synchronous RTP call to brick2 to read the color of the ball and report the RGB array back as structured JSON
     * @param waitUntilDone
     * @return
     */
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

    /**
     * Calculate the euclidean distance between 2 arbitrarily sized arrays
     * @param a
     * @param b
     * @return
     */
    private double euclidDist(double[] a, double[] b){
        double squaresSum = 0;
        for (int i=0;(i<a.length && i<b.length);i++){
            squaresSum += Math.pow(a[i]-b[i], 2);
        }
        return Math.sqrt(squaresSum);
    }

    /**
     * Given the target ball, this will execute path planning and travelTo function to navigate to the ball. Executes bestApproach
     */
    public void moveToTargetBall(){

        //@TODO : need to set up better approach than driving right in to the ball..
        double[][] wayPoints = getBestApproach(this.TargetBall, this.ballCoordinates);
        System.out.println("b");

        travelToPath(wayPoints[0][0], wayPoints[0][1]);

        // slowly approach the ball
        navigator.travelTo(wayPoints[1][0], wayPoints[1][1]);
        while (navigator.isNavigating()){
            Main.pause(500);
        }
    }

    /**
     * Synchronously executes generating a path to a target way point and carrying out the traveling of the returned path.
     * @param xCoord
     * @param yCoord
     */
    private void travelToPath(double xCoord, double yCoord){
        Odometer tempOdo = navigator.getOdometer();

        // line up with target ball..
        List<Location> pathPoints = PathFinder.generatePath(
                new Location(tempOdo.getX(), tempOdo.getY()),
                new Location(xCoord, yCoord)
        );

        if(pathPoints == null){
            return;
        }

        for (Location current:pathPoints) {
            navigator.travelTo(current.getX(), current.getY());
            while (navigator.isNavigating()){
                Main.pause(500);
            }
        }
    }

    /**
     * getter for state.
     * @return
     */
    public STATES getState() {
        return this.state;
    }

    /**
     * setter for state.
     * @param state
     */
    public void setState(STATES state) {
        this.state = state;
    }

    /**
     * getter for TargetBall.
     * @return
     */
    public int getTargetBall() {
        return TargetBall;
    }

    /**
     * setter for TargetBall.
     * @param TargetBall
     */
    public void setTargetBall(int TargetBall) {
        this.TargetBall = TargetBall;
    }

}
