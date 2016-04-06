package ca.mcgill.ecse211.dreamteamrobot.brick1.ballloader;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Driver;
import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Main;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import lejos.robotics.mapping.NavigationModel;
import org.json.simple.JSONObject;

/**
 * Created by Angus on 4/5/2016.
 */
public class BallLoader {

    public static enum STATES{OPEN, CLOSED, HOLDING_RED, HOLDING_BLUE, MANEUVERING};

    private Connection brick2;
    private Connection comp;
    private Driver driver;

    private boolean[] availableBalls;
    private double[][] ballCoordinates;
    private STATES state;

    private int TargetBall;

    public BallLoader(Connection brick2, Connection comp, Driver driver){
        // initialize the state
        this.state = STATES.CLOSED;
        this.availableBalls = new boolean[]{true, true, true, true};
        this.TargetBall = 0;

        this.brick2 = brick2;
        this.comp = comp;
        this.driver = driver;

        // initialize the queues as needed :
        this.brick2.queue.registerQueue("CLAWS_CLOSED");
        this.brick2.queue.registerQueue("BALL_COLOUR");
    }

    public double[][] computeCoordsOfBalls(double[] lowerLeft, double[] upperRight){
        double[][] a = {{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0}};
        return a;
    }

    public void closeGripsSync(boolean waitUntilDone){
        JSONObject closeAngle = new JSONObject();
        closeAngle.put("claws_angle", -KinematicModel.CLASP_MOTOR_ROTATION_ANGLE_FOR_OPEN_OR_CLOSE);
        this.brick2.out.sendJSONObj("CLAWS_MOVE", closeAngle);

        if(waitUntilDone){
            while(this.brick2.queue.get("CLAWS_CLOSED").pollLast() == null){
                Main.pause(200);
            }
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
            JSONObject response = (JSONObject)this.brick2.queue.get(KinematicModel.ROUTES.BALL_COLOUR.toString()).pollLast();
            while(response == null){
                Main.pause(200);
                response = (JSONObject)this.brick2.queue.get(KinematicModel.ROUTES.BALL_COLOUR.toString()).pollLast();
            }
            double blueDist = euclidDist(KinematicModel.BLUE_RGB, (double[])response.get("ball_colour"));
            double redDist = euclidDist(KinematicModel.RED_RGB, (double[])response.get("ball_colour"));

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
        for (int i=0;i<a.length;i++){
            squaresSum += Math.pow(a[i]-b[i], 2);
        }
        return Math.sqrt(squaresSum);
    }

    public void moveToTargetBall(boolean waitUntilDone){
        Navigator tempNav = driver.getNavigator();

        //@TODO : need to set up better approach than driving right in to the ball..
        tempNav.travelTo(this.ballCoordinates[this.TargetBall][0], this.ballCoordinates[this.TargetBall][1]);

        if(waitUntilDone){
            while (tempNav.isNavigating()){
                Main.pause(500);
            }
        }
    }

    public void shootBall(){

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
