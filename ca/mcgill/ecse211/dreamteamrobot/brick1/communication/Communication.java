package ca.mcgill.ecse211.dreamteamrobot.brick1.communication;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.main.Driver;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import org.json.simple.JSONObject;

/**
 * Class containing static methods for communicating between bricks.
 */
public class Communication extends Thread{

    private Driver driver;
    private Connection brick2;
    private Connection comp;

    private int CYCLE_TIME;

    /**
     * Instantiates the Communication object that holds the Connections object for brick2 and the webserver
     * @param driver
     * @param brick2
     * @param comp
     */
    public Communication(Driver driver, Connection brick2, Connection comp){
        this.driver = driver;
        this.brick2 = brick2;
        this.comp = comp;

        if(this.comp != null) this.initializeRoutes(this.comp);
        if(this.brick2 != null) this.initializeRoutes(this.brick2);
    }

    /**
     * Registers the required routes for incoming RPC messages on a particular connection
     * @param con
     */
    private void initializeRoutes(Connection con){
        if(con == null) return;

        con.queue.registerQueue(KinematicModel.ROUTES.NAV_TO.toString());
        con.queue.registerQueue(KinematicModel.ROUTES.TURN_TO.toString());
        con.queue.registerQueue(KinematicModel.ROUTES.UPDATE_ODO.toString());
    }

    public void run(){
        while(true){


            if(this.comp != null){
                // send the heartbeat to computer
                sendHeartBeat(this.comp);

                // execute various handlers
                handleNavTo(this.comp);
                handleTurnTo(this.comp);
                handleUpdateOdo(this.comp);
            }

            if(this.brick2 != null){
                handleNavTo(this.brick2);
                handleTurnTo(this.brick2);
                handleUpdateOdo(this.brick2);
            }

            // sleep the thread
            try {
                Thread.sleep(this.CYCLE_TIME);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Broadcast HeartBeat
     * @return
     */
    private void sendHeartBeat(Connection con){

        if(con == null) return;

        Odometer odo = this.driver.getOdometer();
        Navigator nav = this.driver.getNavigator();

        JSONObject heartBeat = new JSONObject();

        // odometer information
        try{
            //JSONObject odoData = new JSONObject();
            heartBeat.put("odo_X", odo.getX());
            heartBeat.put("odo_Y", odo.getY());
            heartBeat.put("odo_theta", odo.getTheta());
            heartBeat.put("odo_heading", odo.getTheta()*180/Math.PI);
            heartBeat.put("odo_state", odo.getState().toString());
            //heartBeat.put("odometer", odoData);
        } catch (NullPointerException e){}


        // Driver data
        //JSONObject driverData = new JSONObject();
        heartBeat.put("driver_state", driver.getState().toString());
        //heartBeat.put("driver", driverData);

        // Navigator data
        try{
            //JSONObject navData = new JSONObject();
            heartBeat.put("nav_state", nav.getNavState());
            heartBeat.put("nav_X", nav.getCurrentDestination().getX());
            heartBeat.put("nav_Y", nav.getCurrentDestination().getY());
            //heartBeat.put("navigator", navData);
        } catch (NullPointerException e){}

        con.out.sendJSONObj(KinematicModel.ROUTES.HEARTBEAT_B1.toString(), heartBeat);
    }

    /**
     * Handle incoming NAV_TO messages. Change current waypoint in navigator class.
     * @param con
     */
    private void handleNavTo(Connection con){
        if(con == null) return;

        String routeName = KinematicModel.ROUTES.NAV_TO.toString();
        JSONObject incomingMsg = con.queue.popJSON(routeName);

        if(incomingMsg != null){
            System.out.println("NAV_TO : ("+incomingMsg.get("X")+","+incomingMsg.get("Y")+")" );

            Double dest_x = Double.parseDouble((String) incomingMsg.get("X"));
            Double dest_y = Double.parseDouble((String) incomingMsg.get("Y"));

            driver.getNavigator().travelTo(dest_x,dest_y);
        }
    }

    /**
     * Handle TURN_TO messages. Executes Navigator.turnToAngle
     * @param con
     */
    private void handleTurnTo(Connection con){
        if(con == null) return;

        String routeName = KinematicModel.ROUTES.TURN_TO.toString();
        JSONObject incomingMsg = con.queue.popJSON(routeName);

        if(incomingMsg != null){
            System.out.println("TURN_TO : ("+incomingMsg.get("heading")+","+incomingMsg.get("theta")+")" );

            String heading_s = (String) incomingMsg.get("heading");
            String theta_s = (String) incomingMsg.get("theta");

            if((theta_s == null || theta_s.equals("")) && (heading_s != null && !heading_s.equals(""))){
                // heading is valid
                Double heading = Double.parseDouble(heading_s);
                // convert to radians
                heading = heading / 180 * Math.PI;
                driver.getNavigator().turnToAngle(heading);
            } else if((heading_s == null || heading_s.equals("")) && (theta_s != null && !theta_s.equals(""))){
                // theta is valid
                Double theta = Double.parseDouble(theta_s);
                driver.getNavigator().turnToAngle(theta);
            }
        }
    }

    /**
     * Handle UPDATE_ODO messages to update the current value of the odometer
     * @param con
     */
    private void handleUpdateOdo(Connection con){
        if(con == null) return;

        String routeName = KinematicModel.ROUTES.UPDATE_ODO.toString();
        JSONObject incomingMsg = con.queue.popJSON(routeName);

        if(incomingMsg != null){
            System.out.println("UPDATE_ODO : ("+incomingMsg.get("X")+","+incomingMsg.get("Y")+","+incomingMsg.get("theta")+")" );

            String cur_x = (String) incomingMsg.get("X");
            String cur_y = (String) incomingMsg.get("Y");
            String cur_t = (String) incomingMsg.get("theta");

            Odometer odo = driver.getOdometer();

            if(cur_x != null && !cur_x.equals("")) {
                Double d_cur_x = Double.parseDouble(cur_x);
                odo.setX(d_cur_x);
            }
            if(cur_y != null && !cur_y.equals("")) {
                Double d_cur_y = Double.parseDouble(cur_y);
                odo.setY(d_cur_y);
            }
            if(cur_t != null && !cur_t.equals("")) {
                Double d_cur_t = Double.parseDouble(cur_t);
                odo.setTheta(d_cur_t);
            }
        }
    }

    /**
     * Getter for CYCLE_TIME. Dictates how quickly the thread cycles
     * @return
     */
    public int getCYCLE_TIME() {
        return CYCLE_TIME;
    }

    /**
     * Setter for CYCLE_TIME. Dictates how quickly the thread cycles
     * @param CYCLE_TIME
     */
    public void setCYCLE_TIME(int CYCLE_TIME) {
        this.CYCLE_TIME = CYCLE_TIME;
    }

}
