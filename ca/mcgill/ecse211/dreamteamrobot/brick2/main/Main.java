package ca.mcgill.ecse211.dreamteamrobot.brick2.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import ca.mcgill.ecse211.dreamteamrobot.connection.Queue;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import org.json.simple.JSONObject;

/**
 * Central class.
 */
public class Main {

    // /** Constants: Communication Objects */
    private static Connection brick1;
    private static Connection comp;

    /** Static Resources */
    private static EV3LargeRegulatedMotor shootMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    private static EV3LargeRegulatedMotor claspMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    private static Port colourSensorPort = LocalEV3.get().getPort("S1");
    private static TextLCD t = LocalEV3.get().getTextLCD();

	public static void main(String[] args) throws InterruptedException {


        // Set initial diplay.
        t.clear();
        t.drawString("< DreamTeamRobot >", 0, 0);
        t.drawString("<                >", 0, 1);
        t.drawString("<  Initializing  >", 0, 2);
        t.drawString("<  Communication >", 0, 3);
        t.drawString("<                >", 0, 4);
        t.drawString("<                >", 0, 5);

        comp = connectToComp();
        JSONObject welcomeMsg = new JSONObject();
        welcomeMsg.put("client", "brick2");
        comp.out.sendJSONObj("CLIENT_CONNECTED", welcomeMsg);


        brick1 = connectToBrick1();

        // Display initial screen
        t.clear();
        t.drawString("<DreamTeamRobot >", 0, 0);
        t.drawString("<  Brick 2      >", 0, 1);
        t.drawString("<  Initializing >", 0, 2);
        t.drawString("<   Classes     >", 0, 3);
        t.drawString("<               >", 0, 4);
        t.drawString("<               >", 0, 5);

        // Await some orders?

        // Initialize threads
        Gunner gunner = new Gunner(shootMotor, claspMotor, colourSensorPort, brick1, comp);
        gunner.performPreExecute();
        gunner.openClasp();

        // start listening for messages
        gunner.start();

        // Debugging: Testing shooting mechanism.
//        while (true) {
//
//
//
//            // Display home screen prompt.
//            // Wait for user to press button.
//            t.drawString("Super Ultra Mega", 0, 0);
//            t.drawString("    Catapult    ", 0, 1);
//            t.drawString("                ", 0, 2);
//            t.drawString("Press any button", 0, 3);
//            t.drawString("to shoot the    ", 0, 4);
//            t.drawString("load.           ", 0, 5);
//
//            Button.waitForAnyPress();
//
//            // Shoot
//            t.drawString("Super Ultra Mega", 0, 0);
//            t.drawString("    Catapult    ", 0, 1);
//            t.drawString("                ", 0, 2);
//            t.drawString("    Shooting    ", 0, 3);
//            t.drawString("    ********    ", 0, 4);
//            t.drawString("                ", 0, 5);
//            gunner.executeShoot();
//
//            // Wait
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//
//        }

	}

    /**
     * Initialized the connection with brick2. Returns a connection object
     * @return Connection object with brick2
     */
    private static Connection connectToBrick1(){
        Connection brick1 = new Connection();

        // wait for brick1 to connect.. 10 second timeout
        brick1.connect(KinematicModel.BRICK1_HOST, KinematicModel.BRICK_PORT);
        brick1.queue = new Queue(KinematicModel.ROUTE_PROPERTY);

        brick1.listen(100);

        return brick1;
    }

    /**
     * Initialized the connection with comp. Returns a connection object
     * @return Connection object with comp
     */
    private static Connection connectToComp(){
        Connection comp = new Connection();

        // wait for comp to connect.. 10 second timeout
        comp.connect(KinematicModel.COMP_HOST, KinematicModel.COMP_PORT);
        comp.queue = new Queue(KinematicModel.ROUTE_PROPERTY);

        comp.listen(500);

        System.out.println("brick2 Connected to comp and listening");

        return comp;
    }


}






















