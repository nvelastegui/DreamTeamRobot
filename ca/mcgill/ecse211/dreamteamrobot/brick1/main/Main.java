package ca.mcgill.ecse211.dreamteamrobot.brick1.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.ballloader.BallLoader;
import ca.mcgill.ecse211.dreamteamrobot.brick1.communication.Communication;
import ca.mcgill.ecse211.dreamteamrobot.brick1.display.LCDDisplay;
import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.localization.Localization;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import ca.mcgill.ecse211.dreamteamrobot.brick1.wifi.WifiConnection;
import ca.mcgill.ecse211.dreamteamrobot.connection.Queue;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import org.json.simple.JSONObject;

import java.io.IOException;
import java.util.HashMap;

/**
 * Central class.
 */
public class Main {

	// /** Constants */
	private static final TextLCD t = LocalEV3.get().getTextLCD();

	// /** Constants: Motor Ports */

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor leftUltrasonicSensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightUltrasonicSensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));


	// /** Constants: Sensor Ports */
	private static final Port leftUltrasonicSensorPort = LocalEV3.get().getPort("S1");
	private static final Port rightUltrasonicSensorPort = LocalEV3.get().getPort("S2");
	private static final Port leftColorSensorPort = LocalEV3.get().getPort("S3");
	private static final Port rightColorSensorPort = LocalEV3.get().getPort("S4");


	// /** Constants: Communication Objects */
	private static Connection brick2;
	private static Connection comp;

	/**
	 * Before anything can begin to run on the robot, the robot needs to know if it's playing offensive or defensive.
	 * This Main class awaits the signal with details regarding the match, then runs the appropriate threads.
	 * @param args arggggg matey
	 * @throws InterruptedException
     */
	public static void main(String[] args) throws InterruptedException {

		/** Begin initializing communications. */
		t.clear();
		t.drawString("< DreamTeamRobot >", 0, 0);
		t.drawString("<                >", 0, 1);
		t.drawString("<  Initializing  >", 0, 2);
		t.drawString("<  Communication >", 0, 3);
		t.drawString("<                >", 0, 4);
		t.drawString("<                >", 0, 5);


		comp = connectToComp();
		if(comp != null){
			JSONObject welcomeMsg = new JSONObject();
			welcomeMsg.put("client", "brick1");
			comp.out.sendJSONObj("CLIENT_CONNECTED", welcomeMsg);
		}


		brick2 = connectToBrick2();
		if(comp != null){
			JSONObject welcomeMsg2 = new JSONObject();
			comp.out.sendJSONObj("BRICKS_CONNECTED", welcomeMsg2);
		}

		/** Connect to WIFI. */
//		boolean safeToContinue = connectToWifi();
//		// If connecting to wifi proved perilous, then wait for ESC button
//		// to quit program.
//		if (!safeToContinue) {
//			Button.ESCAPE.waitForPress();
//			return;
//		}
		HashMap<String, Integer> spoofRoundData = new HashMap<>();
		spoofRoundData.put("SC", 1);
		spoofRoundData.put("Role", 0);
		spoofRoundData.put("w1", 4);
		spoofRoundData.put("d1", 5);
		spoofRoundData.put("d2", 5);
		spoofRoundData.put("ll-x", 5);
		spoofRoundData.put("ll-y", 1);
		spoofRoundData.put("ur-x", 6);
		spoofRoundData.put("ur-y", 2);
		spoofRoundData.put("BC", 2);
		KinematicModel.roundData = spoofRoundData;

		/** Do Brick 1 Driver setup stuff */
		// Create a new instance of driver thread.
		Driver driver = new Driver(
				leftMotor,
				rightMotor,
				leftUltrasonicSensorMotor,
				rightUltrasonicSensorMotor,
				leftUltrasonicSensorPort,
				rightUltrasonicSensorPort,
				leftColorSensorPort,
				rightColorSensorPort
		);

		// Run pre-execute procedure (starts subthreads).
		driver.performPreExecute();

		// initialize communication and handle incoming messages
		Communication com = new Communication(driver, brick2, comp);
		com.setCYCLE_TIME(1000);
		com.start();

		// Set up lcdDisplay and run it.
		LCDDisplay lcdDisplay = new LCDDisplay(driver);
		lcdDisplay.start();

		/** Do Localization */
		getLocalized(driver);

		/** Start stuff that wasn't started because it interfered with localization */
		// Start odometry correction.
		driver.getNavigator().setThetaToleranceHigh();
		driver.getOdometerCorrection().start();

		// Start obstacle avoidance on navigator.
		driver.getNavigator().setObstacleAvoidanceOn();

		/** Initialize the BallLoader */
		driver.initializeBallLoader(brick2, comp);

		/** START OFFENSIVE! */
		driver.turnOn();


	}

	/**
	 * Performs localization (method is just abstraction layer).
	 * @param driver driver.
     */
	private static void getLocalized (Driver driver) {
		Localization localizer = new Localization(
				leftMotor,
				rightMotor,
				driver.getOdometer(),
				driver.getNavigator(),
				driver.getUltrasonicPollerLeft(),
				driver.getUltrasonicPollerRight(),
				driver.getColourPollerLeft(),
				driver.getColourPollerRight(),
				driver.getOdometerCorrection()
		);
		localizer.setupLocalizer();
		localizer.doLocalization(null);
	}


	/**
	 * Initialized the connection with brick2. Returns a connection object or
	 * @return Connection object with brick2
     */
	private static Connection connectToBrick2(){
		Connection brick2 = new Connection();

		// wait for brick2 to connect.. 10 second timeout
		boolean success = brick2.accept(KinematicModel.BRICK_PORT, KinematicModel.BRICK_TIMEOUT);
		if(success){
			brick2.queue = new Queue(KinematicModel.ROUTE_PROPERTY);
			brick2.listen(100);

			return brick2;
		} else {
			return null;
		}
	}

	/**
	 * Initialized the connection with comp. Returns a connection object
	 * @return Connection object with comp
	 */
	private static Connection connectToComp(){
		Connection comp = new Connection();

		// wait for comp to connect.. 10 second timeout
		boolean successfullyConnected = comp.connect(KinematicModel.COMP_HOST, KinematicModel.COMP_PORT);
		if(successfullyConnected){
			comp.queue = new Queue(KinematicModel.ROUTE_PROPERTY);
			comp.listen(500);
			return comp;
		} else {
			return null;
		}
	}

	/**
	 * Connects to server wifi to retrieve initialization info.
	 * @return A HashMap containing stat data, if connection is successful. Null if connection is not successful.
     */
	private static HashMap<String, Integer> retrieveStartData () {

		try {
			WifiConnection conn = new WifiConnection(KinematicModel.WIFI_SERVER_IP, KinematicModel.TEAM_NUMBER);
			return conn.StartData;
		} catch (IOException e) {
			return null;
		}

	}

	/**
	 * Performs initialization procedures relevant to wifi connection. Puts start data into KinematicModel.roundData.
	 * @return True if operations successful and program can move on. False if
	 * operations not successful and program cannot move on.
     */
	private static boolean connectToWifi () {

		// Display message on screen.
		t.clear();
		t.drawString("<DreamTeamRobot >", 0, 0);
		t.drawString("<               >", 0, 1);
		t.drawString("< Connecting to >", 0, 2);
		t.drawString("<     Wifi      >", 0, 3);
		t.drawString("<               >", 0, 4);
		t.drawString("<               >", 0, 5);

		// Try wifi connection.
		HashMap<String, Integer> startData = retrieveStartData();
		if (startData == null) {
			// If there is an error connecting to wifi, return false.
			t.clear();
			t.drawString("<DreamTeamRobot >", 0, 0);
			t.drawString("< Connecting to >", 0, 1);
			t.drawString("<    Wifi       >", 0, 2);
			t.drawString("<     FAILED    >", 0, 3);
			t.drawString("<               >", 0, 4);
			t.drawString("<  ESC to quit  >", 0, 5);
			return false;
		}

		// If there are no problems, then save the round data into Kinematic Model.
		KinematicModel.roundData = startData;
		// and return true.
		return true;

	}

	/**
	 * Pauses thread to allow for wheel motions to finish.
	 * @param timeToStop amount of time to stop, in milliseconds.
	 */
	public static void pause(int timeToStop) {
		try {
			Thread.sleep(timeToStop);
		} catch (InterruptedException e) {
			e.printStackTrace();
			Sound.beep();
			Sound.twoBeeps();
		}
	}

}






















