package ca.mcgill.ecse211.dreamteamrobot.brick1.main;

import ca.mcgill.ecse211.dreamteamrobot.brick1.ballloader.BallLoader;
import ca.mcgill.ecse211.dreamteamrobot.brick1.communication.Communication;
import ca.mcgill.ecse211.dreamteamrobot.brick1.display.LCDDisplay;
import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.localization.Localization;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import ca.mcgill.ecse211.dreamteamrobot.connection.Queue;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.dreamteamrobot.connection.Connection;
import org.json.simple.JSONObject;

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
		welcomeMsg.put("client", "brick1");
		comp.out.sendJSONObj("CLIENT_CONNECTED", welcomeMsg);

		brick2 = connectToBrick2();
		JSONObject welcomeMsg2 = new JSONObject();
		comp.out.sendJSONObj("BRICKS_CONNECTED", welcomeMsg2);

		// Set initial diplay.
		t.clear();
		t.drawString("<DreamTeamRobot >", 0, 0);
		t.drawString("<               >", 0, 1);
		t.drawString("<  Initializing >", 0, 2);
		t.drawString("<   Classes     >", 0, 3);
		t.drawString("<               >", 0, 4);
		t.drawString("<               >", 0, 5);

		// Wait for signal


		// Process signal


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

		// get ball platform coordinates lying along y-axis
		double[][] ballPlatformCoords = {
				{60,60},
				{60 + 3*2.54, 60 + 12*2.54}
		};
		KinematicModel.BALL_COLORS targColor = KinematicModel.BALL_COLORS.RED;

		// initialize the BallLoader
		BallLoader ballLoader = new BallLoader(brick2, comp, driver, targColor, ballPlatformCoords);



		// Grab balls?
		/*
		Grabbing Balls is as follows :
			- Send message to brick2 to close grips to certain angle
			-
		 */

		// Shoot



	}


	/**
	 * Initialized the connection with brick2. Returns a connection object
	 * @return Connection object with brick2
     */
	private static Connection connectToBrick2(){
		Connection brick2 = new Connection();

		// wait for brick2 to connect.. 10 second timeout
		brick2.accept(KinematicModel.BRICK_PORT, KinematicModel.BRICK_TIMEOUT);
		brick2.queue = new Queue(KinematicModel.ROUTE_PROPERTY);

		brick2.listen(100);

		return brick2;
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

		return comp;
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






















