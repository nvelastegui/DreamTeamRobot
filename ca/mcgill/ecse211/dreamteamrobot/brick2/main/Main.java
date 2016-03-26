package ca.mcgill.ecse211.dreamteamrobot.brick2.main;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Central class.
 */
public class Main {

    /** Static Resources */
    private static EV3LargeRegulatedMotor shootMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    private static EV3LargeRegulatedMotor claspMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    private static Port colourSensorPort = LocalEV3.get().getPort("S1");
    private static TextLCD t = LocalEV3.get().getTextLCD();

	public static void main(String[] args) throws InterruptedException {

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
        Gunner gunner = new Gunner(shootMotor, claspMotor, colourSensorPort);
        gunner.performPreExecute();
        gunner.openClasp();

        // Debugging: Testing shooting mechanism.
        while (true) {

            // Display home screen prompt.
            // Wait for user to press button.
            t.drawString("Super Ultra Mega", 0, 0);
            t.drawString("    Catapult    ", 0, 1);
            t.drawString("                ", 0, 2);
            t.drawString("Press any button", 0, 3);
            t.drawString("to shoot the    ", 0, 4);
            t.drawString("load.           ", 0, 5);

            Button.waitForAnyPress();

            // Shoot
            t.drawString("Super Ultra Mega", 0, 0);
            t.drawString("    Catapult    ", 0, 1);
            t.drawString("                ", 0, 2);
            t.drawString("    Shooting    ", 0, 3);
            t.drawString("    ********    ", 0, 4);
            t.drawString("                ", 0, 5);
            gunner.executeShoot();

            // Wait
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

	}


}






















