package ca.mcgill.ecse211.dreamteamrobot.brick1.localization;

import ca.mcgill.ecse211.dreamteamrobot.brick1.display.LCDDisplay;
import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Navigator;
import ca.mcgill.ecse211.dreamteamrobot.brick1.navigation.Odometer;
import ca.mcgill.ecse211.dreamteamrobot.brick1.sensors.ColourPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Performs final localization assuming robot forward and sideways axes are approximately
 * perpendicular to corner walls. Moves robot to relative (0,0) and faces forward.
 */
public class LightLocalizer {

	// Instance Variables
	private Odometer odo;
	private Navigator nav;
	private ColourPoller colourPollerLeft;
	private ColourPoller colourPollerRight;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	/**
	 * Constructor.
	 * @param leftMotor Left wheel motor.
	 * @param rightMotor Right wheel motor.
	 * @param odo Odometer.
	 * @param nav Navigator.
     * @param colourPollerLeft Thread poller for left colour sensor.
	 * @param colourPollerRight Thread poller for right colour sensor.
     */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odo, Navigator nav, ColourPoller colourPollerLeft, ColourPoller colourPollerRight) {
		this.nav = nav;
		this.odo = odo;
		this.colourPollerLeft = colourPollerLeft;
		this.colourPollerRight = colourPollerRight;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	/**
	 * Corrects theta and brings robot to relative (0,0,0). Relies upon correction in odometerCorrection.
	 */
	public void doLocalization() {

		LCDDisplay.sendToDisplay("Exec: LightLocalization", true);

		// Set odometer values to (-30, -30)
		odo.setX(-20.00);
		odo.setY(-20.00);

		// Drive up to cross y = 0 line
		System.out.println("nav.travelTo(-20.00, 5.00);");
		nav.travelTo(-20.00, 10.00);
		while(nav.isNavigating()){
			System.out.println("X:"+((int)(odo.getX()*100))/100.0+", Y:"+((int)(odo.getY()*100))/100.0+", T:"+((int)(odo.getTheta()*180/Math.PI*100))/100.0);
			pause(100);
		}

		// Move back to vertical middle of block
		System.out.println("nav.travelTo(-20.00, -10.00);");
		nav.travelTo(-20.00, -15.00);
		while(nav.isNavigating()){
			System.out.println("X:"+((int)(odo.getX()*100))/100.0+", Y:"+((int)(odo.getY()*100))/100.0+", T:"+((int)(odo.getTheta()*180/Math.PI*100))/100.0);
			pause(100);
		}

		// Drive right to cross x = 0 line.
		System.out.println("nav.travelTo(5, -10.00);");
		nav.travelTo(15, -15.00);
		while(nav.isNavigating()){
			System.out.println("X:"+((int)(odo.getX()*100))/100.0+", Y:"+((int)(odo.getY()*100))/100.0+", T:"+((int)(odo.getTheta()*180/Math.PI*100))/100.0);
			pause(100);
		}

		// Localization complete.

		// Drive to (0,0)
		System.out.println("nav.travelTo(0,0);");
		nav.travelTo(0,0);
		while(nav.isNavigating()){
			System.out.println("X:"+((int)(odo.getX()*100))/100.0+", Y:"+((int)(odo.getY()*100))/100.0+", T:"+((int)(odo.getTheta()*180/Math.PI*100))/100.0);
			pause(100);
		}

		// Turn to 0 degrees.
		nav.turnToAngle(0.0);

		LCDDisplay.sendToDisplay("Compl: LightLocalization", true);

	}

	/**
	 * Pauses thread to allow for wheel motions to finish.
	 * @param timeToStop amount of time to stop, in milliseconds.
	 */
	private void pause(int timeToStop) {
		try {
			Thread.sleep(timeToStop);
		} catch (InterruptedException e) {
			e.printStackTrace();
			Sound.beep();
			Sound.twoBeeps();
		}
	}

}







