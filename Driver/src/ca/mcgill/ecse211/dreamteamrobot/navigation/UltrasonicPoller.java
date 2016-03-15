package ca.mcgill.ecse211.dreamteamrobot.navigation;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

//
//  Control of the wall follower is applied periodically by the 
//  UltrasonicPoller thread.  The while loop at the bottom executes
//  in a loop.  Assuming that the us.fetchSample, and cont.processUSData
//  methods operate in about 20mS, and that the thread sleeps for
//  50 mS at the end of each loop, then one cycle through the loop
//  is approximately 70 mS.  This corresponds to a sampling rate
//  of 1/70mS or about 14 Hz.
//

public class UltrasonicPoller extends Thread {

	/** Variables */
	private SampleProvider sampleProvider;
	private float[] usData;
	private int distance;

	/** Constructor */
	public UltrasonicPoller(Port ultrasonicSensorPort) {
		SensorModes usSensor = new EV3UltrasonicSensor(ultrasonicSensorPort);
		this.sampleProvider = usSensor.getMode("Distance");			// colorValue provides samples from this instance;
		this.usData = new float[sampleProvider.sampleSize()];;
	}

	/**
	 * @return Current distance.
	 */
	public int getDistance() {
		return distance;
	}

	/**
	 * Continually fetches distance and stores in distance variable.
	 */
	public void run() {

		while (true) {
			sampleProvider.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast
													// to int
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

}
