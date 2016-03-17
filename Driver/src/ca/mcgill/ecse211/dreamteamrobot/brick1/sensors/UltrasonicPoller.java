package ca.mcgill.ecse211.dreamteamrobot.brick1.sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This thread polls an ultrasonic sensor and logs the value.
 * Parent threads can call getDistance() to retrieve the value read by the sensor.
 */
public class UltrasonicPoller extends Thread {

	// Variables
	private SampleProvider sampleProvider;
	private float[] usData;
	private int distance;

	/**
	 * Constructor.
	 * @param ultrasonicSensorPort Port connecting ultrasonic sensor.
     */
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
