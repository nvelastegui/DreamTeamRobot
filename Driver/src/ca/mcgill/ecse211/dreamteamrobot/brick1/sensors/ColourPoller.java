package ca.mcgill.ecse211.dreamteamrobot.brick1.sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This thread runs in the background, polling a color sensor and logging values.
 */
public class ColourPoller {

    /** Variables */
    private SampleProvider sampleProvider;
    private float[] colourData;
    private double sensorValue;

    /** Constructor */
    public ColourPoller(Port colorSensorPort) {
        SensorModes colorSensor = new EV3ColorSensor(colorSensorPort);
        this.sampleProvider = colorSensor.getMode("Red");			// colorValue provides samples from this instance;
        this.colourData = new float[sampleProvider.sampleSize()];;
    }

    /**
     * @return Current distance.
     */
    public double getSensorValue() {
        return sensorValue;
    }

    /**
     * Continually fetches distance and stores in distance variable.
     */
    public void run() {

        while (true) {
            sampleProvider.fetchSample(colourData, 0);
            sensorValue = colourData[0] * 100;
            try {
                Thread.sleep(50);
            } catch (Exception e) {
            } // Poor man's timed sampling
        }
    }

}
