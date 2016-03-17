package ca.mcgill.ecse211.dreamteamrobot.brick1.sensors;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This thread polls a color sensor and logs values.
 * Parent threads can call getSensorValue() to retrieve the value read by the sensor.
 */
public class ColourPoller {

    // Variables
    private SampleProvider sampleProvider;
    private float[] colourData;
    private double sensorValue;

    /**
     * Constructor.
     * @param colorSensorPort Port on brick connecting to color sensor.
     */
    public ColourPoller(Port colorSensorPort) {
        SensorModes colorSensor = new EV3ColorSensor(colorSensorPort);
        this.sampleProvider = colorSensor.getMode("Red");			// colorValue provides samples from this instance;
        this.colourData = new float[sampleProvider.sampleSize()];;
    }

    /**
     * @return Current sensor value.
     */
    public double getSensorValue() {
        return sensorValue;
    }

    /**
     * Continually fetches sensor value and stores in variable.
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
