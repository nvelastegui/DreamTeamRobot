package ca.mcgill.ecse211.dreamteamrobot.brick1.navigation;

/**
 * Thread simultaneous to Odometer. This thread reads values from the environment (ie. color sensor values)
 * and compares them with current values in the Odometer so that it can perform any necessary corrections
 * for improved navigational precision.
 */
public class OdometerCorrection extends Thread {

    private Odometer odometer;

    public OdometerCorrection (Odometer odometer) {

    }

    @Override
    public void run() {
        super.run();
    }
}
