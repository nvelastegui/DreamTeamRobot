package ca.mcgill.ecse211.dreamteamrobot.brick1.communication;

import com.sun.tools.internal.xjc.Driver;

/**
 * Packet for transferring data between Driver and LCDDisplay.
 */
public class DriverStatusPacket {

    private String currentState;
    private double x;
    private double y;
    private double theta;

    public DriverStatusPacket (String currentState, double x, double y, double theta) {
        this.currentState = currentState;
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public String getCurrentState() {
        return currentState;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }
}
