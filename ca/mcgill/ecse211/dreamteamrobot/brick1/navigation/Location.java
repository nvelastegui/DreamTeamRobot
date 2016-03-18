package ca.mcgill.ecse211.dreamteamrobot.brick1.navigation;

/**
 * This object holds a coordinate.
 */
public class Location {

	private double x;
	private double y;

	public Location (double x, double y) {
		setX(x);
		setY(y);
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getX() {
		return x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double getY() {
		return y;
	}
}
