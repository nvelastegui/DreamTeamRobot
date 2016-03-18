package ca.mcgill.ecse211.dreamteamrobot.brick2;

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
