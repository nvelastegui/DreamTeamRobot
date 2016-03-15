package ca.mcgill.ecse211.dreamteamrobot;

// Nicolas Velastegui 260521419
// Siddiqui Hakim 260564770
// Group 26

public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
