/*
* @author Sean Lawlor
* @date November 3, 2011
* @class ECSE 211 - Design Principle and Methods
*
* Modified by F.P. Ferrie
* February 28, 2014
* Changed parameters for W2014 competition
* 
* Modified by Francois OD
* November 11, 2015
* Ported to EV3 and wifi (from NXT and bluetooth)
* Changed parameters for F2015 competition
*/
package ca.mcgill.ecse211.dreamteamrobot.brick1.wifi;

import java.io.*;
import java.net.Socket;
import java.util.HashMap;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/*
 * This class opens a wifi connection, waits for the data
 * and then allows access to the data after closing the wifi socket.
 * 
 * It should be used by calling the constructor which will automatically wait for
 * data without any further user command
 * 
 * Once completed, the HashMap<String,Integer> with the start values is accessible from the field StartData
 */
public class WifiConnection {
	
	public HashMap<String,Integer> StartData;
	
	private TextLCD LCD = LocalEV3.get().getTextLCD();
	
	public WifiConnection(String serverIP, int teamNumber) throws IOException {
		LCD.clear();
		LCD.drawString("WC called", 0, 2);
		
		// Open connection to the server and data streams
		int port = 2000 + teamNumber; //semi-abritrary port number"
		LCD.drawString("Opening wifi connection to server at IP: " + serverIP, 0, 3);
	    Socket socketClient = new Socket(serverIP, port);
	    LCD.drawString("Connected to server", 0, 1);
		DataOutputStream dos = new DataOutputStream(socketClient.getOutputStream());
		DataInputStream dis = new DataInputStream(socketClient.getInputStream());

		// Wait for the server transmission to arrive
		LCD.drawString("Waiting for transmission...", 0, 2);
		while(dis.available() <= 0)
			try {Thread.sleep(10);} catch (InterruptedException e) {}
		LCD.drawString("Receiving transmission", 0, 3);		
		
		// Parse transmission
		this.StartData = ParseTransmission.parseData(dis);
		LCD.drawString("Finished parsing", 0, 4);
		
		// End the wifi connection
		dis.close();
		dos.close();
		socketClient.close();
		LCD.drawString("Connection terminated", 0, 5);
		
	}
	
}
