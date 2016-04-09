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

import java.io.DataInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.HashMap;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/*
 * Static parsers for parsing data off the communication channel
 * 
 * The order of data is defined in the Server's Transmission class
 */

public class ParseTransmission {
	
	public static TextLCD LCD = LocalEV3.get().getTextLCD();
	
	// This should only be called after verifying that there is data in the input stream
	public static HashMap<String,Integer> parseData(DataInputStream dis){
		HashMap<String,Integer> StartData;
		LCD.drawString("Receipt initiated", 0, 1);
		try{
			ObjectInputStream ois = new ObjectInputStream(dis);
			StartData = (HashMap<String,Integer>) ois.readObject();
			LCD.drawString("Map received", 0, 1);
		} catch (Exception e) {
			StartData = null;
		}
		return StartData;
	}
	
	public static void ignore(DataInputStream dis) throws IOException {
		dis.readChar();
	}
	
}
