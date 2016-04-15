package ca.mcgill.ecse211.dreamteamrobot.connection;

//import java.io.DataOutputStream;
import java.io.*;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import org.json.simple.*;

public class Out {
	private OutputStream outStream;
	private OutputStreamWriter outWritter;
	//private OutputStream outData;
	private Object lock;

	/**
	 * Parent class for Out Object and its methods
	 * Given an output stream, will instantiate a StreamWriter around it and set the encoding
	 * @param outData
     */
	public Out(OutputStream outData){
		this.outStream = outData;

		if(outData == null){
			this.outWritter = null;
		} else {
			try {
				this.outWritter = new OutputStreamWriter(outData, "UTF-8");
			} catch (UnsupportedEncodingException e) {
				e.printStackTrace();
			}
		}

		this.lock = new Object();
	}

	/**
	 * Sends a raw String down the socket
	 * @param str
     */
	public void sendRaw(String str) {
		if(this.outStream == null || this.outWritter == null) return;
		try {
			synchronized(lock){
				System.out.println("outgoing : "+str);
				this.outWritter.write(str);
				this.outWritter.write("\n");
				outWritter.flush();
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	// Sends : "{"url":"<routeName>","data":"<data>"}

	/**
	 * Send a String to a specific route on the other end. Structures the message in the standardized route/body format
	 * Message format is as follows : "{"url":"<routeName>","data":"<data>"}
	 * @param routeName
	 * @param body
     */
	public void sendStr(String routeName, String body){
		JSONObject json = new JSONObject();
		json.put(KinematicModel.ROUTE_PROPERTY, routeName);
		json.put("body", body);
		StringWriter out = new StringWriter();
		try {
			json.writeJSONString(out);
			this.sendRaw(out.toString());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	// Sends : "{"url":"<routeName>","data":"JSON.stringify(<data>)"}
	/**
	 * Send a Structured JSON Object to a specific route on the other end. Structures the message in the standardized route/body format
	 * Message format is as follows : "{"url":"<routeName>","data":"JSON.stringify(<data>)"}"
	 * @param routeName
	 * @param body
	 */
	public void sendJSONObj(String routeName, JSONObject body){
		JSONObject json = new JSONObject();
		json.put(KinematicModel.ROUTE_PROPERTY, routeName);
		json.put("body", body);
		StringWriter out = new StringWriter();
		try {
			json.writeJSONString(out);
			this.sendRaw(out.toString());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}

