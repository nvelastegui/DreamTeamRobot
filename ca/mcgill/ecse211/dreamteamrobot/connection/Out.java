package ca.mcgill.ecse211.dreamteamrobot.connection;

import java.io.DataOutputStream;
import java.io.IOException;
import java.io.StringWriter;

import ca.mcgill.ecse211.dreamteamrobot.brick1.kinematicmodel.KinematicModel;
import org.json.simple.*;

public class Out {
	private DataOutputStream outData;
	private Object lock;
	
	public Out(DataOutputStream outData){
		this.outData = new DataOutputStream (outData);
		this.lock = new Object();
	}
	
	public void sendRaw(String str) {
		try {
			synchronized(lock){
				this.outData.writeUTF(str);;
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	// Sends : "{"url":"<routeName>","data":"<data>"}
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

