package ca.mcgill.ecse211.dreamteamrobot.connection;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.util.ArrayDeque;
import java.util.Hashtable;
import java.util.NoSuchElementException;

import java.util.regex.Matcher;
import java.util.regex.Pattern;


public class Queue {
	public final String routeProperty;
	private Hashtable<String, ArrayDeque> queueTable;
	private Hashtable<String, ArrayDeque> unsortedTable;
	
	/*
	 * Sorts all messages (json) in to different queues.
	 * Messages are 'routed' to their designated queue using the routeProperty
	 * ie : {"url":"logs","data":"this is a log"} will get added to the "logs" queue if "url" is the selected routeProperty
	 */

	/**
	 * Sorts all messages (json) in to different queues.
	 * Messages are 'routed' to their designated queue using the routeProperty
	 * ie : {"url":"logs","data":"this is a log"} will get added to the "logs" queue if "url" is the selected routeProperty
	 * @param routeProperty
     */
	public Queue(String routeProperty){
		this.routeProperty = routeProperty;
		this.queueTable = new Hashtable<String, ArrayDeque>();
		this.unsortedTable = new Hashtable<String, ArrayDeque>();
	}
	
	/*
	 * Splits up line when multiple messages were sent with one connection.
	 * Can be used for splitting up batch requests
	 */

	/**
	 * Splits up line when multiple messages were sent with one connection.
	 * Can be used for splitting up batch requests
	 * @param msg
     */
	public void processLine(String msg){
		JSONParser parser = new JSONParser();
		JSONObject obj;
		try {
			obj = (JSONObject)parser.parse(msg);
			queueMsgObj(obj, msg);
			//System.out.println("processLine : "+msg);
		} catch (ParseException e) {

//			String pattern = "\\}\\{";
//			Pattern r = Pattern.compile(pattern);			// Create a Pattern object
//			// Now create matcher object.
//			Matcher m = r.matcher(msg);
//			if(m.find()){
				String[] splitMessages = msg.replaceAll("\\}\\{", "}\u0000{").split("\\u0000");
				for(int i=0;i<splitMessages.length;i++){
					try{
						splitMessages[i].replace("^.\\{","{");		// replace the occasional leading character (byte order mark possibly?)
						obj = (JSONObject)parser.parse(splitMessages[i]);
						queueMsgObj(obj, splitMessages[i]);
						//System.out.println("processLine : "+splitMessages[i]);
					} catch (ParseException e2){
						System.out.println("Failed Parse : "+splitMessages[i]);
						e2.printStackTrace();
					}
				}
//			} else {
//				try{
//					obj = (JSONObject)parser.parse(msg);
//					//System.out.println("processLine : "+msg);
//					queueMsgObj(obj, msg);
//				} catch (ParseException e2){
//					System.out.println("Failed Parse : "+msg);
//					e2.printStackTrace();
//				}
//			}

		}
	}
	
	/*
	 * Takes json string and parses out route.
	 * Adds it to associated queue
	 * Can uncomment so that unregistered messages are queued in an unsortedQueue
	 */

	/**
	 * Takes json string and parses out route.
	 * Adds it to associated queue
	 * Can uncomment so that unregistered messages are queued in an unsortedQueue
	 * @param obj
	 * @param msg
     */
	private void queueMsgObj(JSONObject obj, String msg){
		if(obj == null) return;
		if(obj.get(routeProperty) != null){
			ArrayDeque<String> curQueue = queueTable.get(obj.get(routeProperty));
			if(curQueue != null){
				curQueue.add(msg);
			} else {
//				ArrayDeque<String> unsortedQueue = unsortedTable.get(obj.get(routeProperty));
//				if(unsortedQueue != null){
//					unsortedQueue.add(msg);
//				} else {
//					ArrayDeque<String> newQueue = new ArrayDeque<String>();
//					this.queueTable.put((String)obj.get(routeProperty), newQueue);
//				}
			}
		} else {
			System.out.println("No route property : "+routeProperty+" : "+msg);
		}
	}
	
	/*
	 * Registers a route and adds its associated queue to the hashtable
	 */

	/**
	 * Registers a route and adds its associated queue to the hashtable
	 * @param routeValue
     */
	public void registerQueue(String routeValue){
		ArrayDeque<String> newQueue = new ArrayDeque<String>();
		this.queueTable.put(routeValue, newQueue);
	}
	
	// returns queue for a particular route

	/**
	 * Returns queue for a particular route
	 * @param routeValue
	 * @return
     */
	public ArrayDeque get(String routeValue){
		ArrayDeque cur = queueTable.get(routeValue);
		if(cur != null){
			return cur;
		} else {
			ArrayDeque unsorted = unsortedTable.get(routeValue);
			return unsorted;
		}
	}
	
	// pops string off the given queue

	/**
	 * Pops string off the given queue
	 * @param routeValue
	 * @return
     */
	public String popString(String routeValue){
		ArrayDeque cur = queueTable.get(routeValue);
		if(cur != null){
			try{
				return (String) cur.pop();
			} catch (NoSuchElementException e){
				return null;
			}
		} else {
			return null;
		}
	}
	
	/*
	 *  pops string off the given queue and tries to parse the json.
	 *  if fails to parse.. returns null otherwise returns the body or empty object if body is null
	 */

	/**
	 * Pops string off the given queue and tries to parse the json.
	 * If fails to parse.. returns null otherwise returns the body or empty object if body is null
	 * @param routeValue
	 * @return
     */
	public JSONObject popJSON(String routeValue){
		JSONParser parser = new JSONParser();
		JSONObject obj;
		
		ArrayDeque<String> curQueue = queueTable.get(routeValue);
		
		if(curQueue != null){
			try{
				obj = (JSONObject)parser.parse(curQueue.pop());
				if((JSONObject)obj.get("body") == null){
					return new JSONObject();
				} else {
					return (JSONObject)obj.get("body");
				}

			} catch (ParseException e){
				e.printStackTrace();
				return null;
			} catch (NoSuchElementException e){
				return null;
			}
		} else {
			return null;
		}
	}
}
