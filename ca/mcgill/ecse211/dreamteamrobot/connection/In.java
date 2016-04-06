package ca.mcgill.ecse211.dreamteamrobot.connection;

import lejos.hardware.Sound;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayDeque;

public class In extends Thread {
	private DataInputStream inData;
	private BufferedReader inBuffer;
	
	private Object lock;
	private Boolean running;
	private Queue queue;
	
	private int sleepTime;
	private ArrayDeque<String> Incoming;

	
	public In(InputStreamReader stream){
		this.inBuffer = new BufferedReader(stream);
		this.Incoming = new ArrayDeque<String>();
		this.lock = new Object();

		this.running = true;
	}

	/*
	 * Reads one line from the TCP socket connection
	 */
	public String readLine(){
		StringBuilder sb = new StringBuilder();
		try {
			sb.append(inBuffer.readLine());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return sb.toString();
	}
	
//	public String readLine(){
//		try {
//
//			String inDataStr = this.inData.readUTF();
//			System.out.println("inDataStr : "+inDataStr);
//			return inDataStr;
//		} catch (IOException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			return null;
//		}
//	}
	
	/*
	 * Polls for new lines in the socket connection.
	 * Passes each new line to the queue.processLine function to be sorted
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		this.running = true;
		System.out.println("Starting to listen");
		String curLine;
		while(isRunning()){
			curLine = readLine();
			if(curLine != null && curLine != ""){
				synchronized(lock){
					System.out.println("incoming : "+curLine);
					this.queue.processLine(curLine);
				}
			}
			try {
				Thread.sleep(sleepTime);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	public void stopListening(){
		synchronized(lock){
			this.running = false;
			this.sleepTime = 0;
		}
	}
	
	public Boolean isRunning(){
		synchronized(lock) {
			return this.running;
		}
	}
	
	public void setSleepTime(int sleepTime){
		synchronized(lock){
			this.sleepTime = sleepTime;
		}
	}
	
	public int getSleepTime(){
		synchronized(lock){
			return this.sleepTime;
		}
	}

	public void setQueue(Queue queue) {
		synchronized(lock){
			this.queue = queue;
		}
	}

}
