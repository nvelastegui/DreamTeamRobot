package ca.mcgill.ecse211.dreamteamrobot.connection;

import java.io.*;

import java.net.Socket;
import java.net.UnknownHostException;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

public class Connection {
	
	private ServerSocket serverSocket;
	private Socket socket;
	public Out out;
	public In in;
	public Queue queue;
	
	private DataOutputStream outData;
	private DataInputStream inData;
	
	public Connection(){
		
	}
	
	public boolean accept(int port, int timeout){
		try {
			serverSocket = new ServerSocket(port);
			serverSocket.setSoTimeout(timeout);
			
			System.out.println("Wait on " + serverSocket.getLocalPort() + "...");
			socket = serverSocket.accept();
		
		    System.out.println("Connected to " + socket.getRemoteSocketAddress());
		    outData = new DataOutputStream(socket.getOutputStream());
		    this.out = new Out(socket.getOutputStream());
		    inData = new DataInputStream(socket.getInputStream());
			this.in = new In(new InputStreamReader(socket.getInputStream()));

			return true;
			
		} catch (IOException e) {
			e.printStackTrace();
			this.out = new Out(null);
			this.in = new In(null);

			return false;
		}
	}
	
	public boolean connect(String serverName, int port){
		try {
			socket = new Socket(serverName, port);
			socket.setTcpNoDelay(true);

		    outData = new DataOutputStream(socket.getOutputStream());
			this.out = new Out(outData);
		    inData = new DataInputStream(socket.getInputStream());
			this.in = new In(new InputStreamReader(socket.getInputStream()));

			return true;
		} catch (IOException e) {
			e.printStackTrace();
			this.out = new Out(null);
			this.in = new In(null);

			return false;
		}
	}
	
	public void listen(int sleepTime){
		this.in.setSleepTime(sleepTime);
		this.in.setQueue(this.queue);
		if(this.in != null) {
			this.in.start();
		}
	}
}
