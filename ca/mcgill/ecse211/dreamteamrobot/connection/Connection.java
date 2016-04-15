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

	/**
	 * Empty connection class object
	 */
	public Connection(){
		
	}

	/**
	 * Given a port to open / listen on, this method waits for incoming connections and opens a TCP/IP socket.
	 * Returns whether the connection has been successfully established in the given time limit
	 * @param port
	 * @param timeout
     * @return
     */
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

	/**
	 * Similar to accept but will execute a connection request and open a socket on a remote address and port.
	 * Returns whether the connection has been successfully established in the given time limit
	 * @param serverName
	 * @param port
     * @return
     */
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

	/**
	 * Triggers the In Thread and instantiates the queue object for incoming messages
	 * @param sleepTime
     */
	public void listen(int sleepTime){
		this.in.setSleepTime(sleepTime);
		this.in.setQueue(this.queue);
		if(this.in != null) {
			this.in.start();
		}
	}
}
