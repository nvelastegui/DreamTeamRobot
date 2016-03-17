package communication;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import lejos.remote.nxt.NXTConnection;
import lejos.remote.nxt.SocketConnector;

public class WebSocketServer {
	
	private ServerSocket serverSocket;
	private Socket server;
    public DataOutputStream out;
    public DataInputStream in;
	
	    
	public WebSocketServer(int port, int timeout) throws IOException {
		serverSocket = new ServerSocket(port);
	    serverSocket.setSoTimeout(timeout);
	}
	
	public void connect() throws IOException {
	    System.out.println("Wait on " + serverSocket.getLocalPort() + "...");
	    server = serverSocket.accept();
	
	    System.out.println("Connected to " + server.getRemoteSocketAddress());
	    out = new DataOutputStream(server.getOutputStream());
	    in = new DataInputStream(server.getInputStream());
	}
	
	public int getPort() {
	    int port = 0;
	    if (serverSocket != null) {
	        port = serverSocket.getLocalPort();
	    }
	    return port;
	}
	
	public boolean isConnected() {
	    if (server == null) {
	        return false;
	    }
	    return server.isConnected();
	}
	
	public String socketStatus() {
	    return server == null ? "null" : "not null";
	}
	
	public void connectionLost() {
	    server = null;
	}

	
//	public static void server(){
//		SocketConnector sock2 = new SocketConnector();
//		
//		NXTConnection sock2Con = sock2.waitForConnection(30000, 0);
//		
//		if(sock2Con != null){
//			System.out.println("sock2 connected!");
//		} else {
//			System.out.println("sock2 failed!");
//		}
//	}
}
