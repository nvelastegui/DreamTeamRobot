package communication;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;

public class S_COM_001_client {
	public static void main(String[] args){
		
		// initialize client
		String serverName = "10.0.2.1";
		int port = 6666;
		
		Socket client;
		try {
			client = new Socket(serverName, port);
			
			System.out.println("client connected to " + client.getRemoteSocketAddress());
	        try {
				DataInputStream in = new DataInputStream(client.getInputStream());
				System.out.println("client got inputStream");
				
				// try reading data
				String clientIn = in.readUTF();
				System.out.println("client received : "+clientIn);
				
				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	        
	        try {
				DataOutputStream out = new DataOutputStream(client.getOutputStream());
				System.out.println("client got outputStream");
				
				// try sending data
				String clientOut = "client send data";
				out.writeUTF(clientOut);
				System.out.println("client sent : "+clientOut);
				
				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	        
	        
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
        
		
	}
}
