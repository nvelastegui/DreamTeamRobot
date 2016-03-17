package communication;

import java.io.IOException;

public class S_COM_001_server {
	public static void main(String[] args){
		
		// initialize server
		int port = 6666;
        int timeout = 30000;
        WebSocketServer ss = null;

        try {
            ss = new WebSocketServer(port, timeout);
        } catch (IOException ex) {
            
        }

        try {
            ss.connect();
        } catch (java.net.SocketTimeoutException ex) {
            
        } catch (IOException ex) {
            
        }

        if (ss.isConnected()) {
            String response = null;
            // establish connection
            System.out.println("Server recieved Connection");
            
            // server sends data
            try {
                ss.out.writeUTF("sent from server");
                System.out.println("Server sent data");
            } catch (IOException ex) {
                
            }
            
            // server receives data
            try {
                String dataIn = ss.in.readUTF();
                System.out.println("Server received data : "+dataIn);
            } catch (IOException ex) {
                
            }
            System.out.println("Server has finished.");
            
        }
	}
}
