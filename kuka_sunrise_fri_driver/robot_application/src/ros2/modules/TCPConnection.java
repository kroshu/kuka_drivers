package ros2.modules;

import java.net.*;
import java.util.Arrays;
import java.io.*;

import javax.xml.bind.DatatypeConverter;

public class TCPConnection{
	private int _tcpServerPort;
	private ServerSocket _tcpServer;
	private Socket _tcpClient;
	private Thread _tcpConnectionThread;
	private boolean _closeRequested;
	private byte[] _incomingData;
	private ROS2Connection _ROS2Connection;

	public TCPConnection(int tcpServerPort){
		_tcpServerPort = tcpServerPort;
		_tcpServer = null;
		_tcpClient = null;
		_tcpConnectionThread = new Thread(new ConnectionHandler());
		_closeRequested = false;
		_incomingData = null;
	}

	public void registerROS2ConnectionModule(ROS2Connection ros2ConnectionModule){
		_ROS2Connection = ros2ConnectionModule;
	}

	public void waitUntilDisconnected(){
		try {
			_tcpConnectionThread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private class ConnectionHandler implements Runnable{
		public void run(){
			try{
				while(true){
					waitForConnection();
					if(_closeRequested) {
						System.out.println("Stopped waiting for connection.");
						break;
					}
					handleIncomingData();
					if(_closeRequested) {
						System.out.println("TCP Read interrupted.");
						break;
					}
				}
			}catch (IOException e){
				e.printStackTrace();
			}catch(Exception e){
				e.printStackTrace();
			}

			if(_tcpServer.isClosed() == false){
				try{
					_tcpServer.close();
				}catch(IOException e){
					e.printStackTrace();
				}
			}
			_closeRequested = false;
		}
	}

	public void openConnection() {
		try {
			_tcpServer = new ServerSocket(_tcpServerPort);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		_tcpConnectionThread.start();
	}

	public void sendString(String s){
		if(_tcpClient != null && _tcpClient.isConnected() && !_tcpClient.isClosed()){
			try{
				DataOutputStream outToClient = new DataOutputStream(_tcpClient.getOutputStream());
				outToClient.writeBytes(s);
				outToClient.close();
			}
			catch(IOException e){
				e.printStackTrace();
			}
		} else{
			System.out.println("TCP client not connected.");
		}
	}

	public void sendBytes(byte[] message){
		if(_tcpClient != null && _tcpClient.isConnected() && !_tcpClient.isClosed()){
			try{
				DataOutputStream outToClient = new DataOutputStream(_tcpClient.getOutputStream());
				outToClient.write(message);
				//outToClient.close();
			}catch(IOException e){
				e.printStackTrace();
			}
		} else{
			System.out.println("TCP client not connected.");
		}
	}

	public byte[] getReceivedData(){
		byte[] dataCopy = _incomingData;
		_incomingData = null;
		return dataCopy;
	}

	public void closeConnection(){
		_closeRequested = true;
		try{
			if(_tcpServer.isClosed() == false){
				_tcpServer.close();
			}
			if(_tcpClient.isClosed() == false){
				_tcpClient.close();
			}
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
	}

	private void waitForConnection() throws Exception {
		System.out.println("Waiting for connection...");
		try {
			_tcpClient = _tcpServer.accept();
		} catch (SocketException e) {
			if(_closeRequested){
				return;
			} else {
				// closing of connection wasn't requested, error
				throw e;
			}

		}
		System.out.println("Connection established.");
	}

	private void handleIncomingData() throws IOException{
		BufferedReader inFromClient = new BufferedReader(new InputStreamReader(_tcpClient.getInputStream()));
		while(_tcpClient.isClosed() == false){
			int dataLength = -2;
			char[] charArray = new char[1024];
			try{
				dataLength = inFromClient.read(charArray);
			} catch (SocketException e) {
				if(_closeRequested){
					break;
				} else {
					// closing of connection wasn't requested, error
					throw e;
				}
			}
			if(dataLength < 0){
				_ROS2Connection.handleConnectionLost();
				break;
			}

			byte[] byteArray = Arrays.copyOf(new String(charArray).getBytes(), dataLength);
			String byteHexString = DatatypeConverter.printHexBinary(byteArray);

			if(_incomingData != null){
				System.out.println("ERROR: Previous data not yet processed! Skipping data: " + byteHexString);
			}
			else{
				_incomingData = byteArray;
				//System.out.println("New data received: " + byteHexString + ", " + _incomingData);
				_ROS2Connection.handleMessageFromROS(_incomingData);
				_incomingData = null;
			}
		}
	}



}
