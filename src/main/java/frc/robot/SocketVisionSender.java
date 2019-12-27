package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;

public class SocketVisionSender extends Thread {
	public static final String StartRFT = "R";
	public static final String StartDepth = "E";
	public static final String StartCubeSearch = "C";
	public static final String PlatformRedSearch = "Pr";
	public static final String PlatformBlueSearch = "Pb";
	public static final String CarryOnMyWaywardSon = " ";

	private String ip_;
	private int port_;
	private boolean is_connected_ = false;
	private boolean keep_running = false;
	private byte[] data_ = CarryOnMyWaywardSon.getBytes();
	private DatagramSocket socket_;

	public SocketVisionSender(String ip, int port) {
		ip_ = ip;
		port_ = port;
	}

	/**
	 * This method tries to "connect" by instantiating the necessary socket for sending.
	 * @return
	 * true if successful
	 */
	public boolean connect() {
		try {
			socket_ = new DatagramSocket(port_);
			socket_.connect(InetAddress.getByName(ip_), port_);
			
			is_connected_ = true;
		} catch (UnknownHostException ex) {
			is_connected_ = false;
			return false;
		} catch (IOException ex) {
			is_connected_ = false;
			System.out.println("IOExcepton " + ex.getMessage());
			return false;
		}
		return true;
	}

	/**
	 * The state of the sender
	 * @return
	 * true if connected
	 */
	public boolean is_connected() {
		return is_connected_;
	}

	/**
	 * This method sends the data to the socket.
	 * @return true if successful.
	 */
	public boolean send() {
		
		byte[] data;

		synchronized(this){
			data = data_;
		}
		
		if(Robot.SHOW_DEBUG_VISION) {
			System.out.println("Sending: \"" + getData() + "\"...");
		}

		if(data == null) return false;

		try {
			DatagramPacket packet = new DatagramPacket(data, data.length, InetAddress.getByName(ip_), port_);			
			socket_.send(packet);
		}catch(Exception e) {
			e.printStackTrace();
			return false;
		}

		if(Robot.SHOW_DEBUG_VISION) {
			System.out.println("Sent: \"" + getData() + ".\"");
		}
		
		return true;
	}

	/**
	 * This is the threaded method that constantly checks and sends to the socket.
	 */
	@Override
	public void run() {
		// this is the threaded method that constantly checks
		// and sends to the socket.
		keep_running = true;
		while (keep_running) {
			if (!is_connected()) {
				connect();
			}
			send();
						
			try {
				Thread.sleep(100);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
			
		}
	}

	/**
	 * This function stops the UDP socket properly, so it can be restarted later.
	 */
	public void stoprunning() {
		keep_running = false;

		socket_.disconnect();
		socket_.close();
	}

	/**
	 * Sets the data content of the datagram packet.
	 * @param stringToSend The string to encode into bytes for the datagram packet.
	 */
	public synchronized void setSendData(String stringToSend) {
		if((CarryOnMyWaywardSon + StartCubeSearch + StartDepth + StartRFT).contains(stringToSend)) {
			data_ = stringToSend.getBytes();
		}else {
			data_ = CarryOnMyWaywardSon.getBytes();
		}
	}

	/**
	 * @return a String object representing the data being sent through the socket.
	 */
	public String getData() {
		return new String(data_);
	}
}