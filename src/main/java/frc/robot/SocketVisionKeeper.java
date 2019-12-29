package frc.robot;

import frc.robot.SocketVision;

/**
 * Wrapper for SocketVisionSender (one-to-one) so the threaded objects can be handled in an FRC-safe
 * manner and be passed around commands in a more generic way.
 */
public class SocketVisionKeeper {

  private SocketVision m_reader = null;

  private int m_port;
  private String m_ip;

  public SocketVisionKeeper( String ip, int port){
    m_ip = ip;
    m_port = port;
  }

  public void init(){
    if( m_reader == null) {
			m_reader = new SocketVision(m_ip, m_port);

			m_reader.start();
			if(!m_reader.is_connected()) {
				m_reader.connect();
			}
    }
  }

  public void shutDown(){
    if(m_reader != null) {
			try {
				m_reader.stoprunning();
				m_reader.join();
				m_reader = null;
			}catch (Exception e) {
				e.printStackTrace();
			}
		}
  }

  public SocketVision get(){
    return m_reader;
  }
}