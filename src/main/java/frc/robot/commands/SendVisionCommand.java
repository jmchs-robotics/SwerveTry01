package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.util.SocketVisionSendKeeper;

public class SendVisionCommand extends Command {
  private SocketVisionSendKeeper m_sender;
  String m_message;

	public SendVisionCommand(SocketVisionSendKeeper sender, String message) {
    m_sender = sender;
    m_message = message;
	}

	@Override
	public void execute() { }

	@Override
	protected boolean isFinished() {
    SocketVisionSendKeeper tmp = m_sender;
    if(tmp != null) tmp.get().setSendData(m_message);
		return true;
	}
}
