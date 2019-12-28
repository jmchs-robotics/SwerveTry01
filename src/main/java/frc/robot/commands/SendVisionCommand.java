package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.SocketVisionSender;

public class SendVisionCommand extends Command {
  private SocketVisionSender m_sender;

	public SendVisionCommand(SocketVisionSender sender, String message) {
    if(sender == null) return;

    m_sender = sender;
    switch(message){
      case SocketVisionSender.PlatformBlueSearch:
      case SocketVisionSender.PlatformRedSearch:
      case SocketVisionSender.StartCubeSearch:
      case SocketVisionSender.StartDepth:
      case SocketVisionSender.StartRFT:
        break;
      default:
        message = SocketVisionSender.CarryOnMyWaywardSon;
    }
    m_sender.setSendData(message);
	}

	@Override
	public void execute() { }

	@Override
	protected boolean isFinished() {
		return true;
	}
}
