/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.SendVisionCommand;
import frc.robot.util.SocketVisionSendWrapper;
import frc.robot.util.SocketVisionSender;
import frc.robot.util.SocketVisionWrapper;

public class VisionCommandGroup extends CommandGroup {
  /**
   * Sends a message string to the UP board, then executes a {@link VisionLineUpWithCubeCommand}
   * with the provided {@link SocketVisionWrapper}. Finally, resets the send string to " ".
   */
  public VisionCommandGroup(Robot robot, SocketVisionSendWrapper sender, String message, SocketVisionWrapper receiver) {
    addSequential(new SendVisionCommand(sender, message));
    addSequential(new VisionLineUpWithCubeCommand(robot, receiver));
    addSequential(new SendVisionCommand(sender, SocketVisionSender.CarryOnMyWaywardSon));
  }
}
