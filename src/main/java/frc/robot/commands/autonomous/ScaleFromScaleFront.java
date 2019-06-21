package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.CalibrateElevatorEncoderCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.motion.AutonomousPaths;
import frc.robot.motion.Path;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Side;

public class ScaleFromScaleFront extends CommandGroup {

	public ScaleFromScaleFront(Robot robot, Side scaleSide) {
		Path pathToScale = null;
		if (scaleSide == Side.LEFT)
			pathToScale = AutonomousPaths.LEFT_SCALE_FRONT_TO_LEFT_SCALE;
		else
			pathToScale = AutonomousPaths.RIGHT_SCALE_FRONT_TO_RIGHT_SCALE;


		CommandGroup zeroElevatorGroup = new CommandGroup();
		zeroElevatorGroup.addSequential(new SetElevatorPositionCommand(robot.getElevator(),
				ElevatorSubsystem.GROUND_POSITION + 15));
		zeroElevatorGroup.addSequential(new WaitForElevatorPositionCommand(robot.getElevator(), ElevatorSubsystem.GROUND_POSITION + 15));
		zeroElevatorGroup.addSequential(new CalibrateElevatorEncoderCommand(robot.getElevator()));

		addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToScale));
		addParallel(zeroElevatorGroup);
		addSequential(new SetDrivetrainAngleCommand(robot.getDrivetrain(), 180), 4);
	}
}
