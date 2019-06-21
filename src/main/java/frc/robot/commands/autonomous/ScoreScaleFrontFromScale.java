package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.commands.IntakeCubeNoRequireCommand;
import frc.robot.commands.LaunchCubeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.motion.AutonomousPaths;
import frc.robot.motion.Path;
import frc.robot.motion.Trajectory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Side;

public class ScoreScaleFrontFromScale extends CommandGroup {
    private static final double LAUNCH_TIME = 1;

    public ScoreScaleFrontFromScale(Robot robot, Side scaleSide) {
        Path pathToScaleFront;
        if (scaleSide == Side.LEFT)
            pathToScaleFront = AutonomousPaths.LEFT_SCALE_TO_LEFT_SCALE_FRONT;
        else
            pathToScaleFront = AutonomousPaths.RIGHT_SCALE_TO_RIGHT_SCALE_FRONT;

        Trajectory trajectoryToScale = new Trajectory(pathToScaleFront, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());

        addParallel(new SetElevatorPositionCommand(robot.getElevator(), ElevatorSubsystem.TOP_POSITION));
        addSequential(new SetDrivetrainAngleCommand(robot.getDrivetrain(),
                (scaleSide == Side.LEFT ? 330 : 30)));
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToScaleFront));
        addSequential(new LaunchCubeCommand(robot.getGatherer(), LAUNCH_TIME, 0.9));
    }
}
