package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.LaunchCubeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.motion.AutonomousPaths;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Side;

public class ScoreSwitchFrontFromStartCenter extends CommandGroup {
    public ScoreSwitchFrontFromStartCenter(Robot robot, Side switchSide) {
        addParallel(new SetElevatorPositionCommand(robot.getElevator(), ElevatorSubsystem.SCORE_SWITCH_POISITON));
        if (switchSide == Side.LEFT)
            addSequential(new FollowPathCommand(robot.getDrivetrain(), AutonomousPaths.CENTER_START_TO_LEFT_SWITCH));
        else
            addSequential(new FollowPathCommand(robot.getDrivetrain(), AutonomousPaths.CENTER_START_TO_RIGHT_SWITCH));
        addSequential(new LaunchCubeCommand(robot.getGatherer(), 0.5));
    }
}
