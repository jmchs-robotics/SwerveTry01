package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.CalibrateElevatorEncoderCommand;
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.commands.LaunchCubeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.motion.AutonomousPaths;
import frc.robot.motion.Path;
import frc.robot.motion.Trajectory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Side;

public class ScoreSwitchFrontFromSwitchFront extends CommandGroup {
    private static final double INTAKE_TIME = 2.5;
    private static final double LAUNCH_TIME = 0.5;
    private static final double ELEVATOR_WAIT = 1;

    public ScoreSwitchFrontFromSwitchFront(Robot robot, Side switchSide) {
        addParallel(new CalibrateElevatorEncoderCommand(robot.getElevator()));

        Path pathToCube;
        Path pathToSwitch;
        if (switchSide == Side.LEFT) {
            pathToCube = AutonomousPaths.LEFT_SWITCH_FRONT_SECOND_CUBE_STEP_1;
            pathToSwitch = AutonomousPaths.LEFT_SWITCH_FRONT_SECOND_CUBE_STEP_2;
        } else {
            pathToCube = AutonomousPaths.RIGHT_SWITCH_FRONT_SECOND_CUBE_STEP_1;
            pathToSwitch = AutonomousPaths.RIGHT_SWITCH_FRONT_SECOND_CUBE_STEP_2;
        }

        Trajectory trajectoryToCube = new Trajectory(pathToCube, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());
        Trajectory trajectoryToSwitch = new Trajectory(pathToSwitch, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());

        CommandGroup intakeGroup = new CommandGroup();
        intakeGroup.addSequential(new WaitCommand(Math.max(0, trajectoryToCube.getDuration() - INTAKE_TIME / 2)));
        intakeGroup.addSequential(new IntakeCubeCommand(robot.getGatherer(), INTAKE_TIME));

        CommandGroup scoreGroup = new CommandGroup();
        scoreGroup.addSequential(new WaitCommand(Math.max(0, trajectoryToSwitch.getDuration() - LAUNCH_TIME)));
        scoreGroup.addSequential(new LaunchCubeCommand(robot.getGatherer(),  LAUNCH_TIME));

        CommandGroup elevatorGroup = new CommandGroup();
        elevatorGroup.addSequential(new WaitCommand(ELEVATOR_WAIT));
        elevatorGroup.addSequential(new SetElevatorPositionCommand(robot.getElevator(), ElevatorSubsystem.SCORE_SWITCH_POISITON));

        addParallel(intakeGroup);
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToCube,
                robot.getDrivetrain().getMaxAcceleration() * 0.6,
                robot.getDrivetrain().getMaxVelocity() * 0.6));
        addParallel(elevatorGroup);
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToSwitch,
                robot.getDrivetrain().getMaxAcceleration() * 0.6,
                robot.getDrivetrain().getMaxVelocity() * 0.6));
        addParallel(scoreGroup);
        addSequential(new SetDrivetrainAngleIfNotAngledCommand(robot.getDrivetrain(), 0));
        addSequential(new WaitForElevatorPositionCommand(robot.getElevator(), ElevatorSubsystem.SCORE_SWITCH_POISITON));
        addSequential(new LaunchCubeCommand(robot.getGatherer(), 0.5));
    }
}