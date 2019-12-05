/*package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.IntakeCubeCommand;
import frc.robot.motion.AutonomousPaths;
import frc.robot.motion.Path;
import frc.robot.motion.Trajectory;
import frc.robot.util.Side;

public class GrabCubeFromScale extends CommandGroup {
    private static final double INTAKE_TIME = 2.5;

    public GrabCubeFromScale(Robot robot, Side scaleSide) {
        Path pathToCube;
        Path pathToScale;
        if (scaleSide == Side.LEFT) {
            pathToCube = AutonomousPaths.LEFT_SCALE_TO_LEFT_CUBE;
            pathToScale = AutonomousPaths.LEFT_CUBE_TO_LEFT_SCALE;
        } else {
            pathToCube = AutonomousPaths.RIGHT_SCALE_TO_RIGHT_CUBE;
            pathToScale = AutonomousPaths.RIGHT_CUBE_TO_RIGHT_SCALE;
        }

        Trajectory trajectoryToCube = new Trajectory(pathToCube, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());
        Trajectory trajectoryToScale = new Trajectory(pathToScale, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());

        CommandGroup intakeGroup = new CommandGroup();
        intakeGroup.addSequential(new WaitCommand(Math.max(0, trajectoryToCube.getDuration() - INTAKE_TIME)));
        intakeGroup.addSequential(new IntakeCubeCommand(robot.getGatherer(), INTAKE_TIME));

        addSequential(new VisionLineUpWithCubeCommand(robot));
        addParallel(intakeGroup);
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToCube));
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToScale));
    }
}
*/