package frc.robot;

import frc.robot.commands.*;
import frc.robot.input.DPadButton;
import frc.robot.input.IGamepad;
import frc.robot.input.XboxGamepad;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private static final double SOFT_PLACEMENT_VELOCITY = 2.0;
    private static final double SOFT_PLACEMENT_ACCELERATION = 0.5;

    private IGamepad primaryController = new XboxGamepad(0);
    private IGamepad secondaryController = new XboxGamepad(1);

    private Robot mRobot;

    public OI(Robot robot) {
        mRobot = robot;
    }

    public void registerControls() {
        primaryController.getLeftBumperButton().whenPressed(new SetFieldOrientedCommand(mRobot.getDrivetrain(), false));
        primaryController.getLeftBumperButton().whenReleased(new SetFieldOrientedCommand(mRobot.getDrivetrain(), true));
        primaryController.getStartButton().whenPressed(new ZeroDrivetrainGyroCommand(mRobot.getDrivetrain()));
        //If the swerve is in Brake or Coast Mode... Braking when pressed
        primaryController.getRightBumperButton().whenPressed(new SetMotorBrakeCommand(mRobot,true));
        primaryController.getRightBumperButton().whenReleased(new SetMotorBrakeCommand(mRobot,false));

        primaryController.getAButton().whenPressed(new SendVisionCommand(Robot.sender_, SocketVisionSender.PlatformBlueSearch));
}

    public IGamepad getPrimaryController() {
        return primaryController;
    }

    public IGamepad getSecondaryController() {
        return secondaryController;
    }
}
