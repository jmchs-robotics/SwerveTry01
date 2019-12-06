package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule;

public class SetAngleCommand extends Command {

   // private final Timer timer = new Timer();
    private final SwerveDriveSubsystem drivetrain;
    private SwerveDriveModule[] mSwerveModules;
    //private final double time;
    //private final double forward;
    //private final double strafe;
    private final double angleWant;

    public SetAngleCommand(SwerveDriveSubsystem drivetrain, double a) {
        this.drivetrain = drivetrain;
        this.angleWant = a;

        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        
        for (SwerveDriveModule module : drivetrain.getSwerveModules()) {
            module.setTargetAngle(angleWant);
        }
        SmartDashboard.putNumber("YahoooooooooYEAH!!!!!!!@@#$@$@$$", 100);



      // drivetrain.holonomicDrive(forward, strafe, 0);
    }

    @Override
    protected boolean isFinished() {
        //return timer.hasPeriodPassed(time);
        return true;
    }

    @Override
    protected void end() {
        //timer.stop();
        //timer.reset();
        //drivetrain.holonomicDrive(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
