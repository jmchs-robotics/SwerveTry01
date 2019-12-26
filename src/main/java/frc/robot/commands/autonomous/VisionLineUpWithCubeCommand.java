package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.Side;

public class VisionLineUpWithCubeCommand extends CommandGroup {
    public static final double FINISH_TIMER = 0.5;

    private final Robot robot;
    private final PIDController angleErrorController;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tv = table.getEntry("tv");
    private double pidStrafeValue;
    private double rotationFactor;
    private final Timer finishTimer = new Timer();
    private boolean isFinishTimerRunning = false;

    private PIDController strafeController = new PIDController(0.07, 0.000000001, 0.32, new PIDSource() {

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            // 12/23 jh_vision- read input from SocketVision instead of from NetworkTables
            return Robot.rft_.get_degrees_x(); // tx.getDouble(0);
        }

    }, output -> {
        pidStrafeValue = -output;
        SmartDashboard.putNumber("PID Strafe Value", pidStrafeValue);
    });

    public VisionLineUpWithCubeCommand(Robot robot) {
        this.robot = robot;

        strafeController.setInputRange( -320, 320); // 12/23 jh_vision set for up board range (-27, 27);
        strafeController.setOutputRange(-1, 1);
        strafeController.setAbsoluteTolerance(50); // 12/23 jh_vision need to select tolerance based on the game!

        angleErrorController = new PIDController(0.01, 0, 0, new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return robot.getDrivetrain().getGyroAngle();
            }
        }, output -> {
            rotationFactor = -output;
        });

        angleErrorController.setInputRange(0, 360);
        angleErrorController.setOutputRange(-0.5, 0.5);
        angleErrorController.setContinuous(true);
    }

    protected void initialize() {
        strafeController.enable();
        angleErrorController.enable();

        angleErrorController.setSetpoint(robot.getDrivetrain().getGyroAngle());

        finishTimer.stop();
        finishTimer.reset();

        // Enable snapshots
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        limelight.getEntry("snapshot").setNumber(1);
    }


    protected void execute() {
        System.out.println("[INFO]: Vision error: " + strafeController.getError());

        if (Math.abs(strafeController.getError()) < 0.5)
            pidStrafeValue = 0;

        SmartDashboard.putNumber("Rotation Factor", rotationFactor);
        if (tx.getDouble(0) != 0 || ty.getDouble(0) != 0) {    //Else check that we are not already at the tar
            robot.getDrivetrain().holonomicDrive(0, pidStrafeValue, rotationFactor, false);
        } else {
            robot.getDrivetrain().holonomicDrive(0, 0, 0);
        }
    }

    protected boolean isFinished() {
        if (tv.getDouble(0) == 1 && strafeController.onTarget()) {
            if (!isFinishTimerRunning) {
                finishTimer.reset();
                finishTimer.start();
                isFinishTimerRunning = true;
                System.out.println("[INFO]: Starting timer");
            }

            System.out.println("[INFO]: Finish timer is at " + finishTimer.get());

            return finishTimer.hasPeriodPassed(FINISH_TIMER);
        }

        if (isFinishTimerRunning) {
            finishTimer.reset();
            finishTimer.stop();
            isFinishTimerRunning = false;
        }
        return false;
    }

    protected void end() {
        strafeController.disable();
        angleErrorController.disable();


        // Disable snapshots
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        limelight.getEntry("snapshot").setNumber(0);
    }
}
