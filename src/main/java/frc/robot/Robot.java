package frc.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.ResetMotorsCommand;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.autonomous.stage1.StartingPosition;
import frc.robot.commands.autonomous.stage2.VisionTargetingCubeCommand;
import frc.robot.motion.AutonomousPaths;
import frc.robot.subsystems.SwerveDriveModule;
//import frc.robot.subsystems.GathererSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Side;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	public static final boolean PRACTICE_BOT = false;

	public static final double FIELD_INFO_TIMEOUT = 5;

	public double modAngEncMax = 0;
	public double modAngEncMin = 100;

	private static OI mOI;
	private static SwerveDriveSubsystem swerveDriveSubsystem;
//	private static GathererSubsystem gathererSubsystem;

	private final AutonomousChooser autoChooser = new AutonomousChooser();
	private Command autoCommand;

	private Timer autoTimer;

	//Socket receivers. One is needed for each port to read from
	public static SocketVision rft_;		//5801
	//Socket constants
	public static final boolean SHOW_DEBUG_VISION = true;

	public static OI getOI() {
		return mOI;
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		mOI = new OI(this);

		// gathererSubsystem = new GathererSubsystem();
		swerveDriveSubsystem = new SwerveDriveSubsystem();
		// elevatorSubsystem = new ElevatorSubsystem();

		mOI.registerControls();

		SmartDashboard.putData("Reset Motors", new ResetMotorsCommand(swerveDriveSubsystem));

		SmartDashboard.putNumber("Angle kP ", swerveDriveSubsystem.getAngleKP());
		SmartDashboard.putNumber("Angle kI ", swerveDriveSubsystem.getAngleKI());
		SmartDashboard.putNumber("Angle kD ", swerveDriveSubsystem.getAngleKD());
}

    @Override
    public void robotPeriodic() {
		// display status of all 4 modules
        for (int i = 0; i < 4; i++) {
			SmartDashboard.putNumber("Module " + i + " Current Angle ", swerveDriveSubsystem.getSwerveModule(i).getCurrentAngle());
			SmartDashboard.putNumber("Module " + i + " Angle Raw Encoder Position ", swerveDriveSubsystem.getSwerveModule(i).getRawSensorPosition());
			double x = swerveDriveSubsystem.getSwerveModule(i).getAngleVoltage();
			SmartDashboard.putNumber("Module " + i + " Angle Encoder Voltage ", x); // getSelectedSensorPosition(0));
			
            SmartDashboard.putNumber("Module " + i + " Drive Dist ", (swerveDriveSubsystem.getSwerveModule(i).getDriveDistance()));
			SmartDashboard.putNumber("Module " + i + " Drive Applied Output ", swerveDriveSubsystem.getSwerveModule(i).getDriveMotor().getAppliedOutput()); // getMotorOutputPercent());
            SmartDashboard.putNumber("Module " + i + " Drive Position ", swerveDriveSubsystem.getSwerveModule(i).getDrivePosition()); // getDriveMotor().getSelectedSensorPosition(0));
			SmartDashboard.putNumber("Module " + i + " Drive Output Current ", swerveDriveSubsystem.getSwerveModule(i).getDriveMotor().getOutputCurrent()); // getMotorOutputPercent());
			SmartDashboard.putNumber("Module " + i + " Angle Motor Faults ", swerveDriveSubsystem.getSwerveModule(i).getAngleMotor().getFaults());
		}

		// for debugging and tuning initial swerve software (first module)
		double x = swerveDriveSubsystem.getSwerveModule(1).getAngleVoltage();
		if (x > modAngEncMax) {
				modAngEncMax = x; 
			}
		SmartDashboard.putNumber("Module 1 Endcoder Angle Max ", modAngEncMax);

		if (x < modAngEncMin) {
				modAngEncMin = x; 
			}
		SmartDashboard.putNumber("Module 1 Endcoder Angle Min ", modAngEncMin);
		
		/* Put angle PID onto Smart Dashboard, and read Smart Dashboard for changes to them */
		
		double k;
		k = SmartDashboard.getNumber( "Angle kP ", 0.0);
		if( k != swerveDriveSubsystem.getAngleKP()) {
			swerveDriveSubsystem.setAngleKP( k);
		}
		k = SmartDashboard.getNumber( "Angle kI ", 0.0);
		if( k != swerveDriveSubsystem.getAngleKI()) {
			swerveDriveSubsystem.setAngleKI( k);
		}
		k = SmartDashboard.getNumber( "Angle kD ", 0.0);
		if( k != swerveDriveSubsystem.getAngleKD()) {
			swerveDriveSubsystem.setAngleKD( k);
		}
		
		
		SmartDashboard.putNumber("Drivetrain Angle", swerveDriveSubsystem.getGyroAngle());

		/*
		// from 2910's 2018 code, left in comments as example for 2020
		SmartDashboard.putNumber("Elevator encoder", elevatorSubsystem.getEncoderValue());
		SmartDashboard.putNumber("Elevator height", elevatorSubsystem.getCurrentHeight() + Math.random() * 1e-9);
		SmartDashboard.putNumber("Elevator target", elevatorSubsystem.getTargetHeight() + Math.random() * 1e-9);
		SmartDashboard.putNumber("Elevator percent", elevatorSubsystem.getMotors()[0].getMotorOutputPercent() + Math.random() * 1e-9);
		SmartDashboard.putNumber("Elevator speed", elevatorSubsystem.getMotors()[0].getSelectedSensorVelocity(0));
		*/
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		for (int i = 0; i < 4; i++) {
			swerveDriveSubsystem.getSwerveModule(i).robotDisabledInit();
		}
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Super-simple autonomous command, first try 12/1/19.
	 * also simplified the autonomousChooser.getCommand() to only return "auto line" path
	 */
	@Override
	public void autonomousInit() {
		// open the socket connection to comminucate with the coprocessor, i.e. the UP Board
		socketVisionInit();

		autoTimer = new Timer();
		
		swerveDriveSubsystem.setFieldOriented( true);
		CommandGroup autoGroup = new CommandGroup();
		
		//
		// Testing various autonomous commands, and fixing and tuning them (PIDs etc) 191207
		//
		//autoGroup.addSequential( autoChooser.getCommand(this)); // , Side.LEFT, Side.LEFT); // switchSide, scaleSide); ignoring parameters in getCommand()
		// autoGroup.addSequential( new SetAngleCommand( swerveDriveSubsystem, 45));
		// autoGroup.addSequential( new WaitForTimerCommand( getAutoTimer(), 0.5));
		// autoGroup.addSequential( new DriveForDistanceCommand(swerveDriveSubsystem , -24.0, 24.0)); // , Side.LEFT, Side.LEFT); // switchSide, scaleSide); ignoring parameters in getCommand()
		// autoGroup.addSequential( new WaitForTimerCommand( getAutoTimer(), 1.0));
		autoGroup.addSequential( new SetDrivetrainAngleCommand( swerveDriveSubsystem, 90));
		// autoGroup.addSequential( new WaitForTimerCommand( getAutoTimer(), 0.3));
		// autoGroup.addSequential( new SetAngleCommand( swerveDriveSubsystem,90));
        // autoGroup.addSequential( new DriveForDistanceCommand(swerveDriveSubsystem , 12.0, 0)); // , Side.LEFT, Side.LEFT); // switchSide, scaleSide); ignoring parameters in getCommand()
		autoTimer.start();
		autoGroup.start();
	}
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	/*
	@Override
	public void autonomousInit() {
		autoTimer = new Timer();

		if (autoCommand != null)
			autoCommand.cancel();
//		gathererSubsystem.setRightArm(GathererSubsystem.Position.IN);
//		gathererSubsystem.setLeftArm(GathererSubsystem.Position.IN);

		// Sometimes the FMS doesn't give the game string right away.
		// Wait a little bit for the game string to be given.
		// If no game string is received, go to the auto line.

        System.out.println("[INFO]: Waiting for field info");
		Timer waitTimer = new Timer();
		waitTimer.start();

		while (DriverStation.getInstance().getGameSpecificMessage().isEmpty()
				&& !waitTimer.hasPeriodPassed(FIELD_INFO_TIMEOUT)) {
			Scheduler.getInstance().run();
		}

		swerveDriveSubsystem.setFieldOriented(true);

		if (waitTimer.hasPeriodPassed(FIELD_INFO_TIMEOUT)) {
		    System.err.printf("[ERROR]: Could not get field info in time (%.3f sec), running auto line%n", FIELD_INFO_TIMEOUT);

			autoCommand = new DriveForTimeCommand(swerveDriveSubsystem, 2.5, 0.5, 0);
		} else {
			String fieldString = DriverStation.getInstance().getGameSpecificMessage();

			System.out.printf("[INFO]: Got field info: '%s'%n", fieldString);

			Side switchSide = fieldString.charAt(0) == 'L' ? Side.LEFT : Side.RIGHT;
			Side scaleSide = fieldString.charAt(1) == 'L' ? Side.LEFT : Side.RIGHT;

			autoCommand = autoChooser.getCommand(this, switchSide, scaleSide);
		}

		autoTimer.start();
		
		autoCommand.start();
	}
	*/

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// open the socket connection to read/write the coprocessor, in case it wasn't already done in auto
		socketVisionInit();

		Command c = new SetAngleCommand(swerveDriveSubsystem,0);
		c.start();
		SmartDashboard.putNumber("WHERE IS MY MAYO!!!!@#%$%#$@#$", 1000000);
		if (autoCommand != null) autoCommand.cancel();

		for (int i = 0; i < 4; i++)
			swerveDriveSubsystem.getSwerveModule(i).zeroDistance();
		
		swerveDriveSubsystem.setFieldOriented( true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() { }

	public SwerveDriveSubsystem getDrivetrain() {
		return swerveDriveSubsystem;
	}


	//public GathererSubsystem getGatherer() {
	//	return gathererSubsystem;
	//}

	public Timer getAutoTimer() {
		return autoTimer;
	}


	/**
	 * This method properly instantiates and initializes sockets to read from and write to
	 * coprocessors. This needs to be called before
	 * any of these objects could be used -- so don't use them during disabled mode. This should be called during autonomous
	 * and teleop init methods. For ease of access, these objects are global and instantiated through the main class.
	 */
	private void socketVisionInit() {
		if (rft_ == null) {
			rft_ = new SocketVision("10.59.33.255", 5801);
			if (SHOW_DEBUG_VISION) {
				System.out.println("Vision to RFT started.");
			}
			rft_.start();

			if (!rft_.is_connected()) {
				if (!rft_.connect()) {
					if (SHOW_DEBUG_VISION) {
						System.err.println("socketVisionInit() Failed to connect to the Helmsman.");
					}
				} else {
					if (SHOW_DEBUG_VISION) {
						System.out.println("socketVisionInit() Connected. Love that mayo.");
					}
				}
			}
		}
	}
}
