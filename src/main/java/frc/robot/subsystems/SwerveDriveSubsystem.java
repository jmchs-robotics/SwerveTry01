package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import static frc.robot.RobotMap.*;

public class SwerveDriveSubsystem extends HolonomicDrivetrain {
    // TODO: put our dimensions here
    public static final double WHEELBASE = 14.5;  // Swerve bot: 14.5 Comp bot: 20.5
    public static final double TRACKWIDTH = 13.5; // Swerve bot: 13.5 Comp bot: 25.5

    public static final double WIDTH = 20;  // Swerve bot: 20 Comp bot: 37
    public static final double LENGTH = 19; // Swerve bot: 19 Comp bot: 32

    private double angle_kP = 3.0;
    private double angle_kI = 0.0;
    private double angle_kD = 0.0;

	/*
	 * 0 is Front Left
	 * 1 is Front Right
	 * 2 is Back Right
	 * 3 is Back Left
	 */
    private SwerveDriveModule[] mSwerveModules;

    private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);

    public SwerveDriveSubsystem() {
        super(WIDTH, LENGTH);
        zeroGyro();
        
        // 10/26/19 Big Switch from Talon to Spark Max controllers
        mSwerveModules = new SwerveDriveModule[] {
            new SwerveDriveModule(0,
                new CANSparkMax(DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, MotorType.kBrushless),
                new CANSparkMax(DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless),
                87.890), 
            new SwerveDriveModule(1,
                new CANSparkMax(DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, MotorType.kBrushless),
                new CANSparkMax(DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless),
                235.195),
            // 10/26/19 need to change the other 2 modules to SparkMax
            new SwerveDriveModule(2,
                new CANSparkMax(DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, MotorType.kBrushless),
                new CANSparkMax(DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless),
                320.976),
            new SwerveDriveModule(3,
                new CANSparkMax(DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, MotorType.kBrushless),
                new CANSparkMax(DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless),
                245.742), 
        };
/*
            // 2910's 2018 code.  We may need to do somethig similar for SwervyJr
            mSwerveModules[0].setDriveInverted(true);
            mSwerveModules[3].setDriveInverted(true);
            */
      //  }

        for (SwerveDriveModule module : mSwerveModules) {
            module.setTargetAngle(0);
            module.setDriveGearRatio(5.7777);
            module.setDriveWheelRadius(module.getDriveWheelRadius() * 1.05);
            module.setMotionConstraints(getMaxAcceleration(), getMaxVelocity());
            module.setAngleKD(angle_kD);
            module.setAngleKI(angle_kI);
            module.setAngleKP(angle_kP);
        }
    }

    public double[] calculateSwerveModuleAngles(double forward, double strafe, double rotation) {
        if (isFieldOriented()) {
            double angleRad = Math.toRadians(getGyroAngle());
            double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
            strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
            forward = temp;
        }

        double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
        double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
        double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
        double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

        return new double[]{
                Math.atan2(b, c) * 180 / Math.PI,
                Math.atan2(b, d) * 180 / Math.PI,
                Math.atan2(a, d) * 180 / Math.PI,
                Math.atan2(a, c) * 180 / Math.PI
        };
    }

    public AHRS getNavX() {
        return mNavX;
    }

    public double getGyroAngle() {
        double angle = mNavX.getAngle() - getAdjustmentAngle();
        angle %= 360;
        if (angle < 0) angle += 360;

        if (Robot.PRACTICE_BOT) {
            return angle;
        } else {
            return 360 - angle;
        }
    }

    public double getGyroRate() {
        return mNavX.getRate();
    }

    public double getRawGyroAngle() {
        double angle = mNavX.getAngle();
        angle %= 360;
        if (angle < 0) angle += 360;

        return angle;
    }

    public SwerveDriveModule getSwerveModule(int i) {
        return mSwerveModules[i];
    }

    @Override
    public void holonomicDrive(double forward, double strafe, double rotation, boolean fieldOriented) {
        forward *= getSpeedMultiplier();
        strafe *= getSpeedMultiplier();

        if (fieldOriented) {
            double angleRad = Math.toRadians(getGyroAngle());
            double temp = forward * Math.cos(angleRad) +
                    strafe * Math.sin(angleRad);
            strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
            forward = temp;
        }

        double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
        double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
        double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
        double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

        double[] angles = new double[]{
                Math.atan2(b, c) * 180 / Math.PI,
                Math.atan2(b, d) * 180 / Math.PI,
                Math.atan2(a, d) * 180 / Math.PI,
                Math.atan2(a, c) * 180 / Math.PI
        };

        double[] speeds = new double[]{
                Math.sqrt(b * b + c * c),
                Math.sqrt(b * b + d * d),
                Math.sqrt(a * a + d * d),
                Math.sqrt(a * a + c * c)
        };

        double max = speeds[0];

        for (double speed : speeds) {
            if (speed > max) {
                max = speed;
            }
        }

        for (int i = 0; i < 4; i++) {
            if (Math.abs(forward) > 0.05 ||
                    Math.abs(strafe) > 0.05 ||
                    Math.abs(rotation) > 0.05) {
                mSwerveModules[i].setTargetAngle(angles[i] + 180);
            } else {
                mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
            }
            mSwerveModules[i].setTargetSpeed(speeds[i]);
        }
        SmartDashboard.putNumber("YahooYahooYahooYahooYahooYahooYahooYahoo YahooYahooYahooYahooYahooYahooYahooYahoo", 100);
    }

    //drive command for spark max and Talon controllers
    @Override
    public void holonomicDriveSparkTalon(double forward, double strafe, double rotation, boolean fieldOriented) {
        forward *= getSpeedMultiplier();
        strafe *= getSpeedMultiplier();

        if (fieldOriented) {
            double angleRad = Math.toRadians(getGyroAngle());
            double temp = forward * Math.cos(angleRad) +
                    strafe * Math.sin(angleRad);
            strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
            forward = temp;
        }

        double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
        double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
        double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
        double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

        double[] angles = new double[]{
                Math.atan2(b, c) * 180 / Math.PI,
                Math.atan2(b, d) * 180 / Math.PI,
                Math.atan2(a, d) * 180 / Math.PI,
                Math.atan2(a, c) * 180 / Math.PI
        };

        double[] speeds = new double[]{
                Math.sqrt(b * b + c * c),
                Math.sqrt(b * b + d * d),
                Math.sqrt(a * a + d * d),
                Math.sqrt(a * a + c * c)
        };

        double max = speeds[0];

        for (double speed : speeds) {
            if (speed > max) {
                max = speed;
            }
        }

        for (int i = 0; i < 4; i++) {
            if (Math.abs(forward) > 0.05 ||
                    Math.abs(strafe) > 0.05 ||
                    Math.abs(rotation) > 0.05) {
                    mSwerveModules[i].setTargetAngle(angles[i] + 180);
            } else {
                mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
            }
            mSwerveModules[i].setTargetSpeed(speeds[i]);
        }
    }

    @Override
    public void stopDriveMotors() {
        for (SwerveDriveModule module : mSwerveModules) {
            module.setTargetSpeed(0);
        }
    }
    
    public void resetMotors() {
    	for(int i = 0; i < mSwerveModules.length; i++) {
    		mSwerveModules[i].resetMotor();
    	}
    }

    public SwerveDriveModule[] getSwerveModules() {
        return mSwerveModules;
    }

    @Override
    public double getMaxAcceleration() {
        return 5.5;
    }

    @Override
    public double getMaxVelocity() {
        return 10; 
    }

    public double getAngleKP() {
        return angle_kP;
    }
    public double getAngleKI() {
        return angle_kI;
    }
    public double getAngleKD() {
        return angle_kD;
    }
    public void setAngleKP( double k) {
        angle_kP = k;
        for (int i = 0; i < 4; i++) {
            mSwerveModules[i].setAngleKP( k);
        }
    }
    public void setAngleKI( double k) {
        angle_kI = k;
        for (int i = 0; i < 4; i++) {
            mSwerveModules[i].setAngleKI( k);
        }
    }
    public void setAngleKD( double k) {
        angle_kD = k;
        for (int i = 0; i < 4; i++) {
            mSwerveModules[i].setAngleKD( k);
        }
    }

}
