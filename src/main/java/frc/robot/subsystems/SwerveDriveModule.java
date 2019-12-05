package frc.robot.subsystems;

/* Orig and no longer used
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
*/

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;  // Eric, and no longer available in Rev Robotics library
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.SwerveModuleCommand;
// import frc.robot.commands.SwerveModuleCommandSparkTalon;
import frc.robot.util.MotorStallException;

public class SwerveDriveModule extends Subsystem {
    // 42 ticks per rev (embedded encoder in Neo motor)
    private static final double DRIVE_SENSOR_TICKS_PER_REV = 42.0;
    // standard MK2 gear ratios 42:14, 18:26, 60:15
    // 4" dia wheel
    //private static final double DRIVE_TICKS_PER_INCH = DRIVE_SENSOR_TICKS_PER_REV * 42.0/14.0 * 18.0/26.0 * 60.0/15.0 / ( 4.0*Math.PI);
    private static final double DRIVE_TICKS_PER_INCH = 42.0/14.0 * 18.0/26.0 * 60.0/15.0 / ( 4.0*Math.PI);
    private static final long STALL_TIMEOUT = 2000;

    private long mStallTimeBegin = Long.MAX_VALUE;

    private double mLastError = 0, lastTargetAngle = 0;

    private final int moduleNumber;

    private final double mZeroOffset;

    // Spark Max controllers, sensors, PID
    // private final TalonSRX mAngleMotor; // Orig
    private final CANSparkMax mAngleMotor;
    private CANPIDController m_pidControllerAngle;  // new for all Spark Max controllers
    private CANAnalog m_analogSensorAngle;  // new for all Spark Max controllers
    private double angle_kP;  // PID initial values are set in SwerveDriveSubsystem
    private double angle_kI;
    private double angle_kD;

    private final CANSparkMax mDriveMotor; 
    private CANPIDController m_pidControllerDrive;  // new for all Spark Max controllers
    private CANEncoder m_encoderDrive;  // new for all Spark Max controllers
    

    private boolean driveInverted = false;
    private double driveGearRatio = 1;
    private double driveWheelRadius = 2;
    private boolean angleMotorJam = false;

    public static final double SWERVE_1_ENC_MIN = 0.0; 
    // input to breakout is 3.3 but it expects max 5.0 and then downconverts to max 3.3
	public static final double SWERVE_1_ENC_MAX = 3.3 * 3.3 / 5.0; 
	
    private double ANGLE_SENSOR_MAX_VOLTAGE = SWERVE_1_ENC_MAX;
    private double ANGLE_SENSOR_MIN_VOLTAGE = SWERVE_1_ENC_MIN;
    private double ANGLE_SENSOR_RANGE = ANGLE_SENSOR_MAX_VOLTAGE - ANGLE_SENSOR_MIN_VOLTAGE;
    public long bfc = 0;

    //public SwerveDriveModule(int moduleNumber, TalonSRX angleMotor, CANSparkMax driveMotor, double zeroOffset) {
    public SwerveDriveModule(int moduleNumber, CANSparkMax angleMotor, CANSparkMax driveMotor, double zeroOffset) {        this.moduleNumber = moduleNumber;
        
        mAngleMotor = angleMotor;
        mDriveMotor = driveMotor;

        // 10/26/19 reset Spark Maxes
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.clearFaults();
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.clearFaults();

        //the angle the pot has to be offset to be "straight" at input 0 degrees.
        mZeroOffset = zeroOffset;

        /* Original angleMotor = Talon
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        angleMotor.setSensorPhase(true);
        angleMotor.config_kP(0, 30, 0);
        angleMotor.config_kI(0, 0.001, 0);
        angleMotor.config_kD(0, 200, 0);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.set(ControlMode.Position, 0);
        */

        // New angleMotor controller = Spark Max:
        // set feedback device to analog, setSensorPhase, set position control mode
        // Mk2SwerveModule.java for 2910's 2019 robot has
        // ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001); in P, I, D order
        m_analogSensorAngle = angleMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);
        m_analogSensorAngle.setPositionConversionFactor( 1.0 / ANGLE_SENSOR_RANGE); // sets getPosition to return in range [ 0, 1) for one full rotation
        
        m_analogSensorAngle.setInverted(true);

        m_pidControllerAngle = angleMotor.getPIDController();
        m_pidControllerAngle.setFeedbackDevice(m_analogSensorAngle);
        angleMotor.setMotorType(MotorType.kBrushless);
        m_pidControllerAngle.setP( angle_kP); // 0.5);
        m_pidControllerAngle.setI( angle_kI); // 0.0); 
        m_pidControllerAngle.setD( angle_kD); // 0.0001);  
        angleMotor.setIdleMode(IdleMode.kBrake);
        // prevent more than this many amps to the motor
        // default is 80, which Rev "thinks is a pretty good number for a drivetrain" per Chief Delphi
        // 60 is from 2910's 2019 Mk2SwerveModule.java for drive motor
        angleMotor.setSmartCurrentLimit(60); 

        // new driveMotor controller = Spark Max
        driveMotor.setMotorType(MotorType.kBrushless);

        /* Orig from Eric's code
        driveMotor.setParameter(ConfigParameter.kP_0, 15);
        driveMotor.setParameter(ConfigParameter.kI_0, 0.01);
        driveMotor.setParameter(ConfigParameter.kD_0, 0.1);
        driveMotor.setParameter(ConfigParameter.kF_0, 0.2);
        */
        // new drive controller PID settings, from Rev Robotics example code
        m_encoderDrive = driveMotor.getEncoder(); // EncoderType.kQuadrature, 4096);
        //m_encoderDrive.setPositionConversionFactor( 1.0 / DRIVE_SENSOR_TICKS_PER_REV); // sets getPosition to return in range [ 0, 42) for one full rotation
        m_encoderDrive.setPositionConversionFactor( 1.0 ); // sets getPosition to return in range [ 0, 42) for one full rotation
        m_pidControllerDrive = driveMotor.getPIDController();
        m_pidControllerDrive.setFeedbackDevice(m_encoderDrive);
     
        m_pidControllerDrive.setP(15);
        m_pidControllerDrive.setI(0.01);
        m_pidControllerDrive.setD(0.1);
        m_pidControllerDrive.setFF(0.2);


        //set frame..?
        //driveMotor.setControlFramePeriodMs(periodMs);
        
        driveMotor.setIdleMode(IdleMode.kBrake);
        // prevent more than this many amps to the motor
        // default is 80, which Rev "thinks is a pretty good number for a drivetrain" per Chief Delphi
        // 60 is from 2910's 2019 Mk2SwerveModule.java for drive motor
        driveMotor.setSmartCurrentLimit(60); 
        // driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
        // driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

        // driveMotor.config_kP(0, 15, 0);
        // driveMotor.config_kI(0, 0.01, 0);
        // driveMotor.config_kD(0, 0.1, 0);
        // driveMotor.config_kF(0, 0.2, 0);

        // driveMotor.configMotionCruiseVelocity(640, 0);
        // driveMotor.configMotionAcceleration(200, 0);

        // driveMotor.setNeutralMode(NeutralMode.Brake);

        /*
        // Set amperage limits, Original (Talon)
        angleMotor.configContinuousCurrentLimit(30, 0);
        angleMotor.configPeakCurrentLimit(30, 0);
        angleMotor.configPeakCurrentDuration(100, 0);
        angleMotor.enableCurrentLimit(true);
        */

        // Set amperage limits
        angleMotor.setSmartCurrentLimit(25, 25);
        driveMotor.setSmartCurrentLimit(25, 25);

        // driveMotor.configContinuousCurrentLimit(25, 0);
        // driveMotor.configPeakCurrentLimit(25, 0);
        // driveMotor.configPeakCurrentDuration(100, 0);
        // driveMotor.enableCurrentLimit(true);
        
    	SmartDashboard.putBoolean("Motor Jammed" + moduleNumber, angleMotorJam);
    }

    /**
     * Compute and return the number of inches traveled by the robot given a 
     * given a number of encoder ticks read.
     * 
     * @return inches traveled
     */
    private double encoderTicksToInches(double ticks) {
        return ticks / DRIVE_TICKS_PER_INCH;
    }

    /**
     * Compute and return the number of ticks the encoder will have to move
     * given the number of inches you want the robot to move.
     * 
     * @return number of ticks (int)
     */
    private int inchesToEncoderTicks(double inches) {
        return (int) Math.round( inches * DRIVE_TICKS_PER_INCH);
    }

    @Override
    protected void initDefaultCommand() {
        // setDefaultCommand(new SwerveModuleCommandSparkTalon(this));  // Orig from Eric
        setDefaultCommand(new SwerveModuleCommand(this)); // new for all Spark Max controllers
    }

    public CANSparkMax getAngleMotor() {
        return mAngleMotor;
    }

    public void setAngleKP( double k) {
        angle_kP = k;
        m_pidControllerAngle.setP( k); // 0.5);        
    }
    public void setAngleKI( double k) {
        angle_kI = k;
        m_pidControllerAngle.setI( k); // 0.0);
    }
    public void setAngleKD( double k) {
        angle_kD = k;
        m_pidControllerAngle.setD( k); // 1e-6 or something small
    }
 

    public double getRawSensorPosition() {
        return m_analogSensorAngle.getPosition();
    }
    /**
     * Get the current angle of the swerve module
     * corrected for zero offset
     *
     * @return An angle in the range [0, 360)
     */
    public double getCurrentAngle() {
        // double angle = mAngleMotor.getSelectedSensorPosition(0) * (360.0 / 1024.0); // orig
        // new all spark max controllers and based on 2910's 2019 code
        // double angle = ( 1.0 - (m_analogSensorAngle.getPosition() - ANGLE_SENSOR_MIN_VOLTAGE) / ANGLE_SENSOR_RANGE) * 360.0; 
        // double angle = ( 1.0 - m_analogSensorAngle.getPosition()) * 360.0; // getPosition returning a value in [0,1)
        double angle = ( m_analogSensorAngle.getPosition()) * 360.0; // getPosition returning a value in [0,1)
        angle -= mZeroOffset;
        angle %= 360;
        if (angle < 0) angle += 360;

        return angle;

              /*
              // from 2910's 2019 code in
              // Common-Public/robot/src/main/java/org/frcteam2910/common/robot/drivers/Mk2SwerveModule.java
              // They switched to radians.
              readAngle() {
              double angle = (1.0 - angleEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + angleOffset;
              angle %= 2.0 * Math.PI;
              if (angle < 0.0) {
                  angle += 2.0 * Math.PI;
              }
      
              return angle; }
          */

          

    }

    /** 
     * Get the raw voltage from the angle analog sensor 
     * 
     * @return Raw voltage from sensor, presumably in range either [0, 3] or maybe [0, 5]
     */
    public double getAngleVoltage() {
        return m_analogSensorAngle.getVoltage();
    }

    public double getDriveDistance() {
        double ticks = mDriveMotor.getEncoder().getPosition();
        if (driveInverted)
            ticks = -ticks;

        return encoderTicksToInches(ticks);
    }

    /**
     * Get the raw position from the embedded drive motor's encoder.  
     * If you've not set the Position Conversion Factor, (i.e. default) the return is in rotations.  
     */
    public double getDrivePosition() {
        return m_encoderDrive.getPosition();
    }

    /**
     * Get the raw velocity from the embedded drive motor's encoder.
     * If you've not set the Velocity Conversion Factor, (i.e. default) the return is in RPMs.
     */
    public double getDriveVelocity() {
        return m_encoderDrive.getVelocity();
    }


    public CANSparkMax getDriveMotor() {
        return mDriveMotor;
    }

    public double getTargetAngle() {
        return lastTargetAngle;
    }

    public void robotDisabledInit() {
        mStallTimeBegin = Long.MAX_VALUE;
    }

    public void setDriveGearRatio(double ratio) {
        driveGearRatio = ratio;
    }

    public void setDriveInverted(boolean inverted) {
        driveInverted = inverted;
    }

    public double getDriveWheelRadius() {
        return driveWheelRadius;
    }

    public void setDriveWheelRadius(double radius) {
        driveWheelRadius = radius;
    }

    public void setTargetAngle(double targetAngle) {
//    	if(angleMotorJam) {
//    		mAngleMotor.set(ControlMode.Disabled, 0);
//    		return;
//    	}
        lastTargetAngle = targetAngle;

        targetAngle %= 360;

        SmartDashboard.putNumber("Module " + moduleNumber + " Target Angle Desired ", targetAngle % 360);

        targetAngle += mZeroOffset;

        // double currentAngle = mAngleMotor.getSelectedSensorPosition(0) * (360.0 / 1024.0); // orig
        // double currentAngle = (m_analogSensorAngle.getPosition() - ANGLE_SENSOR_MIN_VOLTAGE) * (360.0 / ANGLE_SENSOR_RANGE); // new all spark max controllers
        // double currentAngle = ( 1.0 - (m_analogSensorAngle.getPosition() - ANGLE_SENSOR_MIN_VOLTAGE) / ANGLE_SENSOR_RANGE) * 360.0; 
        // double currentAngle = ( 1.0 - m_analogSensorAngle.getPosition()) * 360.0; // getPosition returning a value in [0,1)
        double currentAngle = ( m_analogSensorAngle.getPosition()) * 360.0; // getPosition returns a value in [0,1)
        
        double currentAngleMod = currentAngle % 360;
        if (currentAngleMod < 0) currentAngleMod += 360;

        double delta = currentAngleMod - targetAngle;

        if (delta > 180) {
            targetAngle += 360;
        } else if (delta < -180) {
            targetAngle -= 360;
        }

        delta = currentAngleMod - targetAngle;
        if (delta > 90 || delta < -90) {
            if (delta > 90)
                targetAngle += 180;
            else if (delta < -90)
                targetAngle -= 180;
            mDriveMotor.setInverted(false);
        } else {
            mDriveMotor.setInverted(true);
        }

        targetAngle += currentAngle - currentAngleMod;

        // double currentError = mAngleMotor.getClosedLoopError(0);  // new commented out 10/26/19 hoping not needed
//        if (Math.abs(currentError - mLastError) < 7.5 &&
//                Math.abs(currentAngle - targetAngle) > 5) {
//            if (mStallTimeBegin == Long.MAX_VALUE) {
//            	mStallTimeBegin = System.currentTimeMillis();
//            }
//            if (System.currentTimeMillis() - mStallTimeBegin > STALL_TIMEOUT) {
//            	angleMotorJam = true;
//            	mAngleMotor.set(ControlMode.Disabled, 0);
//            	mDriveMotor.set(ControlMode.Disabled, 0);
//            	SmartDashboard.putBoolean("Motor Jammed" + moduleNumber, angleMotorJam);
//            	return;
//            }
//        } else {
//            mStallTimeBegin = Long.MAX_VALUE;
//        }
        // mLastError = currentError;  // new commented out 10/26/19 hoping not needed
        // targetAngle *= 1024.0 / 360.0;
        targetAngle = targetAngle  / 360.0; // * 3.3;  // changed 11/13/19 to be range of [0, 1) 11/29 range [0, 3.3)
        // mAngleMotor.set(ControlMode.Position, targetAngle); // orig
        m_pidControllerAngle.setReference(targetAngle, ControlType.kPosition); // new for all Spark Max controllers
        SmartDashboard.putNumber("Module " + moduleNumber + " Target Angle Set ", targetAngle);

    }

    public void setTargetDistance(double distance) {
//    	if(angleMotorJam) {
//    		mDriveMotor.set(ControlMode.Disabled, 0);
//    		return;
//    	}
        if (driveInverted) distance = -distance;

//        distance /= 2 * Math.PI * driveWheelRadius; // to wheel rotations
//        distance *= driveGearRatio; // to encoder rotations
//        distance *= 80; // to encoder ticks

        distance = inchesToEncoderTicks(distance);

        SmartDashboard.putNumber("Module " + moduleNumber + " Drive Ticks ", distance);

        // TODO: confirm the distance is set in the right units
        m_pidControllerDrive.setReference(distance, ControlType.kPosition);
        // mDriveMotor.set(ControlMode.MotionMagic, distance);
    }

    public void setTargetSpeed(double speed) {
//    	if(angleMotorJam) {
//    		mDriveMotor.set(ControlMode.Disabled, 0);
//    		return;
//    	}
        if (driveInverted) speed = -speed;
        
        // mDriveMotor.set(ControlMode.PercentOutput, speed);  // all Talons
        mDriveMotor.set(speed);  
        // 10/26/19:
        // If for some reason the simple approach of setting the speed 
        // as the voltage percent, above, doesn't work, maybe it will be
        // better to set the motor's speed using the PID controller, i.e.:
        // m_pidControllerDrive.setReference(speed, ControlType.kVelocity);
    }

    public void zeroDistance() {
        // mDriveMotor.setEncPosition(0); // Eric
        m_encoderDrive.setPosition(0);
    }
    
    public void resetMotor() {
    	angleMotorJam = false;
    	mStallTimeBegin = Long.MAX_VALUE;
    	SmartDashboard.putBoolean("Module " + moduleNumber + " Angle Motor Jammed ", angleMotorJam);
    }

    /**
     *  Set the max acceleration and velocity for Smart Motion (Magic Motion for Talons)
     * TODO: Need to look up the Talon command and then find the equivalent Spark Max  
     * command (for current version of API), 
     * then insert the Spark Max command here.  Be very careful about units-- make 
     * sure to write this code to make the robot do, via the Spark Max, exactly what it
     * was built and programmed to do with the Talons.
     * @param maxAcceleration
     * @param maxVelocity
     */
    public void setMotionConstraints(double maxAcceleration, double maxVelocity) {
        /* Eric's: [Eric:] Units all wrong...
        mDriveMotor.setParameter(ConfigParameter.kSmartMotionMaxAccel_0, maxAcceleration);
        mDriveMotor.setParameter(ConfigParameter.kSmartMotionMaxVelocity_0, maxVelocity); */

        // mDriveMotor.configMotionAcceleration(inchesToEncoderTicks(maxAcceleration * 12) / 10, 0);
        // mDriveMotor.configMotionCruiseVelocity(inchesToEncoderTicks(maxVelocity * 12) / 10, 0);

        m_pidControllerDrive.setSmartMotionMaxAccel(inchesToEncoderTicks(maxAcceleration * 12) / 10, 0);
        m_pidControllerDrive.setSmartMotionMaxVelocity(inchesToEncoderTicks(maxVelocity * 12) / 10, 0);
    }
}
