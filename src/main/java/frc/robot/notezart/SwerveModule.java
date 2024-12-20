package frc.robot.notezart;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    //#region -*-*- Constants -*-*-

    // 3.884
    // 3.881
    // 3.877
    // 3.865
    // 3.825
    // 3.79

    // 3.76
    // 3.71
    private final double WHEEL_CIRCUMPHERENCE = (Units.inchesToMeters(3.7) * Math.PI);

    private final double DRIVE_GEAR_RATIO = 6.75;

    private double drivePosConversionFactor = WHEEL_CIRCUMPHERENCE / DRIVE_GEAR_RATIO;
    private double driveVelocityConversionFactor = drivePosConversionFactor  / 60;

    private final double maxMetersPerSec = 4;

    private final double directionPID_P = 0.25;
    private final double directionPID_I = 0;
    private final double directionPID_D = 0;
    //#endregion

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final CANSparkMax directionMotor;
    private final CANcoder directionEncoder;
    private final PIDController directionPID = new PIDController(directionPID_P, directionPID_I, directionPID_D);
    // private SwerveModuleState optimizedState;
    // private double driveMotorPower;
    // private double directionMotorPower;
    private final PIDController drivePIDController = new PIDController(.5, 0, 0);
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.5, 1.2);

    /**
     * A custom-made class that instantiates each individual module of the swerve, 
     * allowing control of each individual module as called seperately
     * @param driveMotorID the set ID of the CANSparkMax on the motor controlling the driving power of the module
     * @param directionMotorID the set ID of the CANSparkMax on the motor controlling direction of the module
     * @param CANcoderID the set ID of the CANCoder (Phoenix Tuner) dedicated to encoder control of the direction module
     * @param isDriveMotorReversed If the power of the drive motor's CANSparkMax is set to kInverted
     * @param isDirectionEncoderReversed if the power of the direction motor's CANSparkMax is set to kInverted
     */
    public SwerveModule(Integer driveMotorID, Integer directionMotorID, Integer CANcoderID, 
    boolean isDriveMotorReversed, boolean isDirectionEncoderReversed, double driveEncoderErrorFactor){

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        directionMotor = new CANSparkMax(directionMotorID, MotorType.kBrushless);

        driveMotor.setInverted(isDriveMotorReversed);
        directionMotor.setInverted(isDirectionEncoderReversed);

        driveEncoder = driveMotor.getEncoder();
        directionEncoder = new CANcoder(CANcoderID);

        drivePosConversionFactor *= (1 - driveEncoderErrorFactor);
        driveVelocityConversionFactor = drivePosConversionFactor  / 60;

        driveEncoder.setPositionConversionFactor(drivePosConversionFactor);
        driveEncoder.setVelocityConversionFactor(driveVelocityConversionFactor);

        directionPID.enableContinuousInput(-Math.PI, Math.PI);

        /** for testing. remove before merge. */
        driveMotor.setSmartCurrentLimit(40);
        directionMotor.setSmartCurrentLimit(40);

    }

    // #region -*-*-*-*-*-  gets -*-*-*-*-*-

    /**
     * @return the absolute value off of the swerve module's CANCoder.
     */
    public double getAbsoluteDirectionPos(){
        return -directionEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * @return the position of the swerve module in degrees at a 0-360 range.
     */
    public double getDirectionPosDeg(){
        return (getAbsoluteDirectionPos()) * 360;
    }

    /**
     * @return the position of the swerve module in radians at a 0 to 2 PI range.
     */
    public double getDirectionPosRad(){
        return Math.toRadians(getDirectionPosDeg());
    }

    /**
     * @return the aggregate gain of the module's drive encoder.
     */
    public double getDrivePos(){
        return -driveEncoder.getPosition();
    }

    /**
     * @return the current drive velocity of the module.
     */
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    /**
     * @return the relative field position of the module as a SwerveModulePosition.
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePos(), new Rotation2d(getDirectionPosRad()));
    }

    /**
     * @return the current "state" of the Swerve module in a SwerveModuleState.
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getDirectionPosRad()));
    }

    /**
     * @return The bus voltage.
     */
    public double getBusVolts() {
        return driveMotor.getBusVoltage();
    }

    // #endregion

    // #region -*-*-*-*-*-  sets  -*-*-*-*-*-

    /**
     * sets the drive power and turn angle according to the inputted desired state.
     * @param state the desired state to input
     */
    public void setAutonDesiredState(SwerveModuleState state){
        double busVoltage = driveMotor.getBusVoltage();

        Rotation2d currentHeading = Rotation2d.fromRadians(getDirectionPosRad());
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, currentHeading);

        // Calculate the drive output from the drive PID controller.
        final double driveFeedbackV = drivePIDController.calculate(driveEncoder.getVelocity(), optimizedState.speedMetersPerSecond);
        final double driveFeedforwardV = driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
        double driveOutputV = MathUtil.clamp(driveFeedbackV + driveFeedforwardV, -busVoltage, busVoltage);

        // Calculate the turning motor output from the turning PID controller.
        double turnOutput = directionPID.calculate(currentHeading.getRadians(), optimizedState.angle.getRadians());

        if (Math.abs(optimizedState.speedMetersPerSecond) <= (maxMetersPerSec * 0.001)) {
            driveOutputV = 0;
            turnOutput = 0;
        }

        driveMotor.setVoltage(-driveOutputV);
        directionMotor.set(MathUtil.clamp(turnOutput, -1, 1));

    }

    public void setTeleopDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        double driveMotorPower = optimizedState.speedMetersPerSecond;
        double directionMotorPower = directionPID.calculate(getDirectionPosRad(), optimizedState.angle.getRadians());
        if(Math.abs(driveMotorPower) < 0.01){
            stop();
        }
        
        driveMotor.set(-driveMotorPower);
        directionMotor.set(directionMotorPower);
    }

    // #endregion

    // #region -*-*-*-*-*- other *-*-*-*-*-*-

    /** sets all the drive encoders to zero. Does not affect the odometry. */
    public void resetDriveEncoder(){
        driveEncoder.setPosition(0);
    }

    /** sets the drive and direction motor speeds to zero. */
    public void stop(){
        driveMotor.stopMotor();
        directionMotor.stopMotor();
    }

    /** sets the idle mode of the drive and direction motors to brake. */
    public void setBrakeMode(){
        driveMotor.setIdleMode(IdleMode.kBrake);
        directionMotor.setIdleMode(IdleMode.kBrake);
    }

    /** sets the idle mode of the drive and direction motors to coast. */
    public void setCoastMode(){
        driveMotor.setIdleMode(IdleMode.kCoast);
        directionMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setAutonCurrentLimits() {
        driveMotor.setSmartCurrentLimit(40);
        directionMotor.setSmartCurrentLimit(40);
    }

    public void setTeleopCurrentLimites() {
        driveMotor.setSmartCurrentLimit(80);
        directionMotor.setSmartCurrentLimit(20);
    }

    //#endregion
}