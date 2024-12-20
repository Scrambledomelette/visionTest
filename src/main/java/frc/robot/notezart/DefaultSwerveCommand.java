package frc.robot.notezart;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultSwerveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;

    private Supplier<Double> xSupplier;
    private Supplier<Double> ySupplier;
    private Supplier<Double> zSupplier;
    private Supplier<Boolean> speakerAssistSupplier;
    private Supplier<Boolean> noteAssistSupplier;
    private Supplier<Boolean> ampAssistSupplier;
    private double xSpeed;
    private double ySpeed;
    private double zSpeed;

    private final PIDController yawController = new PIDController(20, 0, 0.5);
    private final double deadBand = 0.1;

    /**
     * 
     * @param swerveSubsystem imports the swerve subsystem.
     * @param xAxis the % value which increases the robot's position across the x axis of the field.
     * @param yAxis the % value which increases the robot's position across the y axis of the field.
     * @param zAxis the z axis on the controller
     * @param speakerAssistButton while held the chassis angle will automatically angle to the speaker based on odometry
     * @param noteAssistButton while held the chassis angle will automatically angle to the note detected by photonvision
     */
    public DefaultSwerveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> yAxis, 
    Supplier<Double> xAxis, Supplier<Double> zAxis) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xAxis;
        this.ySupplier = yAxis;
        this.zSupplier = zAxis;

        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void execute() {    

        ySpeed = MathUtil.applyDeadband(ySupplier.get(), deadBand);
        xSpeed = MathUtil.applyDeadband(xSupplier.get(), deadBand);

            double speed = MathUtil.applyDeadband(zSupplier.get(), deadBand) * swerveSubsystem.getMaxAngularSpeed();
            if (speed <=0 ) {
                zSpeed = -Math.pow(speed, 2);
            } else {
                zSpeed = Math.pow(speed, 2);
            }


        

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, -xSpeed, -zSpeed, swerveSubsystem.getGyroRotationRaw());
        SwerveModuleState[] swerveMduleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setTeleopModuleStates(swerveMduleStates);
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
