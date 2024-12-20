package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.notezart.BotPoseSource;
import frc.robot.notezart.SwerveSubsystem;

public class Aprilometry extends SubsystemBase implements BotPoseSource {
    private final SwerveSubsystem swerveSubsystem;
    private final AprilTagSubsystem aprilTagSubsystem;
    private Boolean isUsingCameras;
    private Field2d field;

    public Aprilometry(SwerveSubsystem swerveSubsystem, AprilTagSubsystem aprilTagSubsystem, boolean isUsingCameras) {
        this.swerveSubsystem = swerveSubsystem;
        this.aprilTagSubsystem = aprilTagSubsystem;
        this.isUsingCameras = isUsingCameras;

        field = new Field2d();
    }

    @Override
    public void periodic() {
        double interPoseX = 0;
        double interPoseY = 0;
        double camerasChecked = 0;

        if (isUsingCameras) {
      
            // todo: change number of active cameras to be number of cameras with a target.
            for (int camera = 0; camera < aprilTagSubsystem.getNumberOfActiveCameras(); camera++) {
                Pose2d pose = swerveSubsystem.getPose().exp(aprilTagSubsystem.getTrustPoseError(
                    aprilTagSubsystem.protoCameraMatrix(camera, 0), 
                    aprilTagSubsystem.getRawPoseError(camera, swerveSubsystem.getPose())
                ));

                interPoseX = (interPoseX + pose.getX());
                interPoseY = (interPoseY + pose.getY());  
                camerasChecked++;
            }

            interPoseX = (interPoseX / camerasChecked);
            interPoseY = (interPoseY / camerasChecked);

            Pose2d estimatedRobotPose = new Pose2d(interPoseX, interPoseY, swerveSubsystem.getGyroRotation());

            swerveSubsystem.setPose(estimatedRobotPose);
            field.setRobotPose(estimatedRobotPose);
        }

        super.periodic();
    }

    @Override
    public Pose2d getPose() {
        return field.getRobotPose();
    }

    @Override
    public void setPose(Pose2d newPose) {
        swerveSubsystem.setPose(newPose);
    }  
}
