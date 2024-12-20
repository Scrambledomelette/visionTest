package frc.robot;

import java.util.List;

import javax.management.ConstructorParameters;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilCam extends PhotonCamera {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); /* change to desired year */
    private PhotonPipelineResult pipelineResult;
    private PhotonPoseEstimator poseEstimator;
    private Pose3d estimatedRobotPose;
    private Transform3d cameraPosition;
    private double pipelineTimestamp = 0;
    private double updateTimestamp = 0;

    /**
     * 
     * @param cameraName The name of the camera. Needs to be the same name designated on Photonvision
     * @param cameraPosition The relative position of the camera from the designated Origin of the robot.
     * @param defaultPoseStrategy The default pose strategy (essentially what to filter vision by) for the camera
     */
    public AprilCam(String cameraName, Transform3d cameraPosition, PoseStrategy defaultPoseStrategy) {
        super(cameraName);
        this.cameraPosition = cameraPosition;

        poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            defaultPoseStrategy, 
            this,
            cameraPosition
        );

        estimatedRobotPose = new Pose3d();
    }

    private void updatePipelineResult() {
        pipelineResult = this.getLatestResult();
        pipelineTimestamp = pipelineResult.getTimestampSeconds();

    }

    private void updateEstimatedRobotPose() {
        if (pipelineResult.hasTargets() && (pipelineTimestamp != updateTimestamp)) {
            updateTimestamp = pipelineResult.getTimestampSeconds();
            estimatedRobotPose = poseEstimator.update(pipelineResult).orElseThrow().estimatedPose;
        }
    }

    public Pose3d getEstimatedRobotPose() {
        updatePipelineResult();
        updateEstimatedRobotPose();

        return estimatedRobotPose;
    }

    public double getNumberOfTargets() {
        updatePipelineResult();

        return pipelineResult.getTargets().size();
    }

    // public double getDistanceToTagDirect() {
    //     updatePipelineResult();
        
    //     if (pipelineResult.hasTargets()) {
    //     return PhotonUtils.calculateDistanceToTargetMeters(
    //         cameraPosition.getY(), 
    //         aprilTagFieldLayout.getTagPose(pipelineResult.getBestTarget().getFiducialId()).get().getY(),
    //         cameraPosition.getRotation().getY(), /*Y in rotation3Ds is the pitch */ 
    //         Math.toRadians(pipelineResult.getBestTarget().getPitch())
    //     );
    //     } else {
    //         return 9999999;
    //     }
    // }

    public double getDistanceToTagFloor() {
        if (pipelineResult.hasTargets()) {
            return PhotonUtils.getDistanceToPose(
                getEstimatedRobotPose().toPose2d(), 
                aprilTagFieldLayout.getTagPose(pipelineResult.getBestTarget().getFiducialId()).get().toPose2d()
            );
        } else {
            return 999999;
        }
    }

    /** yes, this is needed. */
    public boolean hasTargets() {
        updatePipelineResult();
        if (pipelineResult != null) {
            return pipelineResult.hasTargets();
        } else {
            System.out.println("pipeline is null!!");
            return false;
        }

    }
}
