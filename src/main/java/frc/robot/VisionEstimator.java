package frc.robot;

import java.net.Socket;

import javax.management.ValueExp;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionEstimator {
    private AprilCam camera1;
    private AprilCam camera2;

    private Pose2d originPose = new Pose2d();

    // magic number. decide later ;3
    private final double bufferDuration = 1;

    private TimeInterpolatableBuffer<Pose2d> kalmanBuffer; 
    private Matrix<N3,N3> trustMatrix = new Matrix<>(Nat.N3(), Nat.N3());

    public VisionEstimator(AprilCam camera1, AprilCam camera2) {
        this.camera1 = camera1;
        this.camera2 = camera2;

        originPose = new Pose2d();
        kalmanBuffer = TimeInterpolatableBuffer.createBuffer(bufferDuration);
    }

    // test stuff --------
    private Pose2d testPose1 = new Pose2d();
    private Pose2d testPose2 = new Pose2d();
    private final Vector<N3> originTrust = VecBuilder.fill(0.5, 0.5, 0.5);
    private final Vector<N3> cameraTrust = VecBuilder.fill(0.5, 0.5, 0.5);

    public VisionEstimator() {
        originPose = new Pose2d();
        kalmanBuffer = TimeInterpolatableBuffer.createBuffer(bufferDuration);
        initializeTrustMatrix(originTrust, cameraTrust);
    }

    public Pose2d testFilter(Pose2d testPoseA, Pose2d testPoseB) {
        testPose1 = testPoseA;
        testPose2 = testPoseB;

        Twist2d pose1Error = originPose.log(testPose1);
        Twist2d pose2Error = originPose.log(testPose2);

        Matrix<N3,N1> pose1Matrix = trustMatrix.times(VecBuilder.fill(pose1Error.dx,pose1Error.dy,pose1Error.dtheta));
        Matrix<N3,N1> pose2Matrix = trustMatrix.times(VecBuilder.fill(pose2Error.dx,pose2Error.dy,pose2Error.dtheta));

        Twist2d trust1Pose = new Twist2d(pose1Matrix.get(0, 0),pose1Matrix.get(1, 0), pose1Matrix.get(2, 0));
        Twist2d trust2Pose = new Twist2d(pose2Matrix.get(0, 0),pose2Matrix.get(1, 0), pose2Matrix.get(2, 0));

        Pose2d updatedPose1 = originPose.exp(trust1Pose);
        Pose2d updatedPose2 = originPose.exp(trust2Pose);

        double interposex = (updatedPose1.getX() + updatedPose2.getX()) / 2;
        double interposey = (updatedPose1.getY() + updatedPose2.getY()) / 2;
        Pose2d interPose = new Pose2d(interposex,interposey, new Rotation2d());
        SmartDashboard.putNumber("pose averages", interPose.getX());
        // Twist2d pose1Error = originPose.log(testPose1);
        // Twist2d pose2Error = originPose.log(testPose2);

        // Pose2d updatedPose1 = originPose.exp(pose1Error);
        // Pose2d updatedPose2 = originPose.exp(pose2Error);

        Twist2d interPoseError = originPose.log(interPose);
        SmartDashboard.putNumber("inter pose error", interPoseError.dx);
        Matrix<N3,N1> interPoseMatrix = trustMatrix.times(VecBuilder.fill(interPoseError.dx, interPoseError.dy, interPoseError.dtheta));
        Twist2d interTrustPose = new Twist2d(interPoseMatrix.get(0, 0), interPoseMatrix.get(1, 0), interPoseMatrix.get(2, 0));
        
        return originPose.exp(interTrustPose);

        // testPose1 = testPoseA;
        // testPose2 = testPoseB;

        // Twist2d pose1Error = testPose1.log(testPose2);

        // Matrix<N3,N1> poseMatrix = cameraTrust.times(VecBuilder.fill(pose1Error.dx,pose1Error.dy,pose1Error.dtheta));

        // Twist2d trustPose = new Twist2d(poseMatrix.get(0, 0),poseMatrix.get(1, 0), poseMatrix.get(2, 0));

        // SmartDashboard.putNumber("poseexp", originPose.exp(trustPose).getX());
        // SmartDashboard.putNumber("poseeyp", originPose.exp(trustPose).getY());

        // kalmanBuffer.addSample(Timer.getFPGATimestamp(), originPose.exp(trustPose));
        
        // return kalmanBuffer.getSample(Timer.getFPGATimestamp()).get();
        
    }

    // test stuff --------

    public void resetData() {
        originPose = new Pose2d();
        kalmanBuffer.clear();
    }
    
    public void setOriginPose(Pose2d newOriginPose) {
        originPose = newOriginPose;
    }

    // this is a primitive version of the intended trust matrix. Final product will 
    // change the trust level based on camera data on the fly.
    private void initializeTrustMatrix(Matrix<N3,N1> camera1Stds, Matrix<N3,N1> camera2Stds) {
        Matrix<N3, N1> cam1Stds = camera1Stds;
        Matrix<N3, N1> cam2Stds = camera2Stds;

        for (int column = 0; column < 3; column++) {
            cam1Stds.set(column, 0, cam1Stds.get(column, 0) * cam1Stds.get(column, 0));
            cam2Stds.set(column, 0, cam2Stds.get(column, 0) * cam2Stds.get(column, 0));
        }

        for (int row = 0; row < 3; row++) {
            if (cam1Stds.get(row, 0) == 0.0) {
                trustMatrix.set(row, row, 0.0);
            } else {
                trustMatrix.set(
                    row, 
                    row, 
                    cam1Stds.get(row, 0) / (cam1Stds.get(row, 0) + (Math.sqrt(cam1Stds.get(row, 0)) * cam2Stds.get(row, 0)))
                );
            }
        }
    }

    private boolean isOneMainCamera() {
        if (camera1.getDistanceToTagFloor() > camera2.getDistanceToTagFloor()) {
            return false;
        } else {
            return true; 
        }
    }

    private void updateTrustMatrix() {
        Matrix<N3,N1> mainCameraMatrix = new Matrix<>(Nat.N3(), Nat.N1());
        Matrix<N3,N1> alternateCameraMatrix = new Matrix<>(Nat.N3(), Nat.N1());

        for (int column = 0; column < 3; column++) {
            mainCameraMatrix.set(column, 0, 0.5);
            alternateCameraMatrix.set(column, 0, 0.5);
        }
        
        for (int row = 0; row < 3; row++) {
            trustMatrix.set(
                row, 
                row, 
                mainCameraMatrix.get(row, 0) / (mainCameraMatrix.get(row, 0) + (Math.sqrt(mainCameraMatrix.get(row, 0)) * alternateCameraMatrix.get(row, 0)))
            );
        }
    }

    private void updateFilter() {
        Pose2d mainPose;
        Pose2d alternatePose;

        if (isOneMainCamera()) {
            mainPose = camera1.getEstimatedRobotPose().toPose2d();
            alternatePose = camera2.getEstimatedRobotPose().toPose2d();
        } else {
            mainPose = camera2.getEstimatedRobotPose().toPose2d();
            alternatePose = camera2.getEstimatedRobotPose().toPose2d();
        }
            
        Twist2d poseError = mainPose.log(alternatePose);

        Matrix<N3,N1> poseMatrix = trustMatrix.times(VecBuilder.fill(poseError.dx,poseError.dy,poseError.dtheta));

        Twist2d trustPose = new Twist2d(poseMatrix.get(0, 0),poseMatrix.get(1, 0), poseMatrix.get(2, 0));

        kalmanBuffer.addSample(Timer.getFPGATimestamp(), mainPose.exp(trustPose));
    }

    public Pose2d getEstimatedPose() {
        return kalmanBuffer.getSample(Timer.getFPGATimestamp()).get();
    }

}
