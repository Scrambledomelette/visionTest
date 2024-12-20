package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    /* must be the same as in photonvision. Note: names are reset if you reset a photonvision coprocessor */
    private final String[] cameraNames = {"frontLeftCam", "frontRightCam", "backLeftCam", "backRightCam"};

    /** The Physical positions of the cameras on the robot, relative to the center.
     * x/y/z is in meters, and Roll/Pitch/Yaw are measured in radians.
     * Positive x Values are forwards of the center,
     * Positive y Values are left of the center,
     * Positive z Values are higher from the floor
     * Roll: The camera's rotation axis of itsef. (ex: Math.PI is upside-down)
     * Pitch: The camera's tilt relative to the floor. (ideal is 25 degrees)
     * Yaw: the rotation axis of the robot. positive is COUNTERclockwise from the front. 
    */
    private final Transform3d[] cameraPositions = {
        new Transform3d(0.35, 0.35, 0.26, new Rotation3d(0,Math.toRadians(30),Math.toRadians(45))),
        new Transform3d(0.35, -0.35, 0.26, new Rotation3d(0, Math.toRadians(30), Math.toRadians(320))),
        new Transform3d(-0.35, 0.35, 0.26, new Rotation3d(0, Math.toRadians(30), Math.toRadians(135))),
        new Transform3d(-0.35, -0.35, 0.26, new Rotation3d(0, Math.toRadians(30), Math.toRadians(225)))        
    };

    /* you could turn this into a list if you wanted to have different strategies for each camera, 
    * but this specific strategy is the best strategy for almost every case in FRC. */
    private final PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    /* The list of all the cameras */
    private AprilCam[] aprilCams = new AprilCam[4];

    private Field2d frontLeftTestField;
    private Field2d frontRightTestField;

    public AprilTagSubsystem() {
        /* declare the camera list */
        for (int i = 0; i < 4; i++) {
            aprilCams[i] = new AprilCam(
                cameraNames[i], 
                cameraPositions[i], 
                poseStrategy
            );
        }

        frontLeftTestField = new Field2d();
        frontRightTestField = new Field2d();
    }

    @Override
    public void periodic() {
        
        // SmartDashboard.putBoolean("has front left targets", isDetectingTargets(0));
        // SmartDashboard.putBoolean("has front right targets", isDetectingTargets(1));
        // SmartDashboard.putBoolean("has back left targets", isDetectingTargets(2));
        // SmartDashboard.putBoolean("has back right targets", isDetectingTargets(3));
        frontLeftTestField.setRobotPose(aprilCams[0].getEstimatedRobotPose().toPose2d());
        frontRightTestField.setRobotPose(aprilCams[1].getEstimatedRobotPose().toPose2d());
        SmartDashboard.putData("front left field", frontLeftTestField);
        SmartDashboard.putData("front right field", frontRightTestField);
        SmartDashboard.putNumber("front left distance", aprilCams[0].getDistanceToTagFloor());
        super.periodic();
    }


    /**
     * compares the estimated pose of the camera to the inputted pose.
     * @param camID The camera chosen
     * @param compareToPose The Pose to compare to (likely the origin pose) 
     * @return the difference between the two poses.
     */
    public Twist2d getRawPoseError(Integer camID, Pose2d compareToPose) {
        return compareToPose.log(aprilCams[camID].getEstimatedRobotPose().toPose2d());
    }

    /**
     * Multiplies the difference of two poses by the 'trust' matrix.
     * @param trustMatrix the Matrix to multiply the Pose by. Any items in the matrix should always be between 0 and 1.
     * @param RawError the Difference between to poses being multiplied
     * @return the difference between the two poses, with accounted trust.
     */
    public Twist2d getTrustPoseError(Matrix<N3,N3> trustMatrix, Twist2d RawError) {
        Matrix<N3,N1> matrix = trustMatrix.times(VecBuilder.fill(RawError.dx,RawError.dy,RawError.dtheta));
        return new Twist2d(matrix.get(0, 0),matrix.get(1, 0), matrix.get(2, 0));
    }

    /**
     * calculates the trust level matrix based on the distance, robot speed, and number of tags present.
     * @param camID The ID of the camera to create a matrix for
     * @param robotVelocity the speed of the robot in meters per second.
     * @return the camera's trust matrix
     */
    public Matrix<N3,N3> protoCameraMatrix(Integer camID, double robotVelocity) {
        double distance = aprilCams[camID].getDistanceToTagFloor();
        double tagCount = aprilCams[camID].getNumberOfTargets();

        Matrix<N3,N3> importanceMatrix = new Matrix<>(Nat.N3(), Nat.N3());

        double tagTrust = (tagCount - 1) * 0.1;

        double speedImportance = getProtoSpeedImportance(robotVelocity, distance, tagTrust);
        double speedTrust = getProtoTrust(distance, 1, 4) * speedImportance;

        double distanceImportance = 1 - (speedImportance + (tagTrust * (1/3)));
        double distanceTrust = getProtoTrust(robotVelocity, 3, 4) * distanceImportance;

        double cameraTrust = distanceTrust + speedTrust + tagTrust;
        for (int row = 0; row < 3; row++) {
            importanceMatrix.set(row, row, cameraTrust);
        }

        return importanceMatrix;
    }

    public double getProtoTrust(double value, double lowBound, double highBound) {
        if (value <= lowBound) {
            return 1;
        } else if (value >= highBound) {
            return 0;
        } else {
            return (1/Math.sqrt(value));
        }
    }

    private double getProtoSpeedImportance(double robotSpeed, double distance, double tagTrust) {
        double importance = 0.3;

        if (robotSpeed <= 2) {
            importance = importance - 0.1;
        } else if (robotSpeed >= 4) {
            importance = importance + 0.1;
        }

        if (distance <= 3) {
            importance = importance - 0.1;
        } else if (distance >= 6) {
            importance = importance + 0.1;
        }

        double finalImportance = importance - (tagTrust * (2/3));
        if (finalImportance <= 0.1) {
            finalImportance = 0.1;
        }

        return finalImportance;
    }


    /**
     * @return if there are tags detected by any camera at all.
     */
    public boolean hasAnyTargets() {
        boolean anyTargetDetected = false;

        /* checks through each camera to see if any have targets. */
        for (AprilCam anyCam:aprilCams) {
            if (anyCam.hasTargets()) {
                anyTargetDetected = true;
            }
        }

        return anyTargetDetected;
    }

    /**
     * just a passthrough for if the cam has targets
     * @param camID the camera to check
     * @return if the checked camera has any targets
     */
    public boolean isDetectingTargets(Integer camID) {
        return aprilCams[camID].hasTargets();
    }

    /**
     * @return the number of cameras with a target
     */
    public Integer getNumberOfDetectingCameras() {
        Integer activeCamNumber = 0;
        for (AprilCam eachCam:aprilCams) {
            if (eachCam.hasTargets()) {
                activeCamNumber++;
            }
        }
        return activeCamNumber;
    }

    /**
     * only exists to make it easier to remove a cameras from the system in case one breaks
     * @return the number of cameras
     */
    public Integer getNumberOfActiveCameras() {
        return aprilCams.length;
    }

 //#region Stuff I moved into aprilometry
//    /** Step Three:
//      * obtains the trusted predicted pose from the camera to the originPose.
//      * @param camID The camera chosen 
//      * @param originPose The origin pose. Should be the current Pose of the robot, the trusted odometry pose, or just an odometry pose.
//      * @return The Overall predicted Pose of the camera.
//      */
//     private Pose2d getFilteredPositions(Integer camID, Pose2d originPose) {
//         Matrix<N3,N3> trustMatrix = new Matrix<>(Nat.N3(),Nat.N3());

//         for (int Row = 0; Row < 3; Row++) {
//             /* yes, placing the values diagonally in the matrix is intentional. */
//             trustMatrix.set(
//                 Row, 
//                 Row, 
//                 aprilCams[camID].getBaseMatrix().get(Row, 0)
//             );
//         }

//         return originPose.exp(getTrustPoseError(trustMatrix, getRawPoseError(camID, originPose)));
//     }   
    
//     /** Step Four:
//      * Compiles each camera's predicted pose into a list.
//      * @param originPose The origin pose. See step three for more detailed description.
//      * @return
//      */
//     private List<Pose2d> getFilteredPoseList(Pose2d originPose) {
//         List<Pose2d> poseList = new ArrayList<>();
        
//         for (int cameraID = 0; cameraID < 4; cameraID++) {
//             Pose2d poseToAdd = getFilteredPositions(cameraID, originPose);
//             if (aprilCams[cameraID].hasTargets() && (!poseToAdd.getTranslation().equals(originPose.getTranslation()))) {
//                 poseList.add(poseToAdd);
//             }
//         }

//         return poseList;
//     }

//     /** Final Step.
//      * Averages the result of each active Camera's position to obtain a single comparitive vision pose.
//      * @param poseList A list of each Camera pose AFTER trust interpretation.
//      * @return the assumed position based on Every position.
//      */
//     private Pose2d getVisionAverage(Pose2d originPose) {
//         List<Pose2d> poseList = getFilteredPoseList(originPose);
//         double interPoseX = 0;
//         double interPoseY = 0;

//         for (int poseIndex = 0; poseIndex < poseList.size(); poseIndex++) {
//             interPoseX = (interPoseX + poseList.get(poseIndex).getX());
//             interPoseY = (interPoseY + poseList.get(poseIndex).getY());
//         }

//         interPoseX = interPoseX / poseList.size();
//         interPoseY = interPoseY / poseList.size();

//         return new Pose2d(interPoseX, interPoseY, new Rotation2d(0));
//     }
 //#endregion
}
