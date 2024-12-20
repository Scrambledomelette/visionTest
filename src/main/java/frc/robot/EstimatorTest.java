package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EstimatorTest extends SubsystemBase {
    VisionEstimator testVisionEstimator;
    private double xTestValue1 = 0;
    private double yTestValue1 = 0;
    private double zTestValue1 = 0;
    private double xTestValue2 = 0;
    private double yTestValue2 = 0;
    private double zTestValue2 = 0;
    private double originPoseX = 0;
    private double originPoseY = 0;
    private Pose2d testPose2d;
    private Pose2d testPose2d2;
    private Pose2d overallPose;

    private Field2d testOneField;
    private Field2d testTwoField;
    private Field2d overallField;
 
    public EstimatorTest() { 
        testVisionEstimator = new VisionEstimator();
        testPose2d = new Pose2d();
        testPose2d2 = new Pose2d();
        overallPose = new Pose2d();

        SmartDashboard.putNumber("first x value", xTestValue1);
        SmartDashboard.putNumber("second x value", xTestValue2);
        SmartDashboard.putNumber("first y value", yTestValue1);
        SmartDashboard.putNumber("second y value", yTestValue2);
        SmartDashboard.putNumber("first z value", zTestValue1);
        SmartDashboard.putNumber("second z value", zTestValue2);
        SmartDashboard.putNumber("originPose X", originPoseX);
        SmartDashboard.putNumber("originPose Y", originPoseY);

        testOneField = new Field2d();
        testTwoField = new Field2d();
        overallField = new Field2d();


        testOneField.setRobotPose(testPose2d);
        testTwoField.setRobotPose(testPose2d2);
        overallField.setRobotPose(new Pose2d(0,0,new Rotation2d(0)));
    }

    @Override
    public void periodic() {
        testPose2d = new Pose2d(
            SmartDashboard.getNumber("first x value", xTestValue1),
            SmartDashboard.getNumber("first y value", yTestValue1), 
            new Rotation2d(SmartDashboard.getNumber("first z value", zTestValue1))
        );

        testPose2d2 = new Pose2d(
            SmartDashboard.getNumber("second x value", xTestValue2),
            SmartDashboard.getNumber("second y value", yTestValue2), 
            new Rotation2d(SmartDashboard.getNumber("second z value", zTestValue2))
        );

        testVisionEstimator.setOriginPose(new Pose2d(
            SmartDashboard.getNumber("originPose X", originPoseX), 
            SmartDashboard.getNumber("originPose Y", originPoseY), 
            new Rotation2d(0)
        ));
        overallPose = testVisionEstimator.testFilter(testPose2d, testPose2d2);

        testOneField.setRobotPose(testPose2d);
        testTwoField.setRobotPose(testPose2d2);
        overallField.setRobotPose(overallPose);

        SmartDashboard.putData("data one field", testOneField);
        SmartDashboard.putData("data two field", testTwoField);
        SmartDashboard.putData("overall field", overallField);
        
        super.periodic();
    }
    
}
