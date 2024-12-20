package frc.robot.notezart;

import edu.wpi.first.math.geometry.Pose2d;

public interface BotPoseSource {
    /**
     * @return gets the BotPoseSource location of the robot.
     */
    public Pose2d getPose();

    /**
    * sets the BotPoseSource location of the robot.
    * @param newPose the pose to set the robot to
    */
    public void setPose(Pose2d newPose);
    
}