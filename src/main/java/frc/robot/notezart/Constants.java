// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.notezart;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {
    /** a list of relevant obstacle poses, with more index detail at the definition. */
    public static List<Pose2d> obstaclePositions = List.of(
        new Pose2d(0.0, 5.55, new Rotation2d(0)), /* blue speaker pose */
        new Pose2d(16.2, 5.55, new Rotation2d(0)), /* red speaker pose */
        new Pose2d( 4.84, 4.11, new Rotation2d(0)), /* blue stage center */
        new Pose2d(11.72, 4.11, new Rotation2d(0)) /* red stage center */
    );

    public static final double MAX_BUS_VOLTAGE = 12;
    public static final int LED_CAN_ID = 60;

    public final class ShooterConstants {
        /** the minimum time needed to shoot */
        public static final double SHOOT_TIME = 1.5; 
        /** the angle to shoot into the speaker */
        public static final double MANUAL_SPEAKER_ANGLE = 56;
        public static final double AMP_ANGULAR_ANGLE = 117;

        public static final double DEFAULT_AMP_TOP_SHOOTER_POWER = .315;
        public static final double DEFAULT_AMP_BOTTOM_SHOOTER_POWER = -0.05;
    }

    /** a centralized location for all button values on the driver station. */
    public final class ButtonMap {
        /* basic joystick IDs */
        public static final Integer DRIVER_JOYSTICK_ID = 0;
        public static final Integer MANIPULATOR_PANEL_ID = 1;
        public static final Integer JOYSTICK_X_AXIS = 0;
        public static final Integer JOYSTICK_Y_AXIS = 1;
        public static final Integer JOYSTICK_Z_AXIS = 2;

        /* driver buttons*/
        public static final Integer INTAKE_BUTTON = 1;
        public static final Integer FORCE_INTAKE_OUT_BUTTON = 5;
        public static final Integer FORCE_INTAKE_IN_BUTTON = 10;
        public static final Integer CHASSIS_SPEAKER_AIM_BUTTON = 2;
        public static final Integer CHASSIS_NOTE_AIM_BUTTON = 4;
        public static final Integer CHASSIS_AMP_AIM_BUTTON = 3;
        public static final Integer RESET_GYRO_BUTTON = 11;
        public static final Integer RESET_ODOMETRY_BUTTON = 12;

        /* angler mode buttons */
        public static final Integer AUTO_ANGLE_MODE = 2;
        public static final Integer SUBWOOFER_MODE = 6;
        public static final Integer AMP_MODE = 4;
        public static final Integer PASSING_MODE = 5;
        public static final Integer IDLE_MODE = 12;

        /* manual shooter control buttons */
        public static final Integer FORCE_SHOOT_BUTTON = 14;
        public static final Integer FORCE_SHOOTER_OUT_BUTTON = 10;
        public static final Integer FORCE_SHOOTER_IN_BUTTON = 11;
        public static final Integer ANGULAR_UP_BUTTON = 1;
        public static final Integer ANGULAR_DOWN_BUTTON = 3;
        public static final Integer FORCE_COMPLIANT_WHEELS_IN_BUTTON = 17;
        public static final Integer FORCE_COMPLIANT_WHEELS_OUT_BUTTON = 19;

        /* other manipulator buttons */
        public static final Integer SHOOT_BUTTON = 8;
        public static final Integer COMPLIANT_WHEELS_BUTTON = 15;
        public static final Integer NO_SOURCE_LIGHTS = 9;
        public static final Integer CLIMBER_UP_BUTTON = 16;
        public static final Integer CLIMBER_DOWN_BUTTON = 18;
    }
}
