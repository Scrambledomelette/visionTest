// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.notezart.Constants.ButtonMap;
import frc.robot.notezart.DefaultSwerveCommand;
import frc.robot.notezart.GyroResetCommand;
import frc.robot.notezart.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final CommandJoystick driverJoystick = new CommandJoystick(ButtonMap.DRIVER_JOYSTICK_ID);
  private final AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
  // private final EstimatorTest estimatorTest = new EstimatorTest();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DefaultSwerveCommand(
    swerveSubsystem,
    () -> driverJoystick.getRawAxis(ButtonMap.JOYSTICK_Y_AXIS),
    () -> driverJoystick.getRawAxis(ButtonMap.JOYSTICK_X_AXIS),
    () -> driverJoystick.getRawAxis(ButtonMap.JOYSTICK_Z_AXIS)
    ));
    



    configureBindings();
  }

  private void configureBindings() {
    driverJoystick.button(ButtonMap.RESET_GYRO_BUTTON).onTrue(new GyroResetCommand(swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
