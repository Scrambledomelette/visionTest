package frc.robot.notezart;

import edu.wpi.first.wpilibj2.command.Command;

public class GyroResetCommand extends Command{
    private SwerveSubsystem swerveSubsystem;

    public GyroResetCommand(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }   
}
