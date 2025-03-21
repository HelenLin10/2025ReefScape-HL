package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetGyro extends Command{
    private final SwerveSubsystem swerveDrive;

    public ResetGyro(SwerveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        swerveDrive.zeroGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
