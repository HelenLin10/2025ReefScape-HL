package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class RotateWristDown extends Command {
    private final Wrist wrist;

    public RotateWristDown(Wrist wrist) {
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        wrist.rotateDown();
    }

    @Override
    public boolean isFinished() {
        return wrist.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            wrist.stop();
        }
    }
}