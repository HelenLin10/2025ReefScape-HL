package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

/*
 * Holds the wrist at the point it has been moved to by the move commands 
 */
public class HoldWristPosition extends Command {
    private final Wrist wrist;

    public HoldWristPosition(Wrist wrist) {
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.stop(); // Save current position as setpoint
    }

    @Override
    public void execute() {
        wrist.holdPosition(); // Maintain position
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}