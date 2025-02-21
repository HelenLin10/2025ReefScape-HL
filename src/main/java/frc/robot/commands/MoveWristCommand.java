package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class MoveWristCommand extends Command {
    private final Wrist wrist;
    private final String action;

    public MoveWristCommand(Wrist wrist, String action) {
        this.wrist = wrist;
        this.action = action;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        switch (action) {
            case "UP":
                wrist.rotateUp();
                break;
            case "DOWN":
                wrist.rotateDown();
                break;
            default:
                throw new IllegalArgumentException("Invalid action: " + action);
        }
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}