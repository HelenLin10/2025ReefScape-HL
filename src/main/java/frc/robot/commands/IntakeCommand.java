package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends InstantCommand {
    private final Intake intake;
    private final String action;
    private final double speed;

    public IntakeCommand(Intake intake, String action, double speed) {
        this.intake = intake;
        this.action = action;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        switch (action) {
            case "BallIn_TubeOut":
                intake.BallIn_TubeOut(speed);
                break;
            case "BallOut_TubeIn":
                intake.BallOut_TubeIn(speed);
                break;
            default:
                throw new IllegalArgumentException("Invalid action: " + action);
        }
    }
}
