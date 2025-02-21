package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualMoveElevator extends Command {
    private final Elevator elevator;
    private final String action;

    public ManualMoveElevator(Elevator elevator, String action){
        this.elevator = elevator;
        this.action = action;

        addRequirements(elevator);
    }
    @Override
    public void initialize() {
        switch (action) {
            case "UP":
                elevator.moveUp();
                break;
            case "DOWN":
                elevator.moveDown();
                break;
            default:
                throw new IllegalArgumentException("Invalid action: " + action);
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
