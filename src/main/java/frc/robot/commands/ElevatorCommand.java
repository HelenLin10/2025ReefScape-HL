package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final double targetPosition; // Desired elevator height

    public ElevatorCommand(Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator); // Reserves the subsystem
    }

    @Override 
    public void initialize() {
        elevator.setHeight(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Check if the elevator has reached its target position
        return elevator.isAtSetpoint();
    }
}