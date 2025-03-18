package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class StowWrist extends SequentialCommandGroup {
    public StowWrist(Elevator elevator, Wrist wrist) {
        addCommands(
            Commands.runOnce(() -> elevator.setHeight(3)),
            new WaitCommand(2),
            Commands.runOnce(() -> elevator.setHeight(0.1)),
            new WaitCommand(0.5),
            Commands.runOnce(() -> wrist.setPosition(2)));
    }
}
