package frc.robot.commands.Auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public final class LeftToReef extends SequentialCommandGroup {
  /** Autonomous sequence to drive to the Reef and perform actions */
  public LeftToReef(SwerveSubsystem drivebase, Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
        // Drive to Reef
        Commands.runOnce(() -> new PathPlannerAuto("LeftAuto").withTimeout(4.0)),

        // Wait before AutoAlign
        new WaitCommand(2),

        // Move wrist to position
        Commands.runOnce(() -> {
          wrist.rotateDown(1);
        }, wrist),

        // Wait before elevator movement
        new WaitCommand(2),

        // Raise Elevator
        Commands.runOnce(() -> {
          elevator.setHeight(1.0);
        }, elevator),

        // Wait before intake activation
        new WaitCommand(2),

        // Activate Intake
        Commands.runOnce(() -> {
          intake.BallIn_TubeOut(1);
        }, intake).withTimeout(2)

    );
  }

  private LeftToReef() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
