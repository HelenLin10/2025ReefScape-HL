package frc.robot.commands.Auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAlignLeft;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class LeftToReef extends SequentialCommandGroup {
  /** Autonomous sequence to drive to the Reef and perform actions */
  public LeftToReef(SwerveSubsystem drivebase, Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
        // Start Auto Routine
        Commands.runOnce(() -> SmartDashboard.putString("Auto Step", "Starting LeftToReef")),

        // Drive to Reef
        Commands.runOnce(() -> SmartDashboard.putString("Auto Step", "Driving to Reef")),
        new PathPlannerAuto("LeftAuto").withTimeout(4.0),

        // Wait before AutoAlign
        Commands.runOnce(() -> SmartDashboard.putString("Auto Step", "Waiting before AutoAlign")),
        new WaitCommand(2),

        // Auto Align
        Commands.runOnce(() -> SmartDashboard.putString("Auto Step", "Aligning to AprilTag")),
        new AutoAlignLeft(drivebase),

        // Wait before wrist movement
        Commands.runOnce(() -> SmartDashboard.putString("Auto Step", "Waiting before Wrist Movement")),
        new WaitCommand(2),

        // Move wrist to position
        Commands.runOnce(() -> {
          SmartDashboard.putString("Auto Step", "Moving Wrist Down");
          wrist.rotateDown(1);
        }, wrist),

        // Wait before elevator movement
        Commands.runOnce(() -> SmartDashboard.putString("Auto Step", "Waiting before Elevator Move")),
        new WaitCommand(2),

        // Raise Elevator
        Commands.runOnce(() -> {
          SmartDashboard.putString("Auto Step", "Raising Elevator");
          elevator.setHeight(1.0);
        }, elevator),

        // Wait before intake activation
        Commands.runOnce(() -> SmartDashboard.putString("Auto Step", "Waiting before Intake")),
        new WaitCommand(2),

        // Activate Intake
        Commands.runOnce(() -> {
          SmartDashboard.putString("Auto Step", "Activating Intake");
          intake.BallIn_TubeOut(1);
        }, intake).withTimeout(2),

        // Auto Complete
        Commands.runOnce(() -> SmartDashboard.putString("Auto Step", "Auto Complete"))
    );
  }

  private LeftToReef() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
