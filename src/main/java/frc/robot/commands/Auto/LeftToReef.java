// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public final class LeftToReef extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */
  public LeftToReef(SwerveSubsystem drivebase, Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
      new PathPlannerAuto("LeftAuto").withTimeout(4.0), // Follow the path to Reef
      new WaitCommand(2),
      
      new AutoAlignLeft(drivebase),
      new WaitCommand(2),
      
      Commands.runOnce(() -> wrist.rotateDown(1), wrist), // Move wrist to shooting position
      new WaitCommand(2),

      Commands.runOnce(() -> elevator.setHeight(1.0), elevator), // Raise elevator to preset height
      new WaitCommand(2),

      Commands.run(() -> intake.BallIn_TubeOut(1), intake).withTimeout(2)); // Activate intake
  }

  private LeftToReef() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}