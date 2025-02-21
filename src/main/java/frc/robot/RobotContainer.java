// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.commands.LeftToReef;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualMoveElevator;
import frc.robot.commands.MoveWristCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
      private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
      private final Elevator elevator = new Elevator();
      private final Intake intake = new Intake();
      private final Wrist wrist = new Wrist();
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger binding
    
    autoChooser.setDefaultOption("Reef Auto", new LeftToReef(drivebase, elevator, wrist, intake));
    SmartDashboard.putData("Auto Choices", autoChooser);
    
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    configureButtonBindings();
  }

SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
  drivebase.getSwerveDrive(),
  ()-> m_driverController.getLeftY() * -1,
  ()-> m_driverController.getLeftX() * -1)
  .withControllerRotationAxis(m_driverController::getRightX)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)//If want faster change to 1
  .allianceRelativeControl(true);

SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
  m_driverController::getRightX,
  m_driverController::getRightY)
  .headingWhile(true);

Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  private void configureButtonBindings() {
  
    //ELEVATOR

    m_operatorController.rightBumper().onTrue(new ManualMoveElevator(elevator, "moveUp"));
    m_operatorController.leftBumper().onTrue(new ManualMoveElevator(elevator, "moveDown"));

    //Move elevator to 0 postition when D-Pad Down is pressed
    m_operatorController.povDown().onTrue(new ElevatorCommand(elevator, 0));
    
    // Move elevator to Level 1 when A is pressed
    m_operatorController.a().onTrue(new ElevatorCommand(elevator, 1)); 
    
    // Move elevator to Level 2 when X is pressed   
    m_operatorController.x().onTrue(new ElevatorCommand(elevator, 2));
    
    // Move elevator to Level 3 when B is pressed
    m_operatorController.b().onTrue(new ElevatorCommand(elevator, 3));
    
    // Move elevator to Level 4 when Y is pressed
    m_operatorController.y().onTrue(new ElevatorCommand(elevator, 4));
    
    // INTAKE

    // BallIn_TubeIn on RT press
    m_operatorController.rightTrigger().onTrue(new IntakeCommand( intake, "BallIn_TubeOut", m_operatorController.getRightTriggerAxis()));
    
    // BallOut_TubeOut on LT press
    m_operatorController.leftTrigger().onTrue(new IntakeCommand(intake, "BallOut_TubeIn", m_operatorController.getLeftTriggerAxis()));

    // WRIST

    // Up on Right D-Pad
    m_operatorController.povRight().onTrue(new MoveWristCommand(wrist, "UP"));

    // Down on Left D-Pad
    m_operatorController.povLeft().onTrue(new MoveWristCommand(wrist, "DOWN"));
  
    
  
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
