package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.RotateWristDown;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
    
    //Controllers
    private final static CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
        private final static CommandXboxController m_operatorController = 
        new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    
    private final Elevator elevator = new Elevator();
    private final Wrist wrist = new Wrist();
  
    public static double getYValue(){ 
      return m_operatorController.getLeftY();
    }
    public static boolean rightBumperPressed(){
      return m_operatorController.rightBumper().getAsBoolean();
    }
    public static boolean leftBumperPressed(){
      return m_operatorController.leftBumper().getAsBoolean();
    }
    public static double rightTriggerValue(){
      return m_operatorController.getRightTriggerAxis();
    }
    public static double leftTriggerValue(){
      return m_operatorController.getLeftTriggerAxis();
    }
  
  
  
    
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    NamedCommands.registerCommand("Left", new RotateWristDown(wrist));
    autoChooser = AutoBuilder.buildAutoChooser("autoChooser");
    autoChooser.setDefaultOption("Left", new RotateWristDown(wrist));
    SmartDashboard.putData("Auto Choices", autoChooser);
    
    configureButtonBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

 

private SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
  drivebase.getSwerveDrive(),
  ()-> m_driverController.getLeftY() * 1,
  ()-> m_driverController.getLeftX() * 1)
  .withControllerRotationAxis(m_driverController::getRightX)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)//If want faster change to 1
  .allianceRelativeControl(true);

private SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
  m_driverController::getRightX,
  m_driverController::getRightY)
  .headingWhile(true);

Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  
private void configureButtonBindings() {
     //ELEVATOR

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
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
