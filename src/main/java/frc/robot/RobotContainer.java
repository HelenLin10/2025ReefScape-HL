package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignTest2;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.Auto.LeftToReef;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  public final SwerveSubsystem drivebase = new SwerveSubsystem();
  public final static CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public final static CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  //Subsystem Instances
  private final Elevator elevator = new Elevator();
  private final Wrist wrist = new Wrist();
  private final Intake intake = new Intake();
  
  //Getter methods to fetch controller values
  public static double getLeftYValue(){ 
    return  m_operatorController.getLeftY();
  
  }
  public static double getRightYValue(){ 
    return m_operatorController.getRightY();
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
  
  
  public RobotContainer() {
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("LeftToReef", new LeftToReef(drivebase, elevator, wrist, intake));
    autoChooser.addOption("ForwardAuto", new PathPlannerAuto("ForwardAuto").withTimeout(4.0));
    SmartDashboard.putData("Auto Choices", autoChooser);
    
    configureButtonBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

 
//Drive using Angular Velocity
  private SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    ()-> m_driverController.getLeftY() * -1,
    ()-> m_driverController.getLeftX() * -1)
    .withControllerRotationAxis(m_driverController::getRightX)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(1)//If want faster change to 1
    .allianceRelativeControl(false);

//Drive using Direct Angle
  private SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
    m_driverController::getRightX,
    m_driverController::getRightY)
    .headingWhile(true);

//Assign Drive Commands
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  

  private void configureButtonBindings() {
    m_driverController.x().onTrue(new AutoAlignTest2(drivebase));
    m_driverController.leftBumper().onTrue(new ResetGyro(drivebase));
    //ELEVATOR
    //Move elevator to 0 postition when D-Pad Down is pressed
    m_operatorController.povDown().onTrue(new ElevatorCommand(elevator, 0));
    
    // Move elevator to Level 1 when A is pressed
    m_operatorController.a().onTrue(new ElevatorCommand(elevator, 40)); 
    
    // Move elevator to Level 2 when X is pressed   
    m_operatorController.x().onTrue(new ElevatorCommand(elevator, 77.6));
    
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
