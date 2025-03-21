// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class SwerveSubsystem extends SubsystemBase {

  Pigeon2 pigeon2;
  LimelightHelpers limelight;
  LimelightHelpers limelightPoseEstimator;
  
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive;

  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem() {

    pigeon2 = new Pigeon2(Constants.pigeon2ID);
    pigeon2.setYaw(0);

    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed,
                    new Pose2d(new Translation2d(Meter.of(1),
                                                 Meter.of(4)),
                    Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    setupPathPlanner();
    }


public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }
}

public Command getAutonomousCommand(String pathName) {
  return new PathPlannerAuto(pathName);
}



  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds>velocity){
    return run(()->{swerveDrive.driveFieldOriented(velocity.get());});
  }

  //Get's robot's position & rotation on the field
  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public double GyroAngle(){
    return pigeon2.getYaw().getValueAsDouble();
  }

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }

  private boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance(){
    if(isRedAlliance()){
      zeroGyro();
  
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    }
  }
  
  //Get robot's field-relative yaw
  public double getHeading(){
    return swerveDrive.getOdometryHeading().getDegrees();
  }
    
  //Move and rotate the robot while keeping field-relative control
  public void drive(Translation2d translation2d, double rotation, boolean fieldOriented, boolean isOpenLoop){
    swerveDrive.drive(translation2d, rotation, fieldOriented, isOpenLoop);
  }

  public boolean isManualControlActive(){
    //Detct if driver is manually moving the swerve drive
    double deadband = 0.3;
    return (Math.abs(RobotContainer.m_driverController.getLeftY()) > deadband || 
    Math.abs(RobotContainer.m_driverController.getLeftX()) > deadband ||
    Math.abs(RobotContainer.m_driverController.getRightX()) > deadband);
  }

  public void stop(){
    swerveDrive.drive(new Translation2d(0,0), 0, false, true);
  }


  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    swerveDrive.addVisionMeasurement(visionPose, timestamp);
  }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Gyro Angle", pigeon2.getYaw().getValueAsDouble());
      this.swerveDrive.setGyro(new Rotation3d(new Rotation2d(pigeon2.getYaw().getValue())));
    }
  
  
  
}
