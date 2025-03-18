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
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

import static edu.wpi.first.units.Units.Meter;
import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix6.hardware.Pigeon2;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSubsystem extends SubsystemBase {
  Pigeon2 pigeon2;
  private final Pose2d estimatedPose;
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  private final SwerveDrive  swerveDrive;

  /** Constructor */
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
    //Limelight
    estimatedPose = new Pose2d();
    }

  //Return current pose of the robot
  public Pose2d getEstimatedPose(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose2d){
    swerveDrive.resetOdometry(initialHolonomicPose2d);
  }
  
  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }

  public double GyroAngle(){
  return pigeon2.getYaw().getValueAsDouble();
  }



public SwerveDrive getSwerveDrive() {
    return swerveDrive;
}

public void driveFieldOriented(ChassisSpeeds velocity){
  swerveDrive.driveFieldOriented(velocity);
}

public Command driveFieldOriented(Supplier<ChassisSpeeds>velocity){
  return run(()->{
    swerveDrive.driveFieldOriented(velocity.get());
  });
}





public Pose2d getPose(){
  return swerveDrive.getPose();
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
          swerveDrive::getPose, // Robot pose supplier
          swerveDrive::resetOdometry, // Method to reset odometry(if have starting pose)
          swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
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
<<<<<<< HEAD
<<<<<<< Updated upstream
=======

public void driveAutoAlign(double forward, double rotation) {
  driveFieldOriented(new ChassisSpeeds(forward, 0, rotation));
}

public boolean isManualControlActive() {
    // Detect if driver is manually moving the swerve drive
    double deadband = 0.4; // Adjust as needed to prevent false detection
    return Math.abs(RobotContainer.m_driverController.getLeftX()) > deadband ||
           Math.abs(RobotContainer.m_driverController.getLeftY()) > deadband ||
           Math.abs(RobotContainer.m_driverController.getRightX()) > deadband;
}

public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    // Ensure your swerve drive has a method for adding vision measurements
    swerveDrive.addVisionMeasurement(visionPose, timestamp);
}


public void stop(){
  swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)); // Ensure all motors stop
}

  @Override
  public void periodic(){
=======




  public void stop() {
    swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  }


  //Auto-Align to Score Using AprilTag
  public void trackClosestAprilTag() {
    int closestTagID = getClosestAprilTag();
    if(closestTagID != -1){
      setVisionTargetID(closestTagID);
      SmartDashboard.putNumber("Tracking Tag ID", closestTagID);
    }
  }

  public int getClosestAprilTag(){
    LimelightResults limelightData = LimelightHelpers.getLatestResults("limelight");
    if (limelightData == null || limelightData.targets_Fiducials.length == 0){
      return -1;
    }

    int closestTagID = -1;
    double closestDistance = Double.MAX_VALUE;

    for (LimelightTarget_Fiducial target : limelightData.targets_Fiducials) {
        double distance = target.ty; // Use vertical offset as a distance estimate
        int tagID = (int) target.fiducialID;

        if (distance < closestDistance) {
            closestDistance = distance;
            closestTagID = tagID;
        }
    }
    return closestTagID;
  }
  
  //Set AprilTag as tracking target
  public void setVisionTargetID(int id){
    LimelightHelpers.setPriorityTagID("limelight", id);;
  }

  public boolean isValidVisionTarget(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").tagCount > 0;
  }

  public Pose2d getVisionPose(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose;
  }
  
  //Pathplanning adjust
  public void updateVisionOdometry(){
    var limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if(limelightMeasurement.tagCount >= 2)
    {
      swerveDrive.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("Has Vision Target", isValidVisionTarget());
    SmartDashboard.putNumber("Vision X", getVisionPose().getX());
    SmartDashboard.putNumber("Vision Y", getVisionPose().getY());
    updateVisionOdometry();
>>>>>>> bcceb330c2e1987e4c1a1351b865af09b005e43d
  }

  @Override
  public void simulationPeriodic(){
  }
<<<<<<< HEAD

>>>>>>> Stashed changes
=======
>>>>>>> bcceb330c2e1987e4c1a1351b865af09b005e43d
}
