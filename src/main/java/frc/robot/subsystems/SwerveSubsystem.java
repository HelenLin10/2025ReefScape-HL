// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class SwerveSubsystem extends SubsystemBase {
    private final Pigeon2 pigeon2;
    private final File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    private final SwerveDrive swerveDrive;
    private final PIDController rotationPID;

    /** Constructor */
    public SwerveSubsystem() {
        pigeon2 = new Pigeon2(Constants.pigeon2ID);
        pigeon2.setYaw(0);

        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Initialize Swerve Drive
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                Constants.maxSpeed,
                new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Setup PathPlanner and PID controller for rotation alignment
        setupPathPlanner();
        rotationPID = new PIDController(0.02, 0.0, 0.002);
        rotationPID.setTolerance(1.0); // Stop adjusting when within 1 degree of target
    }

    /** Odometry & Pose Methods */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialPose) {
        swerveDrive.resetOdometry(initialPose);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public double getGyroAngle() {
        return pigeon2.getYaw().getValueAsDouble();
    }

    
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        }
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
    
    /** Drive Methods */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
    }

    public void stop() {
        swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }

    /** Manual Control Detection */
    public boolean isManualControlActive() {
        double deadband = 0.4;
        return Math.abs(RobotContainer.m_driverController.getLeftX()) > deadband ||
               Math.abs(RobotContainer.m_driverController.getLeftY()) > deadband ||
               Math.abs(RobotContainer.m_driverController.getRightX()) > deadband;
    }

    /** Vision Odometry Integration */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        swerveDrive.addVisionMeasurement(visionPose, timestamp);
    }

    public void updateVisionOdometry() {
        var limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (limelightMeasurement.tagCount >= 2) {
            Pose2d visionPose = limelightMeasurement.pose;

            // Flip vision data if Red Alliance
            if (isRedAlliance()) {
                visionPose = new Pose2d(
                    -visionPose.getX(),
                    -visionPose.getY(),
                    visionPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
                );
            }

            // Apply vision update
            addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
        }
    }

     /** Moves the robot using robot-relative speeds (for AutoAlign). */
     public void setRobotRelativeSpeeds(double forwardSpeed, double strafeSpeed, double rotationSpeed) {
        swerveDrive.setChassisSpeeds(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));
    }

    /** PathPlanner Setup */
    public void setupPathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            final boolean enableFeedforward = true;

            AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (enableFeedforward) {
                        swerveDrive.drive(
                            speedsRobotRelative,
                            swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                            moduleFeedForwards.linearForces()
                        );
                    } else {
                        swerveDrive.setChassisSpeeds(speedsRobotRelative);
                    }
                },
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                () -> isRedAlliance(),
                this
            );

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /** Periodic Updates */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Pose X", getPose().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
        SmartDashboard.putNumber("Robot Heading (Degrees)", getPose().getRotation().getDegrees());
        SmartDashboard.putBoolean("Manual Control Active?", isManualControlActive());
        SmartDashboard.putBoolean("Red Alliance?", isRedAlliance());
        updateVisionOdometry();
    }
}
