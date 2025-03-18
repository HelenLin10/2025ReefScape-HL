package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class AutoAlignLeft extends Command {
    private final SwerveSubsystem swerveSubsystem;

    // Thresholds for stopping movement
    private static final double ALIGNMENT_THRESHOLD = 1.0; // Degrees for rotation
    private static final double MOVE_THRESHOLD = 0.5; // Meters for forward/backward
    private static final double STRAFE_THRESHOLD = 0.3; // Meters for left/right strafing

    // Safe stopping distance (meters)
    private static final double MIN_DISTANCE_FROM_TAG = 0.5;

    // Offset to align slightly left of the AprilTag
    private static final double LEFT_OFFSET = 0.3; // 30cm left of tag

    // Vision tag lost timer
    private final Timer tagLostTimer = new Timer();
    private static final double TAG_LOST_TIMEOUT = 0.5; // Time before stopping when losing a tag

    // PID Controllers
    private final PIDController rotationPID = new PIDController(0.02, 0.0, 0.002);
    private final PIDController forwardPID = new PIDController(0.05, 0.0, 0.003);
    private final PIDController strafePID = new PIDController(0.05, 0.0, 0.003);

    public AutoAlignLeft(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        // Set tolerance for when the robot is "aligned enough"
        rotationPID.setTolerance(ALIGNMENT_THRESHOLD);
        forwardPID.setTolerance(MOVE_THRESHOLD);
        strafePID.setTolerance(STRAFE_THRESHOLD);
    }

    @Override
    public void initialize() {
        System.out.println("AutoAlign Command Started");
        tagLostTimer.reset();
        tagLostTimer.start();
    }

    @Override
    public void execute() {
        // Check if Limelight sees an AprilTag
        boolean hasVisionTarget = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").tagCount > 0;

        SmartDashboard.putBoolean("AutoAlign Running", true);
        SmartDashboard.putBoolean("Has Vision Target", hasVisionTarget);

        if (hasVisionTarget) {
            tagLostTimer.reset(); // Reset timer since we see the tag

            // ✅ Minimal Vision Odometry Integration
            Pose2d visionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose;

            // ✅ Update odometry with vision pose
            swerveSubsystem.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());

            // Cancel AutoAlign if driver moves manually
            if (swerveSubsystem.isManualControlActive()) {
                System.out.println("Manual control detected! Cancelling AutoAlign.");
                SmartDashboard.putString("AutoAlign Status", "Manual Override - Cancelled");
                end(true);
                return;
            }

            // Get alignment data
            double tx = LimelightHelpers.getTX("limelight"); // Rotation alignment
            double ty = LimelightHelpers.getTY("limelight"); // Forward/backward distance
            double robotXOffset = visionPose.getX();

            // Calculate PID outputs
            double rotationSpeed = rotationPID.calculate(tx, 0);
            double forwardSpeed = (ty > MIN_DISTANCE_FROM_TAG) ? forwardPID.calculate(ty, 0) : 0;
            double strafeSpeed = strafePID.calculate(robotXOffset - LEFT_OFFSET, 0);

            // Send movement commands to swerve drive
            swerveSubsystem.setRobotRelativeSpeeds(forwardSpeed, strafeSpeed, rotationSpeed);

            // ✅ SmartDashboard Debugging
            SmartDashboard.putNumber("Limelight TX (Rotation)", LimelightHelpers.getTX("limelight"));
            SmartDashboard.putNumber("Limelight TY (Distance)", LimelightHelpers.getTY("limelight"));
            SmartDashboard.putNumber("Vision Odometry X", visionPose.getX());
            SmartDashboard.putNumber("Vision Odometry Y", visionPose.getY());

        } else {
            // If we haven't seen the tag for more than the timeout, stop
            if (tagLostTimer.hasElapsed(TAG_LOST_TIMEOUT)) {
                System.out.println("No AprilTag Found for " + TAG_LOST_TIMEOUT + "s! Stopping AutoAlign.");
                SmartDashboard.putString("AutoAlign Status", "No AprilTag - Stopped");
                swerveSubsystem.stop();
                end(true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Stop command when all alignments are achieved
        return rotationPID.atSetpoint() && forwardPID.atSetpoint() && strafePID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoAlign Command Ended");
        SmartDashboard.putString("AutoAlign Status", "Finished");

        // Reset PID controllers
        rotationPID.reset();
        forwardPID.reset();
        strafePID.reset();


        swerveSubsystem.stop();
    }
}
