package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

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
    }

    @Override
    public void execute() {
        // Check if Limelight sees an AprilTag
        boolean hasVisionTarget = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").tagCount > 0;
         // Display AutoAlign status
        SmartDashboard.putBoolean("AutoAlign Running", true);
        SmartDashboard.putBoolean("Has Vision Target", hasVisionTarget);

        if (!hasVisionTarget) {
            System.out.println("No AprilTag Found! Stopping AutoAlign.");
            swerveSubsystem.stop();
            cancel();
            return; // Exit execution until command is restarted
        }

        // Cancel AutoAlign if driver moves manually
        if (swerveSubsystem.isManualControlActive()) {
            System.out.println("Manual control detected! Cancelling AutoAlign.");
            cancel(); // End the command
            return;
        }

        // Get current alliance (default to Blue if unknown)
    boolean isRedAlliance = DriverStation.getAlliance().isPresent() &&
                            DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        // Get Limelight data
        double tx = LimelightHelpers.getTX("limelight"); // Rotation alignment
        double ty = LimelightHelpers.getTY("limelight"); // Forward/backward distance
        double robotXOffset = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose.getX(); // Left/right position

        
        // Flip the offset if on Red Alliance
    double xOffset = isRedAlliance ? -(robotXOffset - LEFT_OFFSET) : (robotXOffset - LEFT_OFFSET);

        // Calculate PID outputs
        double rotationSpeed = rotationPID.calculate(tx, 0);  // Rotate towards AprilTag
        double forwardSpeed = (ty > MIN_DISTANCE_FROM_TAG) ? forwardPID.calculate(ty, 0) : 0; // Move only if safe
        double strafeSpeed = strafePID.calculate(xOffset, 0); // Strafe to align left of tag

        // Send movement commands to swerve drive
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));

        SmartDashboard.putNumber("AutoAlign Rotation Speed", rotationSpeed);
        SmartDashboard.putNumber("AutoAlign Forward Speed", forwardSpeed);
        SmartDashboard.putNumber("AutoAlign Strafe Speed", strafeSpeed);
    }

    @Override
    public boolean isFinished() {
        // Stop command when all alignments are achieved
        return rotationPID.atSetpoint() && forwardPID.atSetpoint() && strafePID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

         // Gradually slow down to prevent a sudden stop
        for (int i = 0; i < 5; i++) {
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0.2 / (i + 1), 0.2 / (i + 1), 0.1 / (i + 1)));
        }
        swerveSubsystem.stop();
        System.out.println("AutoAlign Command Ended");
    }
}
