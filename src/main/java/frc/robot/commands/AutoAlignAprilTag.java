package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignAprilTag extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private Pose2d targetPose;
    private int closestTagId = -1;
    private boolean validTagFound = false; // Tracks if a valid AprilTag was found at the start

    private final PIDController xController = new PIDController(0.5, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.5, 0.0, 0.0);
    private final PIDController rotController = new PIDController(0.5, 0.0, 0.0);

    private static final double PositionTolerance = 0.2; // 20 cm
    private static final double RotationTolerance = Math.toRadians(3); // 3 degrees

    private static final double SAFE_STOP_DISTANCE = 0.5; // Stop 50 cm before the AprilTag
    private static final double LIMELIGHT_X_OFFSET = 0.15; // Limelight is 15 cm right of the center
    private static final double TARGET_OFFSET = -0.3; // Stop 30 cm to the left of the tag

    public AutoAlignAprilTag(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        validTagFound = updateClosestAprilTag(); // ✅ If no tag is found, validTagFound will be false

        // Debugging: Display status on SmartDashboard
        SmartDashboard.putBoolean("Valid AprilTag Found", validTagFound);
    }

    @Override
    public void execute() {
        if (!validTagFound) {
            return; // 🚨 Exit immediately if no AprilTag was found in `initialize()`
        }

        Pose2d currentPose = swerveSubsystem.getVisionPose();

        // ✅ Calculate errors between current position and target
        Translation2d translationError = targetPose.getTranslation().minus(currentPose.getTranslation());
        double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

        // 🔹 Adjust Stopping Position (To Prevent Crashing into Walls)
        double stopX = targetPose.getX() - SAFE_STOP_DISTANCE;
        double offsetX = TARGET_OFFSET - LIMELIGHT_X_OFFSET; // Offset the stopping position left/right
        double offsetY = 0.0; // Adjust this if lateral offset is needed

        // ✅ Use stopX for stopping distance
        double xSpeed = xController.calculate(currentPose.getX(), stopX + offsetX);
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY() + offsetY);
        double rotSpeed = rotController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // ✅ Limit speeds for smoother motion
        xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), 2.0), xSpeed);
        ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), 2.0), ySpeed);
        rotSpeed = Math.copySign(Math.min(Math.abs(rotSpeed), 1.5), rotSpeed);

        // ✅ Move robot to the adjusted stopping position
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(ySpeed, -xSpeed, rotSpeed));

        // 🔹 Display debugging values on SmartDashboard
        SmartDashboard.putNumber("Translation Error", translationError.getNorm());
        SmartDashboard.putNumber("Rotation Error (degrees)", Math.toDegrees(rotationError));
        SmartDashboard.putNumber("Target Stop X", stopX);
    }

    @Override
    public boolean isFinished() {
        if (!validTagFound) return true; // ✅ If no tag was found, end the command immediately

        Pose2d currentPose = swerveSubsystem.getVisionPose();
        Translation2d translationError = targetPose.getTranslation().minus(currentPose.getTranslation());
        double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

        // 🔹 Stop when the robot is at the Safe Distance from the AprilTag
        return Math.abs(translationError.getX() - SAFE_STOP_DISTANCE) < PositionTolerance &&
               Math.abs(rotationError) < RotationTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }

    // 🔹 Updates the closest AprilTag and returns true if a valid tag was found
    private boolean updateClosestAprilTag() {
        closestTagId = getClosestAprilTag();
        if (closestTagId != -1) {
            swerveSubsystem.setVisionTargetID(closestTagId);
            targetPose = swerveSubsystem.getVisionPose();
            return true; // ✅ Found a valid AprilTag
        } else {
            targetPose = null; // No valid AprilTag found
            return false; // ❌ No tag detected
        }
    }

    // 🔹 Finds the closest AprilTag dynamically
    private int getClosestAprilTag() {
        LimelightResults limelightData = LimelightHelpers.getLatestResults("limelight");
        if (limelightData == null || limelightData.targets_Fiducials.length == 0) {
            return -1; // No tags detected
        }

        int closestTagID = -1;
        double closestDistance = Double.MAX_VALUE;

        for (LimelightTarget_Fiducial target : limelightData.targets_Fiducials) {
            double distance = 1.0 / target.ta; // ✅ Use inverse of target area for distance estim
            int tagID = (int) target.fiducialID;
            if (distance < closestDistance) {
                closestDistance = distance;
                closestTagID = tagID;
            }
        }
        return closestTagID;
    }
}
