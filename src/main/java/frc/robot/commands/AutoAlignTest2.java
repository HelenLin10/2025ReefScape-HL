package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignTest2 extends Command{
    private final SwerveSubsystem swerveDrive;
    private double targetX, targetY, targetYaw;
    private final PIDController rotationPID;
    private final PIDController strafePID;
    private final PIDController forwardPID;

    // Vision tag lost timer
    private final Timer tagLostTimer = new Timer();
    private static final double TAG_LOST_TIMEOUT = 0.5; // Time before stopping when losing a tag
    private static final double MIN_DISTANCE_FROM_TAG = 0.5; // Stop moving forward within 0.5 meters

    double distanceCorrectionFactor = 0.1;

    public AutoAlignTest2(SwerveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;

        this.rotationPID = new PIDController(0.02, 0, 0);
        rotationPID.setTolerance(2);//Stop when within two degrees

        this.strafePID = new PIDController(0.02, 0, 0);
        strafePID.setTolerance(0.05);//Stops when within 5 cm

        this.forwardPID = new PIDController(0.02, 0, 0);
        forwardPID.setTolerance(0.3);//Stops when within 30 cm
        addRequirements(swerveDrive);
    }

@Override
    public void initialize() {
        tagLostTimer.reset();
        tagLostTimer.start();

        LimelightHelpers.PoseEstimate visionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (visionPose.tagCount == 0) {
            System.out.println("No vision target found");
            end(true);
            return;
        }
 
    // Get AprilTag pose data
    targetX = visionPose.pose.getX();
    targetY = visionPose.pose.getY();
    targetYaw = visionPose.pose.getRotation().getDegrees();

    double ty = LimelightHelpers.getTY("limelight");
    targetY += ty * distanceCorrectionFactor;
    swerveDrive.addVisionMeasurement(new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetYaw)), Timer.getFPGATimestamp());
    SmartDashboard.putString("AutoAlign Status", "Tracking AprilTag");
    }

    @Override
    public void execute() {
        if (swerveDrive.isManualControlActive()) {
            end(true);
            return;
        }
    
        LimelightHelpers.PoseEstimate visionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if(visionPose.tagCount == 0){
            if(tagLostTimer.hasElapsed(TAG_LOST_TIMEOUT)){
                SmartDashboard.putString("AutoAlign Status", "No AprilTag Found - Stopping");
                swerveDrive.stop();
                end(true);
            }
            return;
        } else {
            tagLostTimer.reset();
        }
        
        // Update AprilTag pose dynamically
         targetX = visionPose.pose.getX();
         targetY = visionPose.pose.getY();
         targetYaw = visionPose.pose.getRotation().getDegrees();

        double ty = LimelightHelpers.getTY("limelight");
        targetY += ty * distanceCorrectionFactor;
        //Get robot's current pose
        Pose2d currentPose = swerveDrive.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotYaw = swerveDrive.getHeading();

        // Calculate direction of tag relative to robot
        //If robotYaw is too high(turn left), if too low(turn right)
        double rotationSpeed = rotationPID.calculate(robotYaw, targetYaw);

        //Calculate strafe speed using PID control(high=left, low=right)
        double strafeSpeed = strafePID.calculate(robotX, targetX);

        // Compute forward movement
        double distance = Math.hypot(targetX - robotX, targetY - robotY);
        double forwardSpeed = forwardPID.calculate(distance, MIN_DISTANCE_FROM_TAG);
        forwardSpeed = MathUtil.clamp(forwardSpeed, -0.3, 0.3);

        // Move field oriented using Yagsl
        swerveDrive.drive(new Translation2d(forwardSpeed, strafeSpeed), rotationSpeed, false, true);

        //Horizontal offset from AprilTag(degrees)
        SmartDashboard.putNumber("Limelight TX (Rotation)", LimelightHelpers.getTX("limelight"));
        //Vertical offset from AprilTag(degrees)
        SmartDashboard.putNumber("Limelight TY (Distance)", LimelightHelpers.getTY("limelight"));
        
        SmartDashboard.putNumber("Vision Odometry X", visionPose.pose.getX());
        SmartDashboard.putNumber("Vision Odometry Y", visionPose.pose.getY());
}

@Override
    public boolean isFinished() {
        return rotationPID.atSetpoint() && strafePID.atSetpoint() && forwardPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
        rotationPID.reset();
        strafePID.reset();  
        forwardPID.reset();
        

    }
}

