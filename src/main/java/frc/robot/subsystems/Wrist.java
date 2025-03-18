package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase{
    
    private SparkFlex rotationalMotor;
    private PIDController pidController;
    private RelativeEncoder wristEncoder;
    private double position;
    private double maxLimit;
    private double minLimit;
    private double coralIntakePosition;
    public static double currentPosition;
    private boolean manualMove;
    private double wristMultiplier;

    public Wrist(){
        rotationalMotor = new SparkFlex(Constants.rotationalMotorID, MotorType.kBrushless);
        wristEncoder = rotationalMotor.getEncoder();
        pidController = new PIDController(0.1, 0.0, 0.0);
        pidController.setTolerance(0.3);
        maxLimit = 14.5;
        minLimit = 0;
        coralIntakePosition = 1.85;
    }

    //Allow other classes to get wrist position
    public static double getWristPosition(){
        return currentPosition;
    }

    public void rotateUp(double distance){
        // Right Bumper
        position = position + distance;
        if(position > maxLimit){
            position = maxLimit;
        }
        manualMove = true;
    }
    
    public void rotateDown(double distance){
        // Left Bumper
        position = position - distance;
        if(position < minLimit){
            position = minLimit;
        }
        manualMove = true;
        /*if(Elevator.getElevatorPosition() > 10){
            double safeMinLimit = 5.0;
            position = Math.max(position-distance, safeMinLimit);
        }*/
    }
    
    public void holdPosition() {
        if(manualMove){
            wristMultiplier = 1;
        }
        else if(manualMove == false){
            wristMultiplier = 0.5;
        }
        double output = pidController.calculate(wristEncoder.getPosition(), position);
        rotationalMotor.set(output * wristMultiplier);
    }

    public void stop() {
        position = wristEncoder.getPosition(); // Save position to hold
    }

    public void setPosition(double m_position){
        position = m_position;
        manualMove = false;
    }

    public boolean atSetpoint() {
    return pidController.atSetpoint(); // Uses WPILib's built-in tolerance checking
}

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Position", position);
        if(RobotContainer.getRightYValue() > 0.3){
            rotateUp(0.2);
        }
        else if(RobotContainer.getRightYValue() < -0.3){
            rotateDown(0.2);
        }
        else if(RobotContainer.leftBumperPressed()){
            setPosition(coralIntakePosition);
        }
        else if(RobotContainer.rightBumperPressed()){
            setPosition(14.2);
        }
        if(minLimit == 0){
            if(position > 3){
                minLimit = coralIntakePosition;
            }
        }
        holdPosition();
        currentPosition = wristEncoder.getPosition();
    }
}
