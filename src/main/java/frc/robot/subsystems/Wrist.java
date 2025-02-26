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
    /*
     * Clockwise = Down 
     * Counter Clockwise = Up
     */
    
     private PIDController pidController;

    private RelativeEncoder wristEncoder;

    private double position;

    private double maxLimit;
    private double minLimit;

    public Wrist(){
        rotationalMotor = new SparkFlex(Constants.rotationalMotorID, MotorType.kBrushless);
        
        wristEncoder = rotationalMotor.getEncoder();

        pidController = new PIDController(0.1, 0.0, 0.0);
        pidController.setTolerance(0.3);

        maxLimit = 14.5;
        minLimit = 0;
    }

    public void rotateUp(){
        // Right Bumper
        position = position + 0.2;
        if(position > maxLimit){
            position = maxLimit;
        }
        if(position < minLimit){
            position = minLimit;
        }
    }
    
    public void rotateDown(){
        // Left Bumper
        position = position - 0.2;
        if(position > maxLimit){
            position = maxLimit;
        }
        if(position < minLimit){
            position = minLimit;
        }
    }
    
    public void holdPosition() {
        double output = pidController.calculate(wristEncoder.getPosition(), position);
        SmartDashboard.putNumber("Wrist Position", position);
        rotationalMotor.set(output);
    }
    public void stop() {
        position = wristEncoder.getPosition(); // Save position to hold
    }
    public void setPosition(double m_position){
        position = m_position;
    }
    public boolean atSetpoint() {
    return pidController.atSetpoint(); // Uses WPILib's built-in tolerance checking
}

    @Override
    public void periodic(){
        if(RobotContainer.rightBumperPressed()){
            rotateUp();
        }
        if(RobotContainer.leftBumperPressed()){
            rotateDown();
        }

        holdPosition();
    }
}