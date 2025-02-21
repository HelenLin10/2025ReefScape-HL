package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    
    private SparkFlex rotationalMotor;
    /*
     * Clockwise = Down 
     * Counter Clockwise = Up
     */
    
     private PIDController pidController;

    private RelativeEncoder wristEncoder;

    private double speed;
    private double position;

    public Wrist(){
        rotationalMotor = new SparkFlex(Constants.rotationalMotorID, MotorType.kBrushless);
        
        wristEncoder = rotationalMotor.getEncoder();

        pidController = new PIDController(0.1, 0.0, 0.0);
        pidController.setTolerance(0.3);
        speed = 0.3;
    }

    public void rotateUp(){
        // Right D-Pad
        rotationalMotor.set(speed);
    }
    
    public void rotateDown(){
        // Left D-Pad
        rotationalMotor.set(-speed);
    }
    
    public void holdPosition() {
        double output = pidController.calculate(wristEncoder.getPosition(), position);
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
}
