package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;

    private RelativeEncoder encoder; // Use one encoder for control

    // PID Controller (Tune these values)
    private PIDController pidController;

    private double setpoint; // Desired elevator position
    private double maxHeight = 5;
    private double minHeight = 0;

    public Elevator() {
        pidController.setTolerance(0.05); // Small error tolerance
    
        elevatorLeft = new SparkFlex(Constants.elevatorLeftID, MotorType.kBrushless);
        elevatorRight = new SparkFlex(Constants.elevatorRightID, MotorType.kBrushless);

        encoder = elevatorRight.getEncoder();

        pidController = new PIDController(0.1, 0.0, 0.0);

        setpoint = 0.0;
    
    }

    public void moveUp(){
        setpoint = setpoint + 0.1;
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
        if(setpoint < minHeight){
            setpoint = minHeight;
        }
    }

    public void moveDown(){
        setpoint = setpoint - 0.1;
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
        if(setpoint < minHeight){
            setpoint = minHeight;
        }
    }

    public void setHeight(double targetPosition) {
        setpoint = targetPosition;
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
        if(setpoint < minHeight){
            setpoint = minHeight;
        }
    }

    public boolean isAtSetpoint() {
        return pidController.atSetpoint(); //may not be needed
    }    

    @Override
    public void periodic() {
        // Run PID control in the periodic loop
        double position = encoder.getPosition(); 
        double speed = pidController.calculate(position, setpoint);

        // Apply the same speed to both motors for sync
        elevatorRight.set(speed);
        elevatorLeft.set(-speed);
    }
}