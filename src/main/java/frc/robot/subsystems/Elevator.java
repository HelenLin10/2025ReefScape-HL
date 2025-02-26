package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
public class Elevator extends SubsystemBase {

    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;

    private RelativeEncoder encoder; // Use one encoder for control

    // PID Controller (Tune these values)
    private PIDController pidController;

    private double setpoint; // Desired elevator position
    private double maxHeight = 400;
    private double minHeight = 0;

    public Elevator() {
    
        elevatorLeft = new SparkFlex(Constants.elevatorLeftID, MotorType.kBrushless);
        elevatorRight = new SparkFlex(Constants.elevatorRightID, MotorType.kBrushless);

        encoder = elevatorRight.getEncoder();

        //kp controls speed
        pidController = new PIDController(0.25, 0.0, 0.0);

        pidController.setTolerance(0.05); // Small error tolerance

        setpoint = 0.0;
    
    }

    public void moveUp(){
        setpoint = setpoint + 0.2;
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
        if(setpoint < minHeight){
            setpoint = minHeight;
        }
        
    }

    public void moveDown(){
        setpoint = setpoint - 0.2;
        
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
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        // Run PID control in the periodic loop
        double position = encoder.getPosition(); 
        double speed = pidController.calculate(position, setpoint);

        // Apply the same speed to both motors for sync
        elevatorRight.set(speed);
        elevatorLeft.set(-speed);

        if(-0.3 > RobotContainer.getYValue()){
            moveUp();
        }
        if(0.3 < RobotContainer.getYValue()){
            moveDown();
        }
}
}