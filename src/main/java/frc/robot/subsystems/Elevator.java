package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;

    private static RelativeEncoder encoder; // Use one encoder for control
        // PID Controller (Tune these values)
        private PIDController pidController;
    
        private double setpoint; // Desired elevator position
        private double maxHeight = 105.6;
        private double minHeight = 0;
    
        private double elevatorMultiplier;
        private double manualSpeed;
        private boolean manualMove;
    
        private boolean killSwitch;
    
    
        public Elevator() {
        
            elevatorLeft = new SparkFlex(Constants.elevatorLeftID, MotorType.kBrushless);
            elevatorRight = new SparkFlex(Constants.elevatorRightID, MotorType.kBrushless);
    
            encoder = elevatorRight.getEncoder();
    
            //kp controls speed
            pidController = new PIDController(0.1, 0.0, 0.0);
            
            pidController.setTolerance(0.1); // Small error tolerance
    
            manualSpeed = 0.6;
            manualMove = false;
    
            setpoint = 0.0;
    
            killSwitch = false;
    
    
        }
    
        public static double getElevatorPosition(){
            return encoder.getPosition();
    }
    public void moveUp(){
        double wristPosition = Wrist.getWristPosition();
        if (wristPosition < 3){
            setpoint = getElevatorPosition();
        }
        setpoint = setpoint + manualSpeed;
        
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
        manualMove = true;
        
    }

    public void moveDown(){
        setpoint = setpoint - manualSpeed;

        if(setpoint < minHeight){
            setpoint = minHeight;
        }
        manualMove = true;
    }

    //Press a button to move to a certain height
    public void setHeight(double targetPosition) {
        
        setpoint = targetPosition;
        
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
        
        if(setpoint < minHeight){
            setpoint = minHeight;
        }
        manualMove = false;
    }

    public double getHeight() {
        return encoder.getPosition();
    }

    @Override // Runs every 10 ms
    public void periodic() {
        if(killSwitch == false) {
            if(manualMove){
                elevatorMultiplier = 1;
            }
            else if(manualMove == false){
                elevatorMultiplier = 0.5;
            }
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        double position = encoder.getPosition();
        double speed = pidController.calculate(position, setpoint);
        double output = pidController.calculate(encoder.getPosition(), setpoint);
        
        //Apply the same speed to both motors for sync
        elevatorLeft.set(speed * elevatorMultiplier);
        elevatorRight.set(speed * elevatorMultiplier);

        if (-0.3 > RobotContainer.getLeftYValue()){
            moveUp();
        }
        if (0.3 < RobotContainer.getLeftYValue()){
            moveDown();
        }
    }
}
}