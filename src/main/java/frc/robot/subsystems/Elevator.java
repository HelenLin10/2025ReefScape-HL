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
=======
        // kp controls speed
        pidController = new PIDController(0.1, 0.0, 0.001);

        pidController.setTolerance(0.1); // Small error tolerance

        manualSpeed = 0.6;
        manualMove = false;

        setpoint = 0.0;

        killSwitch = false;
    }

    public static double getElevatorPosition() {
        return position;
    }

    public void moveUp() {
        setpoint = setpoint + manualSpeed;

        if (setpoint > maxHeight) {
            setpoint = maxHeight;
        }
        manualMove = true;
    }

    public void moveDown() {
        setpoint = setpoint - manualSpeed;

        if (setpoint < minHeight) {
            setpoint = minHeight;
        }
        manualMove = true;
    }

    public void setHeight(double targetPosition) {
        if(Wrist.getWristPosition() > 7){
        setpoint = targetPosition;
        }
        if (setpoint > maxHeight) {
            setpoint = maxHeight;
        }

        if (setpoint < minHeight) {
>>>>>>> Stashed changes
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
        // Run PID control in the periodic loop
        double position = encoder.getPosition(); 
        double speed = pidController.calculate(position, setpoint);

        // Apply the same speed to both motors for sync
        elevatorRight.set(speed);
        elevatorLeft.set(-speed);
    }
}