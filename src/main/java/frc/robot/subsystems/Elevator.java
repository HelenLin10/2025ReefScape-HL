package frc.robot.subsystems;

import frc.robot.Constants;
<<<<<<< Updated upstream
=======
import frc.robot.subsystems.Wrist;

>>>>>>> Stashed changes
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;

    private RelativeEncoder encoder; // Use one encoder for control

    // PID Controller (Tune these values)
    private PIDController pidController;

    private double setpoint; // Desired elevator position
<<<<<<< Updated upstream
    private double maxHeight = 5;
    private double minHeight = 0;

    public Elevator() {
        pidController.setTolerance(0.05); // Small error tolerance
    
=======
    private double maxHeight = 105.6;
    private double minHeight = 0;

    private double elevatorMultiplier;
    private double manualSpeed;
    private boolean manualMove;
    public static double position;

    private boolean killSwitch;

    public Elevator() {

>>>>>>> Stashed changes
        elevatorLeft = new SparkFlex(Constants.elevatorLeftID, MotorType.kBrushless);
        elevatorRight = new SparkFlex(Constants.elevatorRightID, MotorType.kBrushless);

        encoder = elevatorRight.getEncoder();

<<<<<<< Updated upstream
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
<<<<<<< Updated upstream
        // Run PID control in the periodic loop
        double position = encoder.getPosition(); 
        double speed = pidController.calculate(position, setpoint);

        // Apply the same speed to both motors for sync
        elevatorRight.set(speed);
        elevatorLeft.set(-speed);
=======
        if (killSwitch == false) {
            if (manualMove) {
                elevatorMultiplier = 1;
            } else if (manualMove == false) {
                elevatorMultiplier = 0.5;
            }
            SmartDashboard.putNumber("Elevator Setpoint", setpoint);
            position = encoder.getPosition(); //sdf
            double speed = pidController.calculate(position, setpoint);

            // Apply the same speed to both motors for sync
            elevatorRight.set(speed * elevatorMultiplier);
            elevatorLeft.set(-speed * elevatorMultiplier);

            if (-0.3 > RobotContainer.getLeftYValue()) {
                moveUp();
            }
            if (0.3 < RobotContainer.getLeftYValue()) {
                moveDown();
            }
            
        }
>>>>>>> Stashed changes
    }
}