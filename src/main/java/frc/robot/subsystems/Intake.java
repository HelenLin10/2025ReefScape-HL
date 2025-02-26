package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase{

    private SparkFlex intakeMotor;

    private RelativeEncoder encoder;
    
    private PIDController pidController;

    private double setpoint;
    /*
     *  COUNTERCLOCKWISE IS POSITIVE
     *
     * Clockwise = Ball In / Tube Out
     * Counter Clockwise = Ball Out / Tube In
     */    
    public Intake(){
        intakeMotor = new SparkFlex(Constants.intakeMotorID, MotorType.kBrushless);
        
        encoder = intakeMotor.getEncoder();

        pidController = new PIDController(0.1, 0, 0);
    
        pidController.setTolerance(0.05);
    }

    public void BallIn_TubeOut(double speed){
        // Right Trigger RT
        intakeMotor.set(-speed);
    }

    public void BallOut_TubeIn(double speed){
        // Left Trigger LT
        intakeMotor.set(speed);
    }
    @Override
    public void periodic(){
        if(RobotContainer.rightTriggerValue() > 0.1){
            BallIn_TubeOut(RobotContainer.rightTriggerValue());
            setpoint = encoder.getPosition();
        }
        else if(RobotContainer.leftTriggerValue() > 0.1){
            BallOut_TubeIn(RobotContainer.leftTriggerValue());
            setpoint = encoder.getPosition();
        }
        else{
            intakeMotor.set(0);
            double output = pidController.calculate(encoder.getPosition(), setpoint);
            intakeMotor.set(output);
        }
    }
}