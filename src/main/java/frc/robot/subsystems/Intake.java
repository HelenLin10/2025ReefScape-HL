package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase{

    private SparkFlex intakeMotor;
    private SparkFlex intakeMotor2;

    public Intake(){
        intakeMotor = new SparkFlex(Constants.intakeMotorID, MotorType.kBrushless);
        intakeMotor2 = new SparkFlex(Constants.intakeMotorID2, MotorType.kBrushless);
    }

    public void BallIn_TubeOut(double speed){
        // Right Trigger RT
        intakeMotor.set(-speed * 0.5);
        intakeMotor2.set(speed * 0.5);
    }

    public void BallOut_TubeIn(double speed){
        // Left Trigger LT
        intakeMotor.set(speed);
        intakeMotor2.set(-speed);
    }
    @Override
    public void periodic(){
        if(RobotContainer.rightTriggerValue() > 0.05){
            BallIn_TubeOut(RobotContainer.rightTriggerValue());
        }
        else if(RobotContainer.leftTriggerValue() > 0.05){
            BallOut_TubeIn(RobotContainer.leftTriggerValue());
        }
        else{
            intakeMotor.set(0);
            intakeMotor2.set(0);}
    }
}