package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private SparkFlex intakeMotor;
    /*
     *  COUNTERCLOCKWISE IS POSITIVE
     *
     * Clockwise = Ball In / Tube Out
     * Counter Clockwise = Ball Out / Tube In
     */    
    public Intake(){
        intakeMotor = new SparkFlex(Constants.intakeMotorID, MotorType.kBrushless);
    }

    public void BallIn_TubeOut(double speed){
        // Right Trigger RT
        intakeMotor.set(-speed);
    }

    public void BallOut_TubeIn(double speed){
        // Left Trigger LT
        intakeMotor.set(speed);
    }
}