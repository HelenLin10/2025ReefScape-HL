package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final double maxSpeed = Units.feetToMeters(4.5);
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.5;
  }

  //ELEVATOR
  public static int elevatorRightID = 13;
  public static int elevatorLeftID = 14;


  //INTAKE
  public static int intakeMotorID = 15;
  public static int rotationalMotorID = 16;

}
