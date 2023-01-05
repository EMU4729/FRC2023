package frc.robot.ShuffleControl;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Subsystems;

public class NavTab {
  private ShuffleboardTab nav = Shuffleboard.getTab("Nav");

  private NetworkTableEntry gyroRot = nav
      .add("GyroRotation", 0)
      .withSize(3, 2).withPosition(0, 0)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();

  private NetworkTableEntry encoderRot = nav
      .add("EncodeRotation", 0)
      .withSize(3, 2).withPosition(3, 0)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();

  public void setGyroAngle(double gyro, double encode){
    gyroRot.setDouble(gyro);
    encoderRot.setDouble(encode);
  }
}
