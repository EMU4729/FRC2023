package frc.robot.ShuffleControl;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NavTab {
  private ShuffleboardTab nav = Shuffleboard.getTab("Nav");

  private GenericEntry gyroRot = nav
      .add("GyroRotation", 0)
      .withSize(3, 2).withPosition(0, 0)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();

  private GenericEntry encoderRot = nav
      .add("EncodeRotation", 0)
      .withSize(3, 2).withPosition(3, 0)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();

  public void setGyroAngle(double gyro, double encode) {
    gyroRot.setDouble(gyro);
    encoderRot.setDouble(encode);
  }
}
