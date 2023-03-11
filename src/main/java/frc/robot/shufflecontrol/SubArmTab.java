package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SubArmTab {
  private final ShuffleboardTab tab = Shuffleboard.getTab("SubArm");

  private final GenericEntry outputEntry = tab.add("Pivot Motor Output", 0).getEntry();
  private final GenericEntry targetAngleEntry = tab.add("Pivot Target Angle", 0).getEntry();
  private final GenericEntry encoderAngleEntry = tab.add("Pivot Current Angle", 0).getEntry();
  private final GenericEntry errorEntry = tab.add("Pivot Controller Error", 0).getEntry();
  private final GenericEntry rotationAngleEntry = tab.add("Rotation Current Angle", 0).getEntry();
  private final GenericEntry rotationAngleInBounds = tab.add("Rotation In Bounds", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

  public void setOutput(double output) {
    outputEntry.setDouble(output);
  }

  public void setTargetAngle(double targetAngle) {
    targetAngleEntry.setDouble(targetAngle);
  }

  public void setEncoderAngle(double encoderAngle) {
    encoderAngleEntry.setDouble(encoderAngle);
  }

  public void setControllerError(double error) {
    errorEntry.setDouble(error);
  }

  public void setRotationAngle(double angle) {
    rotationAngleEntry.setDouble(angle);
  }

  public void setRotationInBounds(boolean value) {
    rotationAngleInBounds.setBoolean(value);
  }
}
