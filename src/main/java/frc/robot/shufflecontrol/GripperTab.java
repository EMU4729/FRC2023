package frc.robot.shufflecontrol;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class GripperTab {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Gripper");

  public final GenericEntry valueOneEntry = tab.add("Servos Value", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .withSize(2, 1)
      .getEntry();

  public double getValue() {
    return valueOneEntry.getDouble(0);
  }
}
