package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;

public class CalibrationTab {

  private final ShuffleboardTab tab = Shuffleboard.getTab("Calibration");
  private final GenericEntry armEntry = tab.add("Arm Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();
  private final GenericEntry subArmPivotEntry = tab.add("SubArm Pivot Calibrated", false)
      .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  private final GenericEntry subArmRotateEntry = tab.add("SubArm Rotation Calibrated", false)
      .withWidget(BuiltInWidgets.kBooleanBox).getEntry();

  protected CalibrationTab() {
    tab.add("Calibrate Arm", new InstantCommand(Subsystems.arm::calibrate));
    tab.add("Calibrate SubArm Pivot", new InstantCommand(Subsystems.subArmPivot::calibrate));
    tab.add("Calibrate SubArm Rotation", new InstantCommand(Subsystems.subArmRotate::calibrate));
    tab.add("Calibrate All", new InstantCommand(Subsystems::calibrate));
  }

  public void setArmCalibrated(boolean calibrated) {
    armEntry.setBoolean(calibrated);
  }

  public void setSubArmPivotCalibrated(boolean calibrated) {
    subArmPivotEntry.setBoolean(calibrated);
  }

  public void setSubArmRotateCalibrated(boolean calibrated) {
    subArmRotateEntry.setBoolean(calibrated);
  }

}
