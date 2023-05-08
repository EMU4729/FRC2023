package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Subsystems;
import frc.robot.teleop.TeleopProvider;

public class DriveTab {
  private ShuffleboardTab drive = Shuffleboard.getTab("Drive");

  private final GenericEntry controlX = drive.add("Control X axis", 0)
      .withSize(4, 1).withPosition(0, 2)
      .withWidget(BuiltInWidgets.kNumberBar)
      .getEntry();

  private final GenericEntry controlY = drive.add("Control Y axis", 0)
      .withSize(4, 1).withPosition(0, 3)
      .withWidget(BuiltInWidgets.kNumberBar)
      .getEntry();

  public void setControlAxis(double contX, double contY) {
    controlX.setDouble(contX);
    controlY.setDouble(contY);
  }

  protected DriveTab() {
    drive
        .add("DriveOutput", Subsystems.drive.drive)
        .withSize(4, 2).withPosition(0, 0)
        .withWidget(BuiltInWidgets.kDifferentialDrive);

    drive
        .add("TeleopType", TeleopProvider.getInstance().chooser)
        .withSize(2, 1).withPosition(2, 4)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
  }
}
