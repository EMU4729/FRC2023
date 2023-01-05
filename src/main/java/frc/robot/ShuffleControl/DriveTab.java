package frc.robot.ShuffleControl;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Subsystems;
import frc.robot.teleop.TeleopProvider;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class DriveTab {
  private ShuffleboardTab drive = Shuffleboard.getTab("Drive");

  private NetworkTableEntry controlX = drive
      .add("Control X axis", 0)
      .withSize(4, 1).withPosition(0, 2)
      .withWidget(BuiltInWidgets.kNumberBar)
      .getEntry();

  private NetworkTableEntry controlY = drive
      .add("Control Y axis", 0)
      .withSize(4, 1).withPosition(0, 3)
      .withWidget(BuiltInWidgets.kNumberBar)
      .getEntry();

  public void setControlAxis(double contX, double contY) {
    controlX.setDouble(contX);
    controlY.setDouble(contY);
  }

  private NetworkTableEntry steerPIDGraphSense = drive
      .add("Steering Graph", new double[] { 0, 0 })
      .withSize(4, 3).withPosition(5, 3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();

  public void setSteerGraph(double in, double out) {
    steerPIDGraphSense.setDoubleArray(new double[] { in, out });
  }

  private NetworkTableEntry throtPIDGraphSense = drive
      .add("Throttle Graph", new double[] { 0, 0 })
      .withSize(4, 3).withPosition(5, 0)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();

  public void setThrotGraph(double in, double out) {
    throtPIDGraphSense.setDoubleArray(new double[] { in, out });
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

    drive
        .add("Throttle PID", Subsystems.drive.pidThrot)
        .withSize(1, 3).withPosition(4, 0)
        .withWidget(BuiltInWidgets.kPIDController);

    drive
        .add("Steering PID", Subsystems.drive.pidSteer)
        .withSize(1, 3).withPosition(4, 3)
        .withWidget(BuiltInWidgets.kPIDController);
  }
}
