package frc.robot.shufflecontrol;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleControl {

  private static DriveTab driveTab = new DriveTab();
  private static NavTab navTab = new NavTab();
  public static Field2d field = new Field2d();

  public ShuffleControl() {
    SmartDashboard.putData("Field", field);
  }

  // drive tab
  public static void setControlAxis(double contX, double contY) {
    driveTab.setControlAxis(contX, contY);
  }

  // nav tab
  public void setGyro(double gyro, double encode) {
    // This is commented out as the nav tab is entirely graphs and absolutely hogs
    // the driver station's memory

    // navTab.setGyroAngle(gyro, encode);
  }
}
