package frc.robot.ShuffleControl;

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

  public static void setSteerGraph(double in, double out) {
    driveTab.setSteerGraph(in, out);
  }

  public static void setThrotGraph(double in, double out) {
    driveTab.setThrotGraph(in, out);
  }

  // nav tab
  public void setGyro(double gyro, double encode) {
    navTab.setGyroAngle(gyro, encode);
  }
}
