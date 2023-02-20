package frc.robot.shufflecontrol;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleControl {

  public static DriveTab driveTab = new DriveTab();
  public static NavTab navTab = new NavTab();
  public static ArmTab armTab = new ArmTab();
  public static Field2d field = new Field2d();

  public ShuffleControl() {
    SmartDashboard.putData("Field", field);
  }
}
