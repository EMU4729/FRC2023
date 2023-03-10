package frc.robot.shufflecontrol;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavTab {
  private ShuffleboardTab nav = Shuffleboard.getTab("Nav");

  private GenericEntry yawEntry = nav.add("Heading Degrees", 0).getEntry();
  private GenericEntry pitchEntry = nav.add("Pitch Degrees", 0).getEntry();
  private GenericEntry rollEntry = nav.add("Roll Degrees", 0).getEntry();
  private GenericEntry leftEncoderEntry = nav.add("Left Encoder Distance", 0).getEntry();
  private GenericEntry rightEncoderEntry = nav.add("Right Encoder Distance", 0).getEntry();
  private Field2d field = new Field2d();

  protected NavTab() {
    nav.add("Field", field);
    SmartDashboard.putData(field);
  }

  public void setRotation(double yaw, double pitch, double roll) {
    yawEntry.setDouble(yaw);
    pitchEntry.setDouble(pitch);
    rollEntry.setDouble(roll);
  }

  public void setEncoderDistances(double left, double right) {
    leftEncoderEntry.setDouble(left);
    rightEncoderEntry.setDouble(right);
  }

  public void setPose(Pose2d pose) {
    field.setRobotPose(pose);
  }
}
