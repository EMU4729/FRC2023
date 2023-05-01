package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NavTab {
  private ShuffleboardTab nav = Shuffleboard.getTab("Nav");

  private GenericEntry yawEntry = nav.add("Heading Degrees", 0).getEntry();
  private GenericEntry pitchEntry = nav.add("Pitch Degrees", 0).getEntry();
  private GenericEntry rollEntry = nav.add("Roll Degrees", 0).getEntry();
  private GenericEntry leftEncoderEntry = nav.add("Left Encoder Distance", 0).getEntry();
  private GenericEntry rightEncoderEntry = nav.add("Right Encoder Distance", 0).getEntry();

  protected NavTab() {
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
}
