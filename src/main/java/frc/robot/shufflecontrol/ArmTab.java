package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ArmTab {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  private final GenericEntry upperArmOutput = tab.add("Upper Arm Output", 0).getEntry();
  private final GenericEntry foreArmOutput = tab.add("Fore Arm Output", 0).getEntry();
  private final GenericEntry upperArmEncoderAngle = tab.add("Upper Arm Encoder Distance", 0).getEntry();
  private final GenericEntry foreArmEncoderAngle = tab.add("Fore Arm Encoder Distance", 0).getEntry();
  private final GenericEntry kinematicsX = tab.add("Kinematics-Calculated X", 0).getEntry();
  private final GenericEntry kinematicsY = tab.add("Kinematics-Calculated Y", 0).getEntry();

  public void setOutputs(double upperArm, double foreArm) {
    upperArmOutput.setDouble(upperArm);
    foreArmOutput.setDouble(foreArm);
  }

  public void setEncoderAngles(double upperArm, double foreArm) {
    upperArmEncoderAngle.setDouble(upperArm);
    foreArmEncoderAngle.setDouble(foreArm);
  }

  public void setKinematicsCoords(double x, double y) {
    kinematicsX.setDouble(x);
    kinematicsY.setDouble(y);
  }
}
