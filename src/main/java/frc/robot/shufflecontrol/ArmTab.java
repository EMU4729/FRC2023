package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ArmTab {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  private final GenericEntry seg1Output = tab.add("Seg1 Output", 0).getEntry();
  private final GenericEntry seg2Output = tab.add("Seg2 Output", 0).getEntry();
  private final GenericEntry seg1EncoderAngle = tab.add("Seg1 Encoder Angle (degrees)", 0).getEntry();
  private final GenericEntry seg2EncoderAngle = tab.add("Seg2 Encoder Angle (degrees)", 0).getEntry();
  private final GenericEntry seg1EncoderCounts = tab.add("Seg1 Encoder Counts", 0).getEntry();
  private final GenericEntry seg2EncoderCounts = tab.add("Seg2 Encoder Counts", 0).getEntry();
  private final GenericEntry seg1EncoderRate = tab.add("Seg1 Encoder Rate (degrees/sec)", 0).getEntry();
  private final GenericEntry seg2EncoderRate = tab.add("Seg2 Encoder Rate (degrees/sec)", 0).getEntry();
  private final GenericEntry seg1Voltage = tab.add("Seg1 Voltage", 0).getEntry();
  private final GenericEntry seg2Voltage = tab.add("Seg2 Voltage", 0).getEntry();
  private final GenericEntry seg1Current = tab.add("Seg1 Current", 0).getEntry();
  private final GenericEntry seg2Current = tab.add("Seg2 Current", 0).getEntry();
  private final GenericEntry forKX = tab.add("Forward Kinematics X (m)", 0).getEntry();
  private final GenericEntry forKY = tab.add("Forward Kinematics Y (m)", 0).getEntry();
  private final GenericEntry updateDelta = tab.add("Update Delta (ms)", 0).getEntry();

  public void setOutputs(double seg1, double seg2) {
    seg1Output.setDouble(seg1);
    seg2Output.setDouble(seg2);
  }

  public void setEncoderAngles(double seg1, double seg2) {
    seg1EncoderAngle.setDouble(seg1);
    seg2EncoderAngle.setDouble(seg2);
  }

  public void setEncoderCounts(double seg1, double seg2) {
    seg1EncoderCounts.setDouble(seg1);
    seg2EncoderCounts.setDouble(seg2);
  }

  public void setEncoderRates(double seg1, double seg2) {
    seg1EncoderRate.setDouble(seg1);
    seg2EncoderRate.setDouble(seg2);
  }

  public void setVoltages(double seg1, double seg2) {
    seg1Voltage.setDouble(seg1);
    seg2Voltage.setDouble(seg2);
  }

  public void setCurrents(double seg1, double seg2) {
    seg1Current.setDouble(seg1);
    seg2Current.setDouble(seg2);
  }

  public void setKinematicsCoords(double x, double y) {
    forKX.setDouble(x);
    forKY.setDouble(y);
  }

  public void setUpdateDelta(double delta) {
    updateDelta.setDouble(delta);
  }
}
