package frc.robot.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LEDPattern {
  public void applyPattern(AddressableLEDBuffer buffer);

  public double getDuration();
}
