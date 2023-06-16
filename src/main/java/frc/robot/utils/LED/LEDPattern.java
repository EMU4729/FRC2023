package frc.robot.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Describes a specific pattern on the LED strip */
public interface LEDPattern {
  /**
   * Applies a pattern to the provided {@link AddressableLEDBuffer}
   * 
   * @param buffer The buffer to write to
   */
  public void applyPattern(AddressableLEDBuffer buffer);

  /** @return The duration of the pattern */
  public double getDuration();
}
