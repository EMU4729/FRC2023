package frc.robot.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class SolidPattern implements LEDPattern {
  private final Color color;
  private final double duration;

  public SolidPattern(Color color, double duration) {
    this.color = color;
    this.duration = duration;
  }

  public void applyPattern(AddressableLEDBuffer buffer) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }

  public double getDuration() {
    return duration;
  }
}
