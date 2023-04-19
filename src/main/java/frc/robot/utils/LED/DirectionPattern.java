package frc.robot.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Variables;

public class DirectionPattern implements LEDPattern {
  private final double duration;

  public DirectionPattern(double duration) {
    this.duration = duration;
  };

  public void applyPattern(AddressableLEDBuffer buffer) {
    for (int i = 0; i < buffer.getLength(); i++) {
      if (i <= 2) {
        buffer.setLED(i, Variables.invertDriveDirection ? Color.kGreen : Color.kRed);
      } else if (i >= buffer.getLength() - 3) {
        buffer.setLED(i, Variables.invertDriveDirection ? Color.kRed : Color.kGreen);
      } else {
        buffer.setLED(i, Color.kBlack);
      }
    }
  }

  public double getDuration() {
    return duration;
  }

}
