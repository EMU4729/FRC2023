package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class RainbowLEDCommand extends CommandBase {
  private int firstPixel = 0;

  public RainbowLEDCommand() {
    addRequirements(Subsystems.led);
  }

  @Override
  public void execute() {
    for (int i = 0; i < Subsystems.led.buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (firstPixel + (i * 180 / Subsystems.led.buffer.getLength())) % 180;
      // Set the value
      Subsystems.led.buffer.setHSV(i, hue, 255, 128);
    }

    // Increase by to make the rainbow "move"
    firstPixel += 3;
    // Check bounds
    firstPixel %= 180;

    Subsystems.led.apply();
  }

}
