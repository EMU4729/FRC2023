package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.utils.logger.Logger;

public class LEDSub extends SubsystemBase {
  private int firstPixel = 0;
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.led.STRING_LENGTH);

  private final AddressableLED led = new AddressableLED(Constants.led.STRING_PORT);
  
  private final AddressableLEDSim ledSim = new AddressableLEDSim(led);

  public LEDSub() {
    led.setLength(Constants.led.STRING_LENGTH);
    led.setData(new AddressableLEDBuffer(Constants.led.STRING_LENGTH));
    led.start();
    ledSim.setInitialized(true);
  }

  public void setLEDs(AddressableLEDBuffer buffer) {
    if (buffer.getLength() != Constants.led.STRING_LENGTH) {
      Logger.warn("ArmSub::setLights : Invalid buffer length " + buffer.getLength() + ", expected "
          + Constants.led.STRING_LENGTH);
      return;
    }

    led.setData(buffer);
  }

  @Override
  public void periodic() {
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (firstPixel + (i * 180 / buffer.getLength())) % 180;
      // Set the value
      buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    firstPixel += 3;
    // Check bounds
    firstPixel %= 180;

    setLEDs(buffer);
  }  
}
