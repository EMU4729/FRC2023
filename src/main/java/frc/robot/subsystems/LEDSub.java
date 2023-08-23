package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LEDSub extends SubsystemBase {
  public final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.led.STRING_LENGTH);

  private final AddressableLED led = new AddressableLED(Constants.led.STRING_PORT);
  private final AddressableLEDSim ledSim = new AddressableLEDSim(led);

  public LEDSub() {
    led.setLength(Constants.led.STRING_LENGTH);
    led.setData(new AddressableLEDBuffer(Constants.led.STRING_LENGTH));
    led.start();
    ledSim.setInitialized(true);
  }

  public void apply() {
    led.setData(buffer);
  }
}
