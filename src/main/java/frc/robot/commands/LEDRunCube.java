package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.constants.Constants;

public class LEDRunCube extends CommandBase {
  private Timer timer = new Timer();
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.led.STRING_LENGTH);

  public LEDRunCube() {
    addRequirements(Subsystems.led);
  }

  @Override
  public void initialize() {
    timer = new Timer();
  }

  @Override
  public void execute() {
    double elapsed = timer.get();
    // TODO: Finish this
  }

}
