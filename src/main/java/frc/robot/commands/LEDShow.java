package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.constants.Constants;
import frc.robot.utils.LED.LEDPattern;

public class LEDShow extends CommandBase {
  private Timer timer = new Timer();
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.led.STRING_LENGTH);
  private final List<LEDPattern> patterns;

  private int currentPattern = 0;

  public LEDShow(List<LEDPattern> patterns) {
    this.patterns = patterns;
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
