package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.constants.Constants;
import frc.robot.utils.LED.DirectionPattern;
import frc.robot.utils.LED.LEDPattern;
import frc.robot.utils.LED.SolidPattern;

public class LEDShow extends CommandBase {
  private Timer timer = new Timer();
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.led.STRING_LENGTH);
  private final List<LEDPattern> patterns;
  private final boolean repeat;

  private int currentPattern = 0;

  public static LEDShow direction() {
    return new LEDShow(
        List.of(
            new DirectionPattern(250),
            new SolidPattern(Color.kBlack, 250)),
        true);
  }

  public static LEDShow cube() {
    return new LEDShow(
        List.of(
            new SolidPattern(Color.kMagenta, 250),
            new SolidPattern(Color.kBlack, 250)),
        true);
  }

  public static LEDShow cone() {
    return new LEDShow(
        List.of(
            new SolidPattern(Color.kYellow, 250),
            new SolidPattern(Color.kBlack, 250)),
        true);
  }

  public LEDShow(List<LEDPattern> patterns, boolean repeat) {
    this.patterns = patterns;
    this.repeat = repeat;
    addRequirements(Subsystems.led);
  }

  public LEDShow(List<LEDPattern> patterns) {
    this(patterns, false);
  }

  private void updateLEDs() {
    if (currentPattern >= patterns.size())
      return;
    LEDPattern pattern = patterns.get(currentPattern);
    pattern.applyPattern(buffer);
    Subsystems.led.setLEDs(buffer);
  }

  @Override
  public void initialize() {
    timer = new Timer();
    updateLEDs();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(patterns.get(currentPattern).getDuration())) {
      currentPattern += 1;
      if (repeat && currentPattern >= patterns.size()) {
        currentPattern = 0;
      }
      timer = new Timer();
      updateLEDs();
    }
  }

  @Override
  public boolean isFinished() {
    return repeat && currentPattern >= patterns.size();
  }

  @Override
  public void end(boolean interrupted) {
    new SolidPattern(Color.kBlack, 0).applyPattern(buffer);
    Subsystems.led.setLEDs(buffer);
  }

}
