package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.Variables;

public class DirectionLEDCommand extends CommandBase {
  private final int TICKS_PER_FRAME = 5;
  private final int END_LENGTH = 5;
  private int ticksUntilNextFrame = TICKS_PER_FRAME;
  private boolean isBlack = false;

  public DirectionLEDCommand() {
    addRequirements(Subsystems.led);
  }

  @Override
  public void execute() {
    if (isBlack) {
      for (int i = 0; i < Subsystems.led.buffer.getLength(); i++) {
        Subsystems.led.buffer.setLED(i, Color.kBlack);
      }
    } else {
      for (int i = 0; i < Subsystems.led.buffer.getLength(); i++) {
        if (i < END_LENGTH) {
          Subsystems.led.buffer.setLED(i, Variables.invertDriveDirection ? Color.kBlue : Color.kRed);
        } else if (i >= Subsystems.led.buffer.getLength() - END_LENGTH) {
          Subsystems.led.buffer.setLED(i, Variables.invertDriveDirection ? Color.kRed : Color.kBlue);
        } else {
          Subsystems.led.buffer.setLED(i, Color.kBlack);
        }
      }
    }

    Subsystems.led.apply();

    ticksUntilNextFrame -= 1;
    if (ticksUntilNextFrame <= 0) {
      isBlack = !isBlack;
      ticksUntilNextFrame = TICKS_PER_FRAME;
    }
  }
}
