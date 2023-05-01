package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TimedRunCommand extends SequentialCommandGroup {
  public TimedRunCommand(Runnable toRun, double duration, Subsystem... requirements) {
    addCommands(
        new RunCommand(toRun, requirements).withTimeout(duration),
        new WaitCommand(duration));
  }
}
