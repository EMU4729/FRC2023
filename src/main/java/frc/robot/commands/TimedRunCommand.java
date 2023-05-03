package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * A command that runs a function repeatedly for the specified duration.
 */
public class TimedRunCommand extends SequentialCommandGroup {
  /**
   * @param toRun        The runnable to execute repeatedly
   * @param duration     The duration of the command
   * @param requirements The subsystems the command requires
   */
  public TimedRunCommand(Runnable toRun, double duration, Subsystem... requirements) {
    addCommands(
        new RunCommand(toRun, requirements).withTimeout(duration),
        new WaitCommand(duration));
  }
}
