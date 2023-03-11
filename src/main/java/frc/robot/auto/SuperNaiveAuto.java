package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;

public class SuperNaiveAuto extends SequentialCommandGroup {
  private final double runDuration = 1;

  protected SuperNaiveAuto() {
    addCommands(
        new RunCommand(() -> Subsystems.drive.tank(-0.6, -0.6), Subsystems.drive).withTimeout(runDuration),
        new WaitCommand(runDuration),
        new InstantCommand(Subsystems.drive::off, Subsystems.drive));
  }
}
