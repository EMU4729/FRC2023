package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;

public class BumplessSuperNaiveAuto extends SequentialCommandGroup {
  private final double runDuration = 1;

  protected BumplessSuperNaiveAuto() {
    addCommands(
        new RunCommand(() -> Subsystems.drive.tank(-0.5, -0.5), Subsystems.drive).withTimeout(runDuration),
        new WaitCommand(runDuration),
        new InstantCommand(Subsystems.drive::off, Subsystems.drive));
  }
}
