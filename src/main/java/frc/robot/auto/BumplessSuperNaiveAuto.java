package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.TimedRunCommand;

/** {@link SuperNaiveAuto}, but for the side without the bump */
public class BumplessSuperNaiveAuto extends SequentialCommandGroup {
  protected BumplessSuperNaiveAuto() {
    addCommands(
        new TimedRunCommand(() -> Subsystems.drive.tank(-0.5, -0.5), 0.55, Subsystems.drive),
        new InstantCommand(Subsystems.drive::off, Subsystems.drive));
  }
}
