package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.TimedRunCommand;

public class SuperNaiveAuto extends SequentialCommandGroup {
  protected SuperNaiveAuto() {
    addCommands(
        new TimedRunCommand(() -> Subsystems.drive.tank(-0.6, -0.6), 1.3, Subsystems.drive),
        new InstantCommand(Subsystems.drive::off, Subsystems.drive));
  }
}
