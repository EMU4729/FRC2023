package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;

public class SuperNaiveAuto extends SequentialCommandGroup {
  protected SuperNaiveAuto() {
    addCommands(
        new InstantCommand(() -> Subsystems.drive.tank(-0.75, -0.75), Subsystems.drive),
        new WaitCommand(2.5),
        new InstantCommand(Subsystems.drive::off, Subsystems.drive));
  }
}
