package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;

public class NaiveAuto extends SequentialCommandGroup {
  protected NaiveAuto() {
    addCommands(
        // Reverse for 2 seconds
        new InstantCommand(() -> Subsystems.drive.tank(-0.5, -0.5), Subsystems.drive),
        new WaitCommand(2.5),
        new InstantCommand(Subsystems.drive::off, Subsystems.drive));
  }
}
