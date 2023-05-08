package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;

/**
 * Naive Auto - Drives forwards until the encoders report 3 metres
 */
public class NaiveAuto extends SequentialCommandGroup {
  protected NaiveAuto() {
    addRequirements(Subsystems.drive);
    addCommands(
        new RunCommand(() -> Subsystems.drive.tank(0.75, 0.75), Subsystems.drive)
            .until(() -> Subsystems.nav.getAvgEncoderDistance() >= 3),
        new InstantCommand(Subsystems.drive::off, Subsystems.drive));
  }
}
