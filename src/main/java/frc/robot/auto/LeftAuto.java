package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems;

public class LeftAuto extends SequentialCommandGroup {
  public final Constants cnst = Constants.getInstance();

  public LeftAuto() {
    addCommands(
        new InstantCommand(Subsystems.arm::upperRung, Subsystems.arm),
        new WaitCommand(2),
        new InstantCommand(Subsystems.gripperGrip::open, Subsystems.gripperGrip),
        new WaitCommand(1),
        new PathWeaverCommand("paths/LeftDropOffToLeftGameObject.wpilib.json"),
        new InstantCommand(Subsystems.arm::farField, Subsystems.arm),
        new WaitCommand(2),
        new InstantCommand(Subsystems.gripperGrip::close, Subsystems.gripperGrip),
        new WaitCommand(1),
        new InstantCommand(Subsystems.arm::lowField, Subsystems.arm));
  }
}
