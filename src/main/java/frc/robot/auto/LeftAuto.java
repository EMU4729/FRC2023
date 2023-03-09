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
        // Drop preloaded game object
        Subsystems.arm.upperRung(),
        new InstantCommand(Subsystems.gripperGrip::open, Subsystems.gripperGrip),
        new WaitCommand(1),

        // Move to left game object in middle of field
        new PathWeaverCommand("paths/LeftDropOffToLeftGameObject.wpilib.json"),

        // Pick up game object
        Subsystems.arm.farField(),
        new InstantCommand(Subsystems.gripperGrip::closeCone, Subsystems.gripperGrip),
        new WaitCommand(1),
        Subsystems.arm.lowField());
  }
}
