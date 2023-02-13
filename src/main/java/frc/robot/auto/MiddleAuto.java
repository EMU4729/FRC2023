package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.BalanceChargePad;

public class MiddleAuto extends SequentialCommandGroup {
  public final Constants cnst = Constants.getInstance();

  public MiddleAuto() {
    addCommands(
        // Drop off preloaded game object
        Subsystems.arm.upperRung(),
        new InstantCommand(Subsystems.gripperGrip::open, Subsystems.gripperGrip),
        new WaitCommand(1),

        // Move to middle game object in middle of field
        new PathWeaverCommand("paths/MidDropOffToMidGameObject.wpilib.json"),

        // Pick up game object
        Subsystems.arm.farField(),
        new InstantCommand(Subsystems.gripperGrip::close, Subsystems.gripperGrip),
        new WaitCommand(1),

        Commands.sequence(
            // Move to charge pad and balance
            new PathWeaverCommand("paths/MidGameObjectToChargePad.wpilib.json"),
            new BalanceChargePad())
            // Move the arm to lowfield concurrently
            .alongWith(
                Subsystems.arm.lowField()));
  }
}
