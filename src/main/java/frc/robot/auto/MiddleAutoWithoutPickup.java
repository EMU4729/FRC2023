package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.BalanceChargePad;

public class MiddleAutoWithoutPickup extends SequentialCommandGroup {
  public final Constants cnst = Constants.getInstance();

  public MiddleAutoWithoutPickup() {
    addCommands(
        // Drop off preloaded game object
        Subsystems.arm.upperRung(),
        new InstantCommand(Subsystems.gripperGrip::open, Subsystems.gripperGrip),
        new WaitCommand(1),

        // Move to charge pad via past the charge pad (to get points) and balance
        new PathWeaverCommand("paths/MidDropOffToChargePadViaPastChargePad.wpilib.json"),
        new BalanceChargePad());
  }
}
