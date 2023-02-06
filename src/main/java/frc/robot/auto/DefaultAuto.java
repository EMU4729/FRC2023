package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.BalanceChargePad;

public class DefaultAuto extends SequentialCommandGroup {
  public final Constants cnst = Constants.getInstance();

  public DefaultAuto() {
    addCommands(
        new PathWeaverCommand("paths/GoToDropOff.wpilib.json"),
        new InstantCommand(Subsystems.arm::upperRung, Subsystems.arm),
        new WaitCommand(2),
        new InstantCommand(Subsystems.gripperGrip::open, Subsystems.gripperGrip),
        new WaitCommand(1),
        new PathWeaverCommand("paths/GoToChargePad.wpilib.json"),
        new BalanceChargePad());
  }
}
