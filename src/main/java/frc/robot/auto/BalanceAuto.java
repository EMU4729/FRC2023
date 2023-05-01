package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.BalanceChargePad;
import frc.robot.commands.TimedRunCommand;

public class BalanceAuto extends SequentialCommandGroup {
    protected BalanceAuto() {
        addCommands(
                // Move Forward to unbalance cube
                new TimedRunCommand(
                        () -> Subsystems.drive.tank(-0.2, -0.2),
                        0.1, Subsystems.drive),

                // Move Back to drop cube
                new TimedRunCommand(
                        () -> Subsystems.drive.tank(0.2, 0.2),
                        0.1,
                        Subsystems.drive),

                // Move Forward to charge pad
                new RunCommand(() -> Subsystems.drive.tank(-0.4, -0.4))
                        .until(() -> Subsystems.nav.getRoll() > 10)
                        .withTimeout(4),

                // Balance
                new BalanceChargePad());
    }
}
