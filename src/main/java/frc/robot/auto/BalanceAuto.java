package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.BalanceChargePad;

public class BalanceAuto extends SequentialCommandGroup {
    private final double unbalanceSeconds = 0.1;
    private final double dropSeconds = 0.1;
    private final double moveSeconds = 0.5;

    protected BalanceAuto() {
        addCommands(
            // Move Forward to unbalance cube
            new RunCommand(() -> Subsystems.drive.tank(-0.2, -0.2)).withTimeout(unbalanceSeconds),
            new WaitCommand(unbalanceSeconds),

            // Move Back to drop cube
            new RunCommand(() -> Subsystems.drive.tank(0.2, 0.2), Subsystems.drive).withTimeout(dropSeconds),
            new WaitCommand(dropSeconds),

            // Move Forward to charge pad
            new RunCommand(() -> Subsystems.drive.tank(-0.4, -0.4)).until(() -> Subsystems.nav.getRoll() > 10).withTimeout(4),

            // Balance
            new BalanceChargePad()
        );
    }
}
