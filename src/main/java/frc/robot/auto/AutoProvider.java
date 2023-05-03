package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.BalanceChargePad;

/**
 * Provides the default command for autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final Command superNaiveAuto = new SuperNaiveAuto();
  private final Command bumplessSuperNaiveAuto = new BumplessSuperNaiveAuto();
  private final Command balanceAuto = new BalanceChargePad();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private AutoProvider() {
    chooser.setDefaultOption("Super Naive Auto", superNaiveAuto);
    chooser.addOption("Bumpless Super Naive Auto", bumplessSuperNaiveAuto);
    chooser.addOption("Balance Auto", balanceAuto);
    chooser.addOption("Disable Auto", new InstantCommand());

    SmartDashboard.putData(chooser);
  }

  public static AutoProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new AutoProvider());
    }
    return inst.get();
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}