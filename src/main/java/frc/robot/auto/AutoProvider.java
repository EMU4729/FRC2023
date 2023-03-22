package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Command for Autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final Command superNaiveAuto = new SuperNaiveAuto();
  private final Command bumplessSuperNaiveAuto = new BumplessSuperNaiveAuto();
  // private final Command naiveAuto = new NaiveAuto();
  // private final Command middleAuto = new MiddleAuto();
  // private final Command middleAutoWithoutPickup = new MiddleAutoWithoutPickup();
  // private final Command leftAuto = new LeftAuto();
  // private final Command rightAuto = new RightAuto();
  // private final Command balanceAuto = new BalanceChargePad();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private AutoProvider() {
    chooser.setDefaultOption("Super Naive Auto", superNaiveAuto);
    chooser.addOption("Bumpless Super Naive Auto", bumplessSuperNaiveAuto);
    // chooser.addOption("Naive Auto", naiveAuto);
    // chooser.addOption("Middle Auto", middleAuto);
    // chooser.addOption("Middle Auto Without Pickup", middleAutoWithoutPickup);
    // chooser.addOption("Left Auto", leftAuto);
    // chooser.addOption("Right Auto", rightAuto);
    // chooser.addOption("Balance Auto", balanceAuto);
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