package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Provides the default command for autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private AutoProvider() {
    chooser.setDefaultOption("Super Naive Auto", new SuperNaiveAuto());
    chooser.addOption("Bumpless Super Naive Auto", new BumplessSuperNaiveAuto());
    chooser.addOption("Balance Auto", new BalanceAuto());
    chooser.addOption("Naive Auto", new NaiveAuto());
    chooser.addOption("PathWeaver Test", new PathWeaverCommand("paths/Sample.wpilib.json"));
    chooser.addOption("Disable Auto", new InstantCommand());

    SmartDashboard.putData("Auto Chooser", chooser);
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