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

  private final Command auto = new DefaultAuto();
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private AutoProvider() {
    chooser.setDefaultOption("Default Auto", auto);
    chooser.addOption("Disable Auto", new InstantCommand());
    SmartDashboard.putData(chooser);
  }

  public static AutoProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new AutoProvider());
    }
    return inst.get();
  }

  public Command getAuto() {
    return chooser.getSelected();
  }
}