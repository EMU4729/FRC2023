package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command for Autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final Command auto = new Auto();
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private AutoProvider() {
    chooser.setDefaultOption("Default Auto", auto);

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