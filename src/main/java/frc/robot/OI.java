package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * OI - Use this class to access and initialize all controller-related stuff.
 */
public class OI {
  private static Optional<OI> inst = Optional.empty();

  public static OI getInstance() {
    if (!inst.isPresent())
      inst = Optional.of(new OI());
    return inst.get();
  }

  private final Constants cnst = Constants.getInstance();

  public final CommandXboxController pilot = new CommandXboxController(cnst.PILOT_XBOX_CONTROLLER_PORT);
  public final CommandXboxController copilot = new CommandXboxController(cnst.COPILOT_XBOX_CONTROLLER_PORT);
}
