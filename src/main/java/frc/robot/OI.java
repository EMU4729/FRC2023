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

  public final CommandXboxController xBox1 = new CommandXboxController(cnst.DEVICE_PORT_XBOX_CONTROLLER_1);
}
