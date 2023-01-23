package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
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

  public boolean pilotIsActive() {
    return Math.abs(pilot.getLeftY()) > 0.05 && Math.abs(pilot.getRightX()) > 0.05;
  }

  public double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, cnst.CONTROLLER_AXIS_DEADZONE);
  }

  private final Constants cnst = Constants.getInstance();

  public final CommandXboxController pilot = new CommandXboxController(cnst.PILOT_XBOX_CONTROLLER_PORT);
  public final CommandXboxController copilot = new CommandXboxController(cnst.COPILOT_XBOX_CONTROLLER_PORT);
}
