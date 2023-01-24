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

  private final Constants cnst = Constants.getInstance();

  /**
   * Checks if the pilot is moving the robot.
   * 
   * @return true if the pilot is moving the robot, false otherwise.
   */
  public boolean pilotIsActive() {
    //return true;
    return Math.abs(pilot.getLeftY())  > cnst.CONTROLLER_AXIS_DEADZONE || 
           Math.abs(pilot.getRightY()) > cnst.CONTROLLER_AXIS_DEADZONE ||
           Math.abs(pilot.getRightX()) > cnst.CONTROLLER_AXIS_DEADZONE;
  }

  /**
   * Applies the controller axis deadband to a value
   * 
   * @param value The value to apply the deadband to
   * @return The value with the deadband applied
   */
  public double applyAxisDeadband(double value) {
    return MathUtil.applyDeadband(value, cnst.CONTROLLER_AXIS_DEADZONE);
  }

  public final CommandXboxController pilot = new CommandXboxController(cnst.PILOT_XBOX_CONTROLLER_PORT);
  public final CommandXboxController copilot = new CommandXboxController(cnst.COPILOT_XBOX_CONTROLLER_PORT);
}
