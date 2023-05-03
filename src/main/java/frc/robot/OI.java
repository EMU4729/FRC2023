package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

/**
 * OI - Use this class to access and initialize all controller-related stuff.
 */
public class OI {
  /**
   * Checks if the pilot is moving the robot.
   * 
   * @return true if the pilot is moving the robot, false otherwise.
   */
  public static boolean pilotIsActive() {
    return Math.abs(pilot.getLeftY()) > Constants.controller.CONTROLLER_AXIS_DEADZONE ||
        Math.abs(pilot.getRightY()) > Constants.controller.CONTROLLER_AXIS_DEADZONE ||
        Math.abs(pilot.getRightX()) > Constants.controller.CONTROLLER_AXIS_DEADZONE;
  }

  /**
   * Applies the controller axis deadband to a value
   * 
   * @param value The value to apply the deadband to
   * @return The value with the deadband applied
   */
  public static double applyAxisDeadband(double value) {
    return MathUtil.applyDeadband(value, Constants.controller.CONTROLLER_AXIS_DEADZONE);
  }

  /** The pilot's controller */
  public static final CommandXboxController pilot = new CommandXboxController(
      Constants.controller.PILOT_XBOX_CONTROLLER_PORT);

  /** The copilot's controller */
  public static final CommandXboxController copilot = new CommandXboxController(
      Constants.controller.COPILOT_XBOX_CONTROLLER_PORT);
}
