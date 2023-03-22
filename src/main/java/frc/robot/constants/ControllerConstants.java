package frc.robot.constants;

public class ControllerConstants {
  protected ControllerConstants() {
  }

  /** Port Number for Pilot Xbox Controller */
  public final int PILOT_XBOX_CONTROLLER_PORT = 0; // WORKING
  /** Port Number for Copilot Xbox Controller */
  public final int COPILOT_XBOX_CONTROLLER_PORT = 1; // TEST
  /** Threshold for triggering the controller right and left triggers */
  public final double CONTROLLER_TRIGGER_THRESHOLD = 0.5;
  /** deadband for controller axies either side of 0 */
  public final double CONTROLLER_AXIS_DEADZONE = 0.1;
}
