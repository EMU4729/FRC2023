package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.utils.AxisButton;

/**
 * OI - Use this class to access and initialize all controller-related stuff.
 */
public class OI {
  private static Optional<OI> inst = Optional.empty();
  private final Constants cnst = Constants.getInstance();

  public static OI getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new OI());
    }
    return inst.get();
  }

  public final XboxController controller = new XboxController(cnst.DEVICE_PORT_XBOX_CONTROLLER);

  public final JoystickButton start = new JoystickButton(controller,
      Button.kStart.value);
  public final JoystickButton back = new JoystickButton(controller, Button.kBack.value);

  public final JoystickButton lb = new JoystickButton(controller,
      Button.kLeftBumper.value);
  public final JoystickButton rb = new JoystickButton(controller,
      Button.kRightBumper.value);

  public final AxisButton rtButton = new AxisButton(controller, Axis.kRightTrigger.value,
      cnst.CONTROLLER_TRIGGER_THRESHOLD);
  public final AxisButton ltButton = new AxisButton(controller, Axis.kLeftTrigger.value,
      cnst.CONTROLLER_TRIGGER_THRESHOLD);

  public final JoystickButton lsButton = new JoystickButton(controller, Button.kLeftStick.value);
  public final JoystickButton rsButton = new JoystickButton(controller, Button.kRightStick.value);

  public final JoystickButton a = new JoystickButton(controller, Button.kA.value);
  public final JoystickButton b = new JoystickButton(controller, Button.kB.value);
  public final JoystickButton x = new JoystickButton(controller, Button.kX.value);
  public final JoystickButton y = new JoystickButton(controller, Button.kY.value);

  public final POVButton dPadN = new POVButton(controller, 0);
  public final POVButton dPadNE = new POVButton(controller, 45);
  public final POVButton dPadE = new POVButton(controller, 90);
  public final POVButton dPadSE = new POVButton(controller, 135);
  public final POVButton dPadS = new POVButton(controller, 180);
  public final POVButton dPadSW = new POVButton(controller, 225);
  public final POVButton dPadW = new POVButton(controller, 270);
  public final POVButton dPadNW = new POVButton(controller, 315);

}
