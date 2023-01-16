package frc.robot.OI;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utils.AxisButtonSupplier;

public class XboxCtrl {
  private final Constants cnst = Constants.getInstance();

  public XboxCtrl() {
    this(Constants.getInstance().DEVICE_PORT_XBOX_CONTROLLER_1);
  }

  public XboxCtrl(int portNum) {
    controller = new XboxController(portNum);

    ////////
    start = new JoystickButton(controller, Button.kStart.value);
    back = new JoystickButton(controller, Button.kBack.value);

    lBump = new JoystickButton(controller, Button.kLeftBumper.value);
    rBump = new JoystickButton(controller, Button.kRightBumper.value);

    ltButton = new Trigger(
        new AxisButtonSupplier(controller, Axis.kLeftTrigger.value, cnst.CONTROLLER_TRIGGER_THRESHOLD));
    rtButton = new Trigger(
        new AxisButtonSupplier(controller, Axis.kRightTrigger.value, cnst.CONTROLLER_TRIGGER_THRESHOLD));

    lsButton = new JoystickButton(controller, Button.kLeftStick.value);
    rsButton = new JoystickButton(controller, Button.kRightStick.value);

    a = new JoystickButton(controller, Button.kA.value);
    b = new JoystickButton(controller, Button.kB.value);
    x = new JoystickButton(controller, Button.kX.value);
    y = new JoystickButton(controller, Button.kY.value);

    dPadN = new POVButton(controller, 0);
    dPadNE = new POVButton(controller, 45);
    dPadE = new POVButton(controller, 90);
    dPadSE = new POVButton(controller, 135);
    dPadS = new POVButton(controller, 180);
    dPadSW = new POVButton(controller, 225);
    dPadW = new POVButton(controller, 270);
    dPadNW = new POVButton(controller, 315);
  }

  /////////

  public final XboxController controller;

  public final JoystickButton start;
  public final JoystickButton back;

  public final JoystickButton lBump;
  public final JoystickButton rBump;

  public final Trigger ltButton;
  public final Trigger rtButton;

  public final JoystickButton lsButton;
  public final JoystickButton rsButton;

  public final JoystickButton a;
  public final JoystickButton b;
  public final JoystickButton x;
  public final JoystickButton y;

  public final POVButton dPadN;
  public final POVButton dPadNE;
  public final POVButton dPadE;
  public final POVButton dPadSE;
  public final POVButton dPadS;
  public final POVButton dPadSW;
  public final POVButton dPadW;
  public final POVButton dPadNW;
}
