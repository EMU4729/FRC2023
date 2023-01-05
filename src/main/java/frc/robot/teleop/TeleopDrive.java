package frc.robot.teleop;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.ShuffleControl.ShuffleControl;
import frc.robot.Constants;
import frc.robot.utils.CurveFit;

/**
 * The Teleop Command.
 */
public class TeleopDrive extends CommandBase {
  private final Variables vars = Variables.getInstance();
  private final Constants cnst = Constants.getInstance();
  private final OI oi = OI.getInstance();

  private final CurveFit throtFit;
  private final CurveFit steerFit;

  private Optional<ShuffleControl> shuffle = Optional.empty();
  private boolean accel = false;
  private double lastThrot = 0;
  int i = 0;

  public TeleopDrive() {
    this(Variables.getInstance().DriveSettingsTELEOP);
  }

  public TeleopDrive(double[][] settings) {
    throtFit = new CurveFit(settings[0][0], settings[0][1], settings[0][2]);
    steerFit = new CurveFit(settings[1][0], settings[1][1], settings[1][2]).setThrotEffect(settings[1][3]);
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttle = throtFit.fit(MathUtil.applyDeadband(oi.controller.getLeftY(), cnst.CONTROLLER_AXIS_DEADZONE));
    double steering = steerFit.fit(MathUtil.applyDeadband(oi.controller.getRightX(), cnst.CONTROLLER_AXIS_DEADZONE),
        throttle);// limiting max steering based on throttle

    if (i % 100 == 0) {
      i = 0;
      // Logger.info("throt stick : " + oi.controller.getLeftY() + " throt : " + throttle + " steer stick : "
          // + oi.controller.getRightX() + "steer : " + steering);
    } else {
      i++;
    }

    throttle = throttle * (vars.invertDriveDirection ? 1 : -1); // flips the direction of forward based on controller
                                                                // button

    lastThrot += Math.copySign(vars.accelInterval, (throttle - lastThrot));
    if (accel) {
      throttle = lastThrot;
    }

    ShuffleControl.setControlAxis(-oi.controller.getLeftY(), oi.controller.getRightX());
    ShuffleControl.setThrotGraph(-oi.controller.getLeftY(), throttle);
    ShuffleControl.setSteerGraph(oi.controller.getRightX(), steering);

    Subsystems.drive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
