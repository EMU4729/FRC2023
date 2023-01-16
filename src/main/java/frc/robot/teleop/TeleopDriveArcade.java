package frc.robot.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.OI.OICtrl;
import frc.robot.ShuffleControl.ShuffleControl;
import frc.robot.Constants;
import frc.robot.utils.CurveFit;

/**
 * The Teleop Command.
 */
public class TeleopDriveArcade extends CommandBase {
  private final Variables vars = Variables.getInstance();
  private final Constants cnst = Constants.getInstance();
  private final OICtrl oi = OICtrl.getInstance();

  private final CurveFit throtFit;
  private final CurveFit steerFit;

  int i = 0;

  public TeleopDriveArcade() {
    this(Variables.getInstance().DriveSettingsTELEOP);
  }

  public TeleopDriveArcade(double[][] settings) {
    throtFit = new CurveFit(settings[0][0], settings[0][1], settings[0][2]);
    steerFit = new CurveFit(settings[1][0], settings[1][1], settings[1][2]).setThrotEffect(settings[1][3]);
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttle = throtFit.fit(MathUtil.applyDeadband(oi.xBox1.controller.getLeftY(), 
        cnst.CONTROLLER_AXIS_DEADZONE));
    double steering = steerFit.fit(MathUtil.applyDeadband(oi.xBox1.controller.getRightX(), 
        cnst.CONTROLLER_AXIS_DEADZONE), throttle);// limiting max steering based on throttle

    //flips the direction of forward based on controller button
    throttle = throttle * (vars.invertDriveDirection ? 1 : -1);

    ShuffleControl.setControlAxis(-oi.xBox1.controller.getLeftY(), oi.xBox1.controller.getRightX());
    ShuffleControl.setThrotGraph(-oi.xBox1.controller.getLeftY(), throttle);
    ShuffleControl.setSteerGraph(oi.xBox1.controller.getRightX(), steering);

    Subsystems.drive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
