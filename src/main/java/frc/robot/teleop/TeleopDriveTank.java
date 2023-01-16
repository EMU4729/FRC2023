package frc.robot.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.ShuffleControl.ShuffleControl;
import frc.robot.utils.CurveFit;

/**
 * The Teleop Command.
 */
public class TeleopDriveTank extends CommandBase {
  private final Variables vars = Variables.getInstance();
  private final Constants cnst = Constants.getInstance();
  private final OI oi = OI.getInstance();

  private final CurveFit throtFit;

  int i = 0;

  public TeleopDriveTank() {
    this(Variables.getInstance().DriveSettingsTELEOP);
  }

  public TeleopDriveTank(double[][] settings) {
    throtFit = new CurveFit(settings[0][0], settings[0][1], settings[0][2]);
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttleL = throtFit.fit(MathUtil.applyDeadband(oi.xBox1.getLeftY(),
        cnst.CONTROLLER_AXIS_DEADZONE));
    double throttleR = throtFit.fit(MathUtil.applyDeadband(oi.xBox1.getRightY(),
        cnst.CONTROLLER_AXIS_DEADZONE));

    // flips the direction of forward based on controller button
    throttleL = throttleL * (vars.invertDriveDirection ? 1 : -1);
    throttleR = throttleR * (vars.invertDriveDirection ? 1 : -1);

    ShuffleControl.setControlAxis(-oi.xBox1.getLeftY(), oi.xBox1.getRightY());
    ShuffleControl.setThrotGraph(-oi.xBox1.getLeftY(), throttleL);
    ShuffleControl.setSteerGraph(oi.xBox1.getRightY(), throttleR);

    Subsystems.drive.tank(throttleL, throttleR);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
