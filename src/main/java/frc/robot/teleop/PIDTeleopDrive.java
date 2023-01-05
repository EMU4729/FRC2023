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
 * The PID Teleop Command.
 */
public class PIDTeleopDrive extends CommandBase {
  private final Variables vars = Variables.getInstance();
  private final Constants cnst = Constants.getInstance();
  private final OI oi = OI.getInstance();

  private final CurveFit throtFit;
  private final CurveFit steerFit;

  public PIDTeleopDrive() {
    this(Variables.getInstance().DriveSettingsPID1, Variables.getInstance().DriveSettingsPID2);
  }

  public PIDTeleopDrive(double[][] settings1, double[][] settings2) {
    throtFit = new CurveFit(settings1[0][0], settings1[0][1], settings1[0][2]);
    steerFit = new CurveFit(settings1[1][0], settings1[1][1], settings1[1][2]).setThrotEffect(settings1[1][3]);
    Subsystems.drive.pidArcadeSetup(settings2);
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double speed = throtFit.fit(MathUtil.applyDeadband(oi.controller.getLeftY(), cnst.CONTROLLER_AXIS_DEADZONE));
    double steering = steerFit.fit(MathUtil.applyDeadband(oi.controller.getRightX(), cnst.CONTROLLER_AXIS_DEADZONE),
        speed / throtFit.outAbsMax);
    int reversalMultiplier = vars.invertDriveDirection ? 1 : -1;

    ShuffleControl.setControlAxis(-oi.controller.getLeftY(), oi.controller.getRightX());
    Subsystems.drive.pidArcade(speed * reversalMultiplier, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
