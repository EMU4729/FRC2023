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
public class TeleopDriveArcade extends CommandBase {
  private final Variables vars = Variables.getInstance();
  private final Constants cnst = Constants.getInstance();
  private final OI oi = OI.getInstance();

  private final CurveFit throtFit;
  private final CurveFit steerFit;

  private final CurveFit copilotThrotFit;
  private final CurveFit copilotSteerFit;

  public TeleopDriveArcade() {
    this(Variables.getInstance().DriveSettingsTELEOP);
  }

  public TeleopDriveArcade(double[][] settings) {
    throtFit = new CurveFit(settings[0][0], settings[0][1], settings[0][2]);
    steerFit = new CurveFit(settings[1][0], settings[1][1], settings[1][2]).setThrotEffect(settings[1][3]);

    copilotThrotFit = new CurveFit(vars.DriveSettingsCOPILOT[0][0], vars.DriveSettingsCOPILOT[0][1],
        vars.DriveSettingsCOPILOT[0][2]);
    copilotSteerFit = new CurveFit(vars.DriveSettingsCOPILOT[1][0], vars.DriveSettingsCOPILOT[1][1],
        vars.DriveSettingsCOPILOT[1][2]).setThrotEffect(vars.DriveSettingsCOPILOT[1][3]);
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttle;
    double steering;

    // If pilot isn't moving the robot
    if (Math.abs(oi.pilot.getLeftY()) < 0.05 && Math.abs(oi.pilot.getRightX()) < 0.05) {
      // take input from the copilot
      throttle = copilotThrotFit.fit(MathUtil.applyDeadband(oi.copilot.getLeftY(),
          cnst.CONTROLLER_AXIS_DEADZONE));
      steering = copilotSteerFit.fit(MathUtil.applyDeadband(oi.copilot.getRightX(),
          cnst.CONTROLLER_AXIS_DEADZONE), throttle);// limiting max steering based on throttle
    } else {
      // take input from the pilot
      throttle = throtFit.fit(MathUtil.applyDeadband(oi.pilot.getLeftY(),
          cnst.CONTROLLER_AXIS_DEADZONE));
      steering = steerFit.fit(MathUtil.applyDeadband(oi.pilot.getRightX(),
          cnst.CONTROLLER_AXIS_DEADZONE), throttle);// limiting max steering based on throttle
    }

    // flips the direction of forward based on controller button
    throttle = throttle * (vars.invertDriveDirection ? 1 : -1);

    ShuffleControl.setControlAxis(-oi.pilot.getLeftY(), oi.pilot.getRightX());

    Subsystems.drive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
