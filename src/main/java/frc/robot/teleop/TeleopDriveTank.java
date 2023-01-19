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
  private final CurveFit copilotThrotFit;

  int i = 0;

  public TeleopDriveTank() {
    this(Variables.getInstance().DriveSettingsTELEOP);
  }

  public TeleopDriveTank(double[][] settings) {
    throtFit = new CurveFit(settings[0][0], settings[0][1], settings[0][2]);
    copilotThrotFit = new CurveFit(vars.DriveSettingsCOPILOT[0][0], vars.DriveSettingsCOPILOT[0][1],
        vars.DriveSettingsCOPILOT[0][2]);
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttleL;
    double throttleR;

    // if the pilot isn't moving the robot
    if (Math.abs(oi.pilot.getLeftY()) < 0.05 && Math.abs(oi.pilot.getRightY()) < 0.05) {
      // take input from the copilot
      throttleL = copilotThrotFit.fit(MathUtil.applyDeadband(oi.copilot.getLeftY(),
          cnst.CONTROLLER_AXIS_DEADZONE));
      throttleR = copilotThrotFit.fit(MathUtil.applyDeadband(oi.copilot.getRightY(),
          cnst.CONTROLLER_AXIS_DEADZONE));
    } else {
      // take input from the pilot
      throttleL = throtFit.fit(MathUtil.applyDeadband(oi.pilot.getLeftY(),
          cnst.CONTROLLER_AXIS_DEADZONE));
      throttleR = throtFit.fit(MathUtil.applyDeadband(oi.pilot.getRightY(),
          cnst.CONTROLLER_AXIS_DEADZONE));
    }

    // flips the direction of forward based on controller button
    throttleL = throttleL * (vars.invertDriveDirection ? 1 : -1);
    throttleR = throttleR * (vars.invertDriveDirection ? 1 : -1);

    ShuffleControl.setControlAxis(-oi.pilot.getLeftY(), oi.pilot.getRightY());

    Subsystems.drive.tank(throttleL, throttleR);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
