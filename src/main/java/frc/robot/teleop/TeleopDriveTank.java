package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.shufflecontrol.ShuffleControl;
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

  public TeleopDriveTank() {
    this(Variables.getInstance().teleopDriveSettings);
  }

  public TeleopDriveTank(double[][] settings) {
    throtFit = CurveFit.throtFromDriveSettings(settings);
    copilotThrotFit = CurveFit.throtFromDriveSettings(vars.copilotDriveSettings);

    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttleL;
    double throttleR;

    // if the pilot is moving the robot
    if (oi.pilotIsActive()) {
      // take input from the pilot
      throttleL = throtFit.fit(oi.applyDeadband(oi.pilot.getLeftY()));
      throttleR = throtFit.fit(oi.applyDeadband(oi.pilot.getRightY()));
    } else {
      // take input from the copilot
      throttleL = copilotThrotFit.fit(oi.applyDeadband(oi.copilot.getLeftY()));
      throttleR = copilotThrotFit.fit(oi.applyDeadband(oi.copilot.getRightY()));
    }

    // flips the direction of forward based on controller button
    throttleL *= vars.invertDriveDirection ? 1 : -1;
    throttleR *= vars.invertDriveDirection ? 1 : -1;

    ShuffleControl.setControlAxis(-oi.pilot.getLeftY(), oi.pilot.getRightY());

    Subsystems.drive.tank(throttleL, throttleR);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
