package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.constants.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.CurveFit;

/**
 * The Drive Teleop Command.
 */
public class TeleopDriveTank extends CommandBase {
  private final CurveFit throtFit;
  private final CurveFit copilotThrotFit;

  public TeleopDriveTank() {
    this(Constants.drive.PILOT_SETTINGS);
  }

  public TeleopDriveTank(double[][] settings) {
    throtFit = CurveFit.throtFromDriveSettings(settings);
    copilotThrotFit = CurveFit.throtFromDriveSettings(Constants.drive.COPILOT_SETTINGS);

    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    double throttleL = 0;
    double throttleR = 0;

    // if the pilot is moving the robot
    if (OI.pilotIsActive()) {
      // take input from the pilot
      throttleL = throtFit.fit(OI.applyAxisDeadband(OI.pilot.getLeftY()));
      throttleR = throtFit.fit(OI.applyAxisDeadband(OI.pilot.getRightY()));
    } else {
      // take input from the copilot
      throttleL = copilotThrotFit.fit(OI.applyAxisDeadband(OI.copilot.getLeftY()));
      throttleR = copilotThrotFit.fit(OI.applyAxisDeadband(OI.copilot.getRightY()));
    }

    // flips the direction of forward based on controller button
    if (Variables.invertDriveDirection) {
      throttleL *= -1;
      throttleR *= -1;
    }

    ShuffleControl.driveTab.setControlAxis(-OI.pilot.getLeftY(), OI.pilot.getRightY());

    Subsystems.drive.tank(throttleL, throttleR);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
