package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private final OI oi = OI.getInstance();

  private final CurveFit throtFit;
  private final CurveFit copilotThrotFit;

  public TeleopDriveTank() {
    this(Variables.getInstance().pilotDriveSettings);
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
    double throttleL = 0;
    double throttleR = 0;

    // if the pilot is moving the robot
    if (oi.pilotIsActive()) {
      // take input from the pilot
      throttleL = throtFit.fit(oi.applyAxisDeadband(oi.pilot.getLeftY()));
      throttleR = throtFit.fit(oi.applyAxisDeadband(oi.pilot.getRightY()));
    } else {
      // take input from the copilot
      throttleL = copilotThrotFit.fit(oi.applyAxisDeadband(oi.copilot.getLeftY()));
      throttleR = copilotThrotFit.fit(oi.applyAxisDeadband(oi.copilot.getRightY()));
    }

    // flips the direction of forward based on controller button
    if (vars.invertDriveDirection) {
      throttleL *= -1;
      throttleR *= -1;
    }

    ShuffleControl.driveTab.setControlAxis(-oi.pilot.getLeftY(), oi.pilot.getRightY());

    Subsystems.drive.tank(throttleL, throttleR);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
