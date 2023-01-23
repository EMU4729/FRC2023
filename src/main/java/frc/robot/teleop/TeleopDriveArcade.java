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
public class TeleopDriveArcade extends CommandBase {
  private final Variables vars = Variables.getInstance();
  private final Constants cnst = Constants.getInstance();
  private final OI oi = OI.getInstance();

  private final CurveFit throtFit;
  private final CurveFit steerFit;

  private final CurveFit copilotThrotFit;
  private final CurveFit copilotSteerFit;

  public TeleopDriveArcade() {
    this(Variables.getInstance().teleopDriveSettings);
  }

  public TeleopDriveArcade(double[][] settings) {
    throtFit = new CurveFit(settings[0][0], settings[0][1], settings[0][2]);
    steerFit = new CurveFit(settings[1][0], settings[1][1], settings[1][2]).setThrotEffect(settings[1][3]);

    copilotThrotFit = new CurveFit(vars.copilotDriveSettings[0][0], vars.copilotDriveSettings[0][1],
        vars.copilotDriveSettings[0][2]);
    copilotSteerFit = new CurveFit(vars.copilotDriveSettings[1][0], vars.copilotDriveSettings[1][1],
        vars.copilotDriveSettings[1][2]).setThrotEffect(vars.copilotDriveSettings[1][3]);
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttle;
    double steering;

    // If pilot is moving the robot
    if (oi.pilotIsActive()) {
      // take input from the pilot
      throttle = throtFit.fit(oi.applyDeadband(oi.pilot.getLeftY()));
      steering = steerFit.fit(oi.applyDeadband(oi.pilot.getRightX()), throttle);// limiting max steering based on
                                                                                // throttle
    } else {
      // take input from the copilot
      throttle = copilotThrotFit.fit(oi.applyDeadband(oi.copilot.getLeftY()));
      steering = copilotSteerFit.fit(oi.applyDeadband(oi.copilot.getRightX()), throttle);// limiting max steering based
                                                                                         // on throttle
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
