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
    throtFit = CurveFit.throtFromDriveSettings(settings);
    steerFit = CurveFit.steerFromDriveSettings(settings);

    copilotThrotFit = CurveFit.throtFromDriveSettings(vars.copilotDriveSettings);
    copilotSteerFit = CurveFit.steerFromDriveSettings(vars.copilotDriveSettings);

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
    throttle *= vars.invertDriveDirection ? 1 : -1;

    ShuffleControl.setControlAxis(-oi.pilot.getLeftY(), oi.pilot.getRightX());

    Subsystems.drive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
