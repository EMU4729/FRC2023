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
public class TeleopDriveArcade extends CommandBase {
  private final Variables vars = Variables.getInstance();
  private final OI oi = OI.getInstance();

  private final CurveFit throtFit;
  private final CurveFit steerFit;

  public TeleopDriveArcade() {
    this(Variables.getInstance().pilotDriveSettings);
  }

  public TeleopDriveArcade(double[][] settings) {
    throtFit = CurveFit.throtFromDriveSettings(settings);
    steerFit = CurveFit.steerFromDriveSettings(settings);

    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttle = 0;
    double steering = 0;

    throttle = throtFit.fit(oi.applyAxisDeadband(oi.pilot.getLeftY()));
    steering = steerFit.fit(oi.applyAxisDeadband(oi.pilot.getRightX()), throttle);// limiting max steering based on

    // Invert steering when throttle >= 0 to mimic car controls
    if (throttle >= 0) {
      steering *= -1;
    }

    // flips the direction of forward based on controller button
    if (vars.invertDriveDirection) {
      throttle *= -1;
    }

    ShuffleControl.driveTab.setControlAxis(-oi.pilot.getLeftY(), oi.pilot.getRightX());

    Subsystems.drive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
