package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.constants.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.CurveFit;

/**
 * The Arcade Teleop command
 */
public class TeleopDriveArcade extends CommandBase {
  private final CurveFit throtFit;
  private final CurveFit steerFit;

  public TeleopDriveArcade() {
    this(Constants.drive.PILOT_SETTINGS);
  }

  public TeleopDriveArcade(double[][] settings) {
    throtFit = CurveFit.throtFromDriveSettings(settings);
    steerFit = CurveFit.steerFromDriveSettings(settings);

    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    double throttle = 0;
    double steering = 0;

    throttle = throtFit.fit(OI.applyAxisDeadband(OI.pilot.getLeftY()));
    // limiting max steering based on throttle
    steering = steerFit.fit(OI.applyAxisDeadband(OI.pilot.getRightX()), throttle);

    // Invert steering when throttle >= 0 to mimic car controls
    // if (throttle > 0) {
    //   steering *= -1;
    // }

    // flips the direction of forward based on controller button
    if (Variables.invertDriveDirection) {
      throttle *= -1;
    }

    ShuffleControl.driveTab.setControlAxis(-OI.pilot.getLeftY(), OI.pilot.getRightX());

    Subsystems.drive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
