package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

/**
 * Naive Auto - Drives forwards until the encoders report 3 metres
 * 
 * @deprecated this isn't working
 */
public class NaiveAuto extends CommandBase {
  protected NaiveAuto() {
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    Subsystems.drive.tank(-0.75, -0.75);
  }

  @Override
  public boolean isFinished() {
    return Subsystems.nav.getAvgEncoderDistance() >= 3;
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.off();
  }
}
