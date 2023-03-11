package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class NaiveAuto extends CommandBase {
  protected NaiveAuto() {
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
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
