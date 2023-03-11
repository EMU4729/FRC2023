package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class NaiveAuto extends CommandBase {
  private final Timer timer = new Timer();

  protected NaiveAuto() {
    timer.stop();
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
    Subsystems.drive.tank(-.75, -.75);
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.off();
  }
}
