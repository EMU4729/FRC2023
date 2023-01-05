package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;

/**
 * Command to drive straight with help of the IMU.
 * 
 * Only to be used in autonomous.
 */
public class AutoDriveStraight extends CommandBase {
  private final Constants cnst = Constants.getInstance();

  private double speed;
  private PIDController pid;

  public AutoDriveStraight() {
    addRequirements(Subsystems.drive);
  }

  public void run(double targetAngle, double speed) {
    this.speed = speed;
    this.schedule(true);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // double steering = pid.calculate(Subsystemsystems.drive.getHeading());
    // Subsystemsystems.drive.arcade(speed, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.off();
  }

}
