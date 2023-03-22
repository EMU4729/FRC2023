package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

/** Automatically balances the robot on the charge pad using the IMU's gyro. */
public class BalanceChargePad extends CommandBase {
  private final double DEADBAND = 2;
  private final PIDController controller = new PIDController(0.1, 0, 0); // UPDATE

  public BalanceChargePad() {
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
    controller.setSetpoint(0);
  }

  @Override
  public void execute() {
    double pitch = Subsystems.nav.getPitch();

    // don't move if the angle is correct
    if (Math.abs(pitch) < DEADBAND)
      return;

    double speed = controller.calculate(pitch);
    Subsystems.drive.tank(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.off();
  }
}
