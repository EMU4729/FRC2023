package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;

/** Automatically balances the robot on the charge pad using the IMU's gyro. */
public class BalanceChargePad extends CommandBase {
  public final Constants cnst = Constants.getInstance();
  public final PIDController controller = cnst.BALANCE_CHARGE_PAD_PID.createPIDController();

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
    if (Math.abs(pitch) < cnst.BALANCE_CHARGE_PAD_DEADBAND)
      return;

    double speed = controller.calculate(pitch);
    Subsystems.drive.tank(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.off();
  }
}
