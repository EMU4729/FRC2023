package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;

public class BalanceChargePad extends CommandBase {
  public final Constants cnst = Constants.getInstance();
  public final PIDController controller = cnst.BALANCE_CHARGE_PAD_PID.createPIDController();

  public BalanceChargePad() {
    addRequirements(Subsystems.nav, Subsystems.drive);
  }

  @Override
  public void initialize() {
    controller.setSetpoint(0);
  }

  @Override
  public void execute() {
    double roll = Subsystems.nav.getRoll();

    // don't move if the angle is correct
    if (Math.abs(roll) < cnst.BALANCE_CHARGE_PAD_DEADBAND)
      return;

    double speed = controller.calculate(roll);
    Subsystems.drive.tank(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.off();
  }
}
