package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubArmPivotSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final MotorController motor = cnst.SUBARM_ROTATE_MOTOR_ID.createMotorController();
  private final Encoder encoder = cnst.SUBARM_ROTATE_MOTOR_ID.createEncoder();

  /**
   * Calibrates the subarm.
   * Only run this when the subarm is pointed as low as possible.
   */
  public void calibrate() {
    encoder.reset();
  }

  /** @return a {@link Command} to pivot the subarm upwards */
  public Command up() {
    return new FunctionalCommand(
        () -> motor.set(0.1),
        () -> {
        },
        (interrupted) -> motor.stopMotor(),
        () -> encoder.getDistance() >= cnst.SUBARM_PIVOT_UPPER_LIMIT,
        this);
  }

  /** @return a {@link Command} to pivot the subarm downwards */
  public Command down() {
    return new FunctionalCommand(
        () -> motor.set(-0.1),
        () -> {
        },
        (interrupted) -> motor.stopMotor(),
        () -> encoder.getDistance() <= cnst.SUBARM_PIVOT_LOWER_LIMIT,
        this);
  }
}
