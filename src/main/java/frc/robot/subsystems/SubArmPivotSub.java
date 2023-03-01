package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.logger.Logger;

public class SubArmPivotSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final MotorController motor = cnst.SUBARM_PIVOT_MOTOR_ID.createMotorController();
  private final Encoder encoder = cnst.SUBARM_PIVOT_MOTOR_ID.createEncoder();
  private final PIDController controller = cnst.SUBARM_PIVOT_PID.createPIDController();

  private boolean calibrated = false;
  private double targetAngle = 0;

  /**
   * Calibrates the subarm.
   * Only run this when the subarm is pointed as low as possible.
   */
  public void calibrate() {
    encoder.reset();
    calibrated = true;
    Logger.info("SubArmPivotSub : Calibrated!");
  }

  /** Warns user if uncalibrated */
  private void calibrationCheck() {
    if (!calibrated) {
      Logger.warn("SubArmPivotSub : Uncalibrated! Be careful!");
    }
  }

  private void updateShuffleboard(double output) {
    ShuffleControl.subArmTab.setOutput(output);
    ShuffleControl.subArmTab.setEncoderAngle(encoder.getDistance());
    ShuffleControl.subArmTab.setTargetAngle(targetAngle);
    ShuffleControl.subArmTab.setControllerError(controller.getPositionError());
  }

  private boolean angleIsValid(double angle) {
    return angle >= cnst.SUBARM_PIVOT_LOWER_LIMIT && angle <= cnst.SUBARM_PIVOT_UPPER_LIMIT;
  }

  private void setAngle(double angle) {
    calibrationCheck();
    if (!angleIsValid(angle)) {
      Logger.warn("SubArmRotateSub::setAngle : Invalid angle " + angle);
      return;
    }
    targetAngle = angle;
  }

  private void shiftAngle(double angle) {
    setAngle(targetAngle + angle);
  }

  private Command turnTo(double angle) {
    return new FunctionalCommand(
        () -> setAngle(angle),
        () -> {
        },
        (interrupted) -> {
        },
        () -> {
          if (RobotBase.isSimulation()) {
            Logger.info("SubArmPivotSub::turnTo : In simulation, skipping...");
            return true;
          }

          return controller.atSetpoint();
        }, this);
  }

  /** @return a {@link Command} to pivot the subarm upwards */
  public Command moveUp() {
    return this.run(() -> shiftAngle(cnst.SUBARM_PIVOT_VELOCITY));
  }

  /** @return a {@link Command} to pivot the subarm downwards */
  public Command moveDown() {
    return this.run(() -> shiftAngle(-cnst.SUBARM_PIVOT_VELOCITY));
  }

  public Command pointForward() {
    return turnTo(90);
  }

  public Command pointDown() {
    return turnTo(0);
  }

  @Override
  public void periodic() {
    // DO NOT USE THIS CODE IT WILL BREAK SUBARM AND I DONT WANT TO MAKE KEITH SAD

    if (true) {
      return;
    }

    if (!calibrated) {
      return;
    }

    double armOffset = 0; // TODO get the arm angle offset

    double output = controller.calculate(encoder.getDistance() + armOffset);

    output = MathUtil.clamp(output, -0.5, 0.5); // This is a safety measure

    motor.set(output);

    updateShuffleboard(output);
  }
}
