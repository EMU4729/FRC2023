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
import frc.robot.Subsystems;
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

  /**
   * Updates shuffleboard. Call this function regularly.
   * 
   * @param output The pivot motor's output.
   */
  private void updateShuffleboard(double output) {
    ShuffleControl.subArmTab.setOutput(output);
    ShuffleControl.subArmTab.setEncoderAngle(encoder.getDistance());
    ShuffleControl.subArmTab.setTargetAngle(targetAngle);
    ShuffleControl.subArmTab.setControllerError(controller.getPositionError());
    ShuffleControl.calibrationTab.setSubArmPivotCalibrated(calibrated);
  }

  /** @return If a provided angle is within the bounds of the pivot limits. */
  private boolean angleIsValid(double angle) {
    return angle >= cnst.SUBARM_PIVOT_LOWER_LIMIT && angle <= cnst.SUBARM_PIVOT_UPPER_LIMIT;
  }

  /**
   * Sets the destination angle of the subarm pivot.
   * 
   * @param angle The desired angle
   */
  private void setAngle(double angle) {
    calibrationCheck();
    if (!angleIsValid(angle)) {
      Logger.warn("SubArmRotateSub::setAngle : Invalid angle " + angle);
      return;
    }
    targetAngle = angle;
  }

  /**
   * Shifts the destination angle by a specified angle.
   * 
   * @param angle The amount to shift the destination by.
   */
  private void shiftAngle(double angle) {
    setAngle(targetAngle + angle);
  }

  /** @return a {@link Command} that pivots the subarm to a specified angle. */
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

  /** @return a {@link Command} to move the subarm to the forward position. */
  public Command pointForward() {
    return turnTo(90);
  }

  /** @return a {@link Command} to move the subarm to the downwards position. */
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
      updateShuffleboard(0);
      return;
    }

    // This should make the subarm's angle independent of the arm's angle
    double armOffset = -Subsystems.arm.getEndAngle();

    double output = controller.calculate(encoder.getDistance() + armOffset);

    output = MathUtil.clamp(output, -0.2, 0.2); // This is a safety measure, will be increased to -1 and 1 when stable

    motor.set(output);

    updateShuffleboard(output);
  }
}
