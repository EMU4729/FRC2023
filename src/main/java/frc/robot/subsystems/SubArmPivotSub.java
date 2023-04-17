package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.constants.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.logger.Logger;

public class SubArmPivotSub extends SubsystemBase {
  private final MotorController motor = Constants.subarm.PIVOT_MOTOR_ID.build();
  private final Encoder encoder = Constants.subarm.PIVOT_ENCODER_ID.build();
  private final PIDController controller = Constants.subarm.PIVOT_PID.build();

  private boolean calibrated = false;
  private final double targetAngle = 0;

  /**
   * Calibrates the subarm.
   * Only run this when the subarm is pointed as high as possible forwards.
   */
  public void calibrate() {
    encoder.reset();
    controller.reset();
    controller.setSetpoint(targetAngle);
    calibrated = true;
    Logger.info("SubArmPivotSub : Calibrated!");
  }

  /**
   * Updates shuffleboard. Call this function regularly.
   * 
   * @param output The pivot motor's output.
   */
  private void updateShuffleboard(double output) {
    ShuffleControl.subArmTab.setOutput(output);
    ShuffleControl.subArmTab.setEncoderAngle(encoder.getDistance());
    ShuffleControl.subArmTab.setControllerError(controller.getPositionError());
    ShuffleControl.calibrationTab.setSubArmPivotCalibrated(calibrated);
  }

  /** @return If a provided angle is within the bounds of the pivot limits. */
  private boolean angleIsValid(double angle) {
    return angle >= Constants.subarm.PIVOT_LOWER_LIMIT && angle <= Constants.subarm.PIVOT_UPPER_LIMIT;
  }

  /**
   * Clamps the angle to the valid range.
   * 
   * @param angle The angle to clamp.
   * @return The clamped angle.
   */
  private double clampAngle(double angle) {
    return MathUtil.clamp(angle, Constants.subarm.PIVOT_LOWER_LIMIT, Constants.subarm.PIVOT_UPPER_LIMIT);
  }

  @Override
  public void periodic() {
    if (!calibrated) {
      updateShuffleboard(0);
      return;
    }

    // This should make the subarm's angle independent of the arm's angle
    double armOffset = -Subsystems.arm.getEndAngle() - 90;

    double currentAngle = encoder.getDistance();
    double output;

    if (!angleIsValid(currentAngle)) {
      output = clampAngle(currentAngle) + armOffset;
    } else {
      output = clampAngle(currentAngle + armOffset);
    }

    output = controller.calculate(output) * -1;

    output = MathUtil.clamp(output, -0.2, 0.2); // This is a safety measure, will be increased to -1 and 1 when stable

    motor.set(output);

    updateShuffleboard(output);
  }
}
