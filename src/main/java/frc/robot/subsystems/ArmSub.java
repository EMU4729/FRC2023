package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final PIDController upperArmController = cnst.UPPER_ARM_PID.createPIDController();
  private final PIDController foreArmController = cnst.FORE_ARM_PID.createPIDController();

  private final MotorController upperArmMotor = cnst.UPPER_ARM_MOTOR_ID.createMotorController();
  private final MotorController foreArmMotor = cnst.FORE_ARM_MOTOR_ID.createMotorController();

  private final Encoder upperArmEncoder = cnst.UPPER_ARM_MOTOR_ID.createEncoder();
  private final Encoder foreArmEncoder = cnst.FORE_ARM_MOTOR_ID.createEncoder();

  private boolean invert = false;

  private double targetX;
  private double targetY;
  private double upperArmTargetAngle;
  private double foreArmTargetAngle;

  /**
   * Calculates the angles of the two arms from a given pose with
   * inverse kinematics.
   * 
   * @param pose The desired pose
   * @return An array where the 1st element is the angle between the upper arm and
   *         the robot, and the 2nd element is the angle between the fore arm and
   *         the upper arm. All angles are in degrees.
   */
  private double[] ik(double x, double y) {
    double a = cnst.UPPER_ARM_LENGTH;
    double b = cnst.FORE_ARM_LENGTH;

    double r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    double theta = Math.atan(y / x);
    double alpha = Math.acos(
        (Math.pow(a, 2) + Math.pow(r, 2) - Math.pow(b, 2))
            / (2 * a * r));
    double beta = Math.asin(
        (r * Math.sin(alpha)) / b);

    alpha = Math.toDegrees(alpha);
    beta = Math.toDegrees(beta);
    theta = Math.toDegrees(theta);

    double[] res = { alpha + theta, beta };
    return res;
  }

  /** Inverts the arm. */
  public void invert() {
    invert = !invert;
    setCoords(targetX, targetY);
  }

  /**
   * Sets the target coordinates for the arm.
   * 
   * @param x The target x
   * @param y The target y
   */
  public void setCoords(double x, double y) {
    targetX = x;
    targetY = y;
    double[] res = ik(targetX, targetY);
    upperArmTargetAngle = MathUtil.clamp(res[0], 0, 90);
    foreArmTargetAngle = MathUtil.clamp(res[1], 0, 180);

    if (invert) {
      upperArmTargetAngle *= -1;
      foreArmTargetAngle *= -1;
    }

    upperArmController.setSetpoint(upperArmTargetAngle);
    foreArmController.setSetpoint(foreArmTargetAngle);
  }

  /** Returns a {@link Command} that moves the arm up indefinitely. */
  public Command moveUp() {
    return this.run(() -> {
      setCoords(targetX, targetY + cnst.ARM_VELOCITY);
    });
  }

  /** Returns a {@link Command} that moves the arm down indefinitely. */
  public Command moveDown() {
    return this.run(() -> {
      setCoords(targetX, targetY - cnst.ARM_VELOCITY);
    });
  }

  /** Returns a {@link Command} that moves the arm forward indefinitely. */
  public Command moveForward() {
    return this.run(() -> {
      setCoords(targetX + cnst.ARM_VELOCITY, targetY);
    });
  }

  /** Returns a {@link Command} that moves the arm backward indefinitely. */
  public Command moveBack() {
    return this.run(() -> {
      setCoords(targetX - cnst.ARM_VELOCITY, targetY);
    });
  }

  // TODO: Update the coordinates on all the preconfigured methods
  /** Move the arm to the low field position */
  public void lowField() {
    setCoords(1, 1);
  }

  /** Move the arm to the far field position */
  public void farField() {
    setCoords(1, 1);
  }

  /** Move the arm to the lower rung position */
  public void lowerRung() {
    setCoords(1, 1);
  }

  /** Move the arm to the upper rung position */
  public void upperRung() {
    setCoords(1, 1);
  }

  private double getForeArmAngle() {
    // No offset needed as the angle between it and the upper arm will be 0 at
    // calibration.
    return foreArmEncoder.getDistance();
  }

  private double getUpperArmAngle() {
    // Adds 90 degrees to offset the calibration value.
    return upperArmEncoder.getDistance() + 90;
  }

  /**
   * Zeroes the upper and fore arm encoders.
   * Use this method when the upper arm pointing up and the fore arm is pointed
   * down.
   */
  public void calibrate() {
    upperArmEncoder.reset();
    foreArmEncoder.reset();
  }

  @Override
  public void periodic() {
    // THIS CODE IS UNTESTED. IT CAN CAUSE SOME SERIOUS DAMAGE.
    if (false) {
      double upperArmOutput = upperArmController.calculate(getUpperArmAngle());
      double foreArmOutput = foreArmController.calculate(getForeArmAngle());

      upperArmOutput = MathUtil.clamp(upperArmOutput, -1, 1);
      foreArmOutput = MathUtil.clamp(foreArmOutput, -1, 1);

      upperArmMotor.set(upperArmOutput);
      foreArmMotor.set(foreArmOutput);
    }
  }
}
