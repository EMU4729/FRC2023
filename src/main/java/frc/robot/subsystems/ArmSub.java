package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final PIDController upperArmController = cnst.UPPER_ARM_PID.createPIDController();
  private final PIDController foreArmController = cnst.FORE_ARM_PID.createPIDController();

  private final MotorController upperArmMasterMotor = cnst.UPPER_ARM_MASTER_MOTOR_ID.createMotorController();
  private final MotorController upperArmSlaveMotor = cnst.UPPER_ARM_SLAVE_MOTOR_ID.createMotorController();

  private final MotorController foreArmMasterMotor = cnst.FORE_ARM_MASTER_MOTOR_ID.createMotorController();
  private final MotorController foreArmSlaveMotor = cnst.FORE_ARM_SLAVE_MOTOR_ID.createMotorController();

  private final MotorControllerGroup upperArmMotors = new MotorControllerGroup(upperArmMasterMotor, upperArmSlaveMotor);
  private final MotorControllerGroup foreArmMotors = new MotorControllerGroup(foreArmMasterMotor, foreArmSlaveMotor);

  private final Encoder upperArmEncoder = cnst.UPPER_ARM_MASTER_MOTOR_ID.createEncoder();
  private final Encoder foreArmEncoder = cnst.FORE_ARM_MASTER_MOTOR_ID.createEncoder();

  private boolean invert = false;
  private boolean calibrated = false;

  private double targetX;
  private double targetY;
  private double upperArmTargetAngle;
  private double foreArmTargetAngle;

  public ArmSub() {
    upperArmController.setTolerance(3);
    foreArmController.setTolerance(3);
  }

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

    double[] res = { alpha + theta - 90, beta };
    return res;
  }

  /** Inverts the arm. */
  public void invert() {
    invert = !invert;
    setCoords(targetX, targetY);
  }

  /**
   * Sets the target angles for the arm
   * 
   * @param upperArm The target angle for the upper arm
   * @param foreArm  The target angle for the fore arm
   */
  private void setAngles(double upperArm, double foreArm) {
    upperArmTargetAngle = MathUtil.clamp(upperArm, 0, 90);
    foreArmTargetAngle = MathUtil.clamp(foreArm, 0, 180);

    if (invert) {
      upperArmTargetAngle *= -1;
      foreArmTargetAngle *= -1;
    }

    upperArmController.setSetpoint(upperArmTargetAngle);
    foreArmController.setSetpoint(foreArmTargetAngle);
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
    setAngles(res[0], res[1]);
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
  /** @return A {@link Command} that moves the arm to the low field position */
  public Command lowField() {
    return moveTo(1, 0.5);
  }

  /** @return A {@link Command} that moves the arm to the far field position */
  public Command farField() {
    return moveTo(2, 0.5);
  }

  /** @return a {@link Command} that moves the arm to the lower rung position */
  public Command lowerRung() {
    return moveTo(1.5, 1);
  }

  /** @return a {@link Command} that moves the arm to the upper rung position */
  public Command upperRung() {
    return moveTo(2, 1.5);
  }

  public Command moveTo(double x, double y) {
    return new FunctionalCommand(
        () -> this.setCoords(x, y),
        () -> {
        },
        (interrupted) -> {
        },
        () -> foreArmController.atSetpoint() && upperArmController.atSetpoint(), this);
  }

  /**
   * Zeroes the upper and fore arm encoders.
   * Use this method when the upper arm pointing up and the fore arm is pointed
   * down.
   */
  public void calibrate() {
    upperArmEncoder.reset();
    foreArmEncoder.reset();
    setAngles(0, 0);
    calibrated = true;
  }

  @Override
  public void periodic() {
    // PRAY TO GOD THAT THIS CODE WORKS.

    if (!calibrated) {
      // Don't do anything if no calibration has happened.
      return;
    }

    double upperArmOutput = upperArmController.calculate(upperArmEncoder.getDistance());
    double foreArmOutput = foreArmController.calculate(foreArmEncoder.getDistance());

    upperArmOutput = MathUtil.clamp(upperArmOutput, -0.2, 0.2);
    foreArmOutput = MathUtil.clamp(foreArmOutput, -0.2, 0.2);

    upperArmMotors.set(upperArmOutput);
    foreArmMotors.set(foreArmOutput);
  }
}
