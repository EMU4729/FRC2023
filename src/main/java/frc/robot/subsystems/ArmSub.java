package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final PIDController upperArmController = cnst.UPPER_ARM_PID.createPIDController();
  private final PIDController foreArmController = cnst.FORE_ARM_PID.createPIDController();

  //private final MotorController upperArmMotor = cnst.UPPER_ARM_MOTOR_ID.createMotorController();
  //private final MotorController foreArmMotor = cnst.FORE_ARM_MOTOR_ID.createMotorController();

  //private final Encoder upperArmEncoder = cnst.UPPER_ARM_MOTOR_ID.createEncoder();
  //private final Encoder foreArmEncoder = cnst.FORE_ARM_MOTOR_ID.createEncoder();

  private Translation2d targetCoords = new Translation2d(1., 1.);

  /**
   * Calculates the angles of the two arms from a given pose with
   * inverse kinematics.
   * 
   * @param pose The desired pose
   * @return An array where the 1st element is the angle between the upper arm and
   *         the robot, and the 2nd element is the angle between the fore arm and
   *         the upper arm. All angles are in degrees.
   */
  private double[] ik(Translation2d pose) {
    double a = cnst.UPPER_ARM_LENGTH;
    double b = cnst.FORE_ARM_LENGTH;
    double x = pose.getX();
    double y = pose.getY();

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

  @Override
  public void periodic() {
    // THIS CODE IS UNTESTED. IT CAN CAUSE SOME SERIOUS DAMAGE.
    if (false) {
      double[] ikRes = ik(targetCoords);
      double upperArmTargetAngle = ikRes[0];
      double foreArmTargetAngle = ikRes[1];

      upperArmController.setSetpoint(upperArmTargetAngle);
      foreArmController.setSetpoint(foreArmTargetAngle);

      //double upperArmOutput = upperArmController.calculate(upperArmEncoder.getDistance());
      //double foreArmOutput = foreArmController.calculate(foreArmEncoder.getDistance());

      //upperArmOutput = MathUtil.clamp(upperArmOutput, -1, 1);
      //foreArmOutput = MathUtil.clamp(foreArmOutput, -1, 1);

      //upperArmMotor.set(upperArmOutput);
      //foreArmMotor.set(foreArmOutput);
    }
  }
}
