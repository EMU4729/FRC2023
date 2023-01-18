package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();
  private final MotorController upperArmMotor = cnst.UPPER_ARM_MOTOR_ID.createMotorController();
  private final MotorController foreArmMotor = cnst.FORE_ARM_MOTOR_ID.createMotorController();

  private double[] ik(Pose2d pose) {
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

}
