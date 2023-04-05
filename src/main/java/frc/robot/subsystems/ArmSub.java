package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.constants.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.CurveFit;
import frc.robot.utils.logger.Logger;

public class ArmSub extends SubsystemBase {
  private final MotorController seg1MasterMotor = Constants.arm.SEG1_MASTER_MOTOR_ID.build();
  private final MotorController seg1SlaveMotor = Constants.arm.SEG1_SLAVE_MOTOR_ID.build();

  private final MotorController seg2MasterMotor = Constants.arm.SEG2_MASTER_MOTOR_ID.build();
  private final MotorController seg2SlaveMotor = Constants.arm.SEG2_SLAVE_MOTOR_ID.build();

  private final MotorControllerGroup seg1Motors = new MotorControllerGroup(seg1MasterMotor, seg1SlaveMotor);
  private final MotorControllerGroup seg2Motors = new MotorControllerGroup(seg2MasterMotor, seg2SlaveMotor);

  private final Encoder seg1Encoder = Constants.arm.SEG1_ENCODER.build();
  private final Encoder seg2Encoder = Constants.arm.SEG2_ENCODER.build();

  private final CurveFit seg1Curve = new CurveFit(-0.5, 0.5, 0.1, 0.5, 1);
  private final CurveFit seg2Curve = new CurveFit(-0.5, 0.5, 0.1, 0.5, 1);

  private boolean calibrated = false;

  /**
   * Updates the arm tab in shuffleboard. Call this function regularly.
   * 
   * @param armSeg1Output The speed of the upper arm motors
   * @param armSeg2Output The speed of the fore arm motors
   */
  private void updateShuffleboard(double armSeg1Output, double armSeg2Output) {
    ShuffleControl.armTab.setOutputs(armSeg1Output, armSeg2Output);
    ShuffleControl.armTab.setEncoderAngles(seg1Encoder.getDistance(), seg2Encoder.getDistance());
    ShuffleControl.calibrationTab.setArmCalibrated(calibrated);
  }

  /**
   * Forward kinematics.
   * 
   * @param armSeg1Angle The angle of the upper arm
   * @param armSeg2Angle The angle of the fore arm
   * @return The calculated coordinates of the end of the arm.
   */
  private Pair<Double, Double> forK(double armSeg1Angle, double armSeg2Angle) {
    armSeg2Angle *= -1;
    double l1 = Constants.arm.SEG1_LENGTH;
    double l2 = Constants.arm.SEG2_LENGTH;

    double x1 = l1 * Math.cos(Math.toRadians(armSeg1Angle + 90));
    double y1 = l1 * Math.sin(Math.toRadians(armSeg1Angle + 90));

    double x2 = l2 * Math.cos(Math.toRadians(armSeg2Angle - 90)) + x1;
    double y2 = l2 * Math.sin(Math.toRadians(armSeg2Angle - 90)) + y1;

    return new Pair<Double, Double>(x2, y2);
  }

  /**
   * Forward kinematics. When run without arguments, this uses the arms' encoder
   * values.
   * 
   * @return The calculated coordinates of the end of the arm.
   */
  private Pair<Double, Double> forK() {
    return forK(seg1Encoder.getDistance(), seg2Encoder.getDistance());
  }

  /** @return The angle that the end of the arm makes with the robot horizontal */
  public double getEndAngle() {
    double armSeg2Angle = seg2Encoder.getDistance();
    return armSeg2Angle - 90;
  }

  /**
   * Zeroes the upper and fore arm encoders.
   * Use this method when the upper arm pointing up and the fore arm is pointed
   * down.
   */
  public void calibrate() {
    seg1Encoder.reset();
    seg2Encoder.reset();
    calibrated = true;
    Logger.info("ArmSub : Calibrated!");
  }

  /** Kills the robot if an illegal arm angle is reached. */
  private void killCheck() {
    double armSeg1Angle = seg1Encoder.getDistance();
    double armSeg2Angle = seg2Encoder.getDistance();

    if (Math.abs(armSeg1Angle) > 90 || Math.abs(armSeg2Angle) > 185) {
      throw new IllegalStateException(
          String.format("ArmSub::killCheck : Illegal angles reached, killing robot! (armSeg2: %f, armSeg1: %f)",
              armSeg2Angle, armSeg1Angle));
    }
  }

  /**
   * Inverse cosine rule.
   * 
   * @param a Length of side a
   * @param b Length of side b
   * @param c Length of side c
   * @return The included angle C
   */
  double invCosRule(double a, double b, double c) {
    double C = Math.acos(
        (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2))
            / (2 * a * b));
    return C;
  }

  @Override
  public void periodic() {
    if (!calibrated) {
      // Don't do anything if no calibration has happened.
      updateShuffleboard(0, 0);
      return;
    }

    double seg1Throttle = seg1Curve.fit(OI.copilot.getRawAxis(XboxController.Axis.kLeftX.value));
    double seg2Throttle = seg2Curve.fit(OI.copilot.getRawAxis(XboxController.Axis.kRightY.value));

    double seg1Output = MathUtil.clamp(seg1Throttle, -0.3, 0.3);
    double seg2Output = MathUtil.clamp(seg2Throttle, -0.3, 0.3);

    seg1Motors.set(seg1Output);
    seg2Motors.set(seg2Output);

    updateShuffleboard(seg1Output, seg2Output);
  }
}
