package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.logger.Logger;

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

  private List<Pair<Double, Double>> targets = new ArrayList<Pair<Double, Double>>();
  private double upperArmTargetAngle;
  private double foreArmTargetAngle;
  private Pair<Double, Double> prevArmTargetPoints;

  public ArmSub() {
    targets.add(0, new Pair<Double, Double>(0.0, 0.0));
    upperArmController.setTolerance(3);
    foreArmController.setTolerance(3);
  }

  /**
   * Updates the arm tab in shuffleboard. Call this function regularly.
   * 
   * @param upperArmOutput The speed of the upper arm motors
   * @param foreArmOutput  The speed of the fore arm motors
   */
  private void updateShuffleboard(double upperArmOutput, double foreArmOutput) {
    ShuffleControl.armTab.setOutputs(upperArmOutput, foreArmOutput);
    ShuffleControl.armTab.setTargetCoords(getFinalTarget().getFirst(), getFinalTarget().getSecond());
    ShuffleControl.armTab.setTargetAngles(upperArmTargetAngle, foreArmTargetAngle);
    ShuffleControl.armTab.setEncoderAngles(upperArmEncoder.getDistance(), foreArmEncoder.getDistance());
    ShuffleControl.armTab.setControllerErrors(upperArmController.getPositionError(),
        foreArmController.getPositionError());
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
    double xSign = Math.signum(x);
    x = Math.abs(x);

    double L1 = cnst.UPPER_ARM_LENGTH;
    double L2 = cnst.FORE_ARM_LENGTH;

    double r = Math.hypot(x, y);

    double theta = Math.atan2(y, x);
    double alpha = invCosRule(L1, r, L2);
    double beta = invCosRule(L2, L1, r);

    alpha = Math.toDegrees(alpha);
    beta = Math.toDegrees(beta);
    theta = Math.toDegrees(theta);

    double foreArmAngle = xSign * (alpha + theta - 90);
    double upperArmAngle = xSign * beta;

    // Return previous results if coordinates are invalid
    if (Double.isNaN(foreArmAngle) || Double.isNaN(upperArmAngle)) {
      return new double[] { foreArmTargetAngle, upperArmTargetAngle };
    }

    return new double[] { foreArmAngle, upperArmAngle };
  }

  /** Inverts the arm. */
  public void invert() {
    invert = !invert;
    double tmp1 = invert ? cnst.ARM_REACH_EXCLUSION[0][0] : cnst.ARM_REACH_EXCLUSION[0][1];
    double tmp2 = invert ? cnst.ARM_REACH_EXCLUSION[0][1] : cnst.ARM_REACH_EXCLUSION[0][0];
    addCoord(0, tmp1, cnst.ARM_SWING_THROUGH_HEIGHT, false);
    addCoord(1, tmp2, cnst.ARM_SWING_THROUGH_HEIGHT, false);
    Pair<Double, Double> tmp = getCurTarget();
    setDestCoord(-tmp.getFirst(), tmp.getSecond(), true);
  }

  /**
   * Sets the target angles for the arm
   * 
   * @param upperArm The target angle for the upper arm
   * @param foreArm  The target angle for the fore arm
   */
  private void setAngles(double upperArm, double foreArm) {
    if (!calibrated) {
      Logger.warn("ArmSub : Arm hasn't been calibrated yet!");
    }

    upperArmTargetAngle = MathUtil.clamp(upperArm, -90, 90);
    foreArmTargetAngle = MathUtil.clamp(foreArm, -180, 180);

    upperArmController.setSetpoint(upperArmTargetAngle);
    foreArmController.setSetpoint(foreArmTargetAngle);
  }

  /**
   * Sets the target coordinates for the arm.
   * 
   * @param x The target x
   * @param y The target y
   */
  public void setCoord(Pair<Double, Double> coord, double x, double y, boolean invertable) {
    if (!targetIsValid(x, y)) { // if target coord is not allowed stay still
      Logger.warn("ArmSub : setCoords : dest is not allowed");
      Pair<Double, Double> tmp = getCurTarget();
      x = tmp.getFirst();
      y = tmp.getSecond();
    }
    int inv = invert && invertable ? -1 : 1;
    targets.set(targets.indexOf(coord), new Pair<Double, Double>(x * inv, y));
    Pair<Double, Double> tmp = getCurTarget();
    double[] res = ik(tmp.getFirst(), tmp.getSecond());
    setAngles(res[0], res[1]);
  }

  public void setDestCoord(double x, double y, boolean invertable) {
    setCoord(getFinalTarget(), x, y, invertable);
  }

  public void shiftCoord(Pair<Double, Double> coord, double x, double y, boolean invertable) {
    setCoord(coord, coord.getFirst() + x, coord.getSecond() + y, invertable);
  }

  public void shiftDestCoord(double x, double y, boolean invertable) {
    shiftCoord(getFinalTarget(), x, y, invertable);
  }

  public void addCoord(int idx, double x, double y, boolean invertable) {
    if (idx > targets.size()) {
      Logger.warn("ArmSub : addCoord : idx of new too large check code -1 = end");
      idx = targets.size();
    }
    if (idx < 0)
      idx = targets.size();

    if (!targetIsValid(x, y)) { // if target coord is not allowed stay still
      Logger.warn("ArmSub : setCoords : dest is not allowed");
      Pair<Double, Double> tmp = getCurTarget();
      x = tmp.getFirst();
      y = tmp.getSecond();
    }
    int inv = invert && invertable ? -1 : 1;
    targets.add(idx, new Pair<Double, Double>(x * inv, y));
    Pair<Double, Double> tmp = getCurTarget();
    double[] res = ik(tmp.getFirst(), tmp.getSecond());
    setAngles(res[0], res[1]);
  }

  /** Returns a {@link Command} that moves the arm up indefinitely. */
  public Command moveUp() {
    return this.run(() -> {
      shiftDestCoord(0, cnst.ARM_VELOCITY, true);
    });
  }

  /** Returns a {@link Command} that moves the arm down indefinitely. */
  public Command moveDown() {
    return this.run(() -> {
      shiftDestCoord(0, -cnst.ARM_VELOCITY, true);
    });
  }

  /** Returns a {@link Command} that moves the arm forward indefinitely. */
  public Command moveForward() {
    return this.run(() -> {
      shiftDestCoord(cnst.ARM_VELOCITY, 0, true);
    });
  }

  /** Returns a {@link Command} that moves the arm backward indefinitely. */
  public Command moveBack() {
    return this.run(() -> {
      shiftDestCoord(-cnst.ARM_VELOCITY, 0, true);
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
        () -> this.setDestCoord(x, y, true),
        () -> {
        },
        (interrupted) -> {
        },
        () -> {
          if (RobotBase.isSimulation()) {
            Logger.info("ArmSub::moveTo : In simulation, skipping...");
            return true;
          }

          return foreArmController.atSetpoint() && upperArmController.atSetpoint();
        }, this);
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
    Logger.info("ArmSub : Calibrated!");
  }

  @Override
  public void periodic() {
    // PRAY TO GOD THAT THIS CODE WORKS.

    if (!calibrated) {
      // Don't do anything if no calibration has happened.
      updateShuffleboard(0, 0);
      return;
    }

    double upperArmOutput = upperArmController.calculate(upperArmEncoder.getDistance() * -1);
    double foreArmOutput = foreArmController.calculate(foreArmEncoder.getDistance());

    upperArmOutput = MathUtil.clamp(upperArmOutput, -0.2, 0.2);
    foreArmOutput = MathUtil.clamp(foreArmOutput, -0.2, 0.2);

    if (!upperArmController.atSetpoint()) {
      upperArmMotors.set(upperArmOutput);
    } else {
      upperArmMotors.stopMotor();

      if (foreArmController.atSetpoint() && prevArmTargetPoints != getFinalTarget()) { // if at both setpoints
        if (targets.size() > 1)
          targets.remove(0);
        Pair<Double, Double> tmp = getCurTarget();
        double[] res = ik(tmp.getFirst(), tmp.getSecond());
        setAngles(res[0], res[1]);
      }
    }

    if (!foreArmController.atSetpoint()) {
      foreArmMotors.set(foreArmOutput);
    } else {
      foreArmMotors.stopMotor();
    }

    updateShuffleboard(upperArmOutput, foreArmOutput);
  }

  Pair<Double, Double> getFinalTarget() {
    if (targets.isEmpty())
      return new Pair<Double, Double>(0.0, 0.0);
    return targets.get(targets.size() - 1);
  }

  Pair<Double, Double> getCurTarget() {
    if (targets.isEmpty())
      return new Pair<Double, Double>(0.0, 0.0);
    return targets.get(0);
  }

  boolean targetIsValid(double x, double y) {
    if (x < cnst.MAX_ARM_REACH_LEGAL[0][0] || x > cnst.MAX_ARM_REACH_LEGAL[0][1])
      return false;
    if (y < cnst.MAX_ARM_REACH_LEGAL[1][0] || y > cnst.MAX_ARM_REACH_LEGAL[1][1])
      return false;
    // max arm reach is around the axle x is around robot center x is adjusted to be
    // around axle
    // before checking
    if (Math.hypot(x + cnst.UPPER_ARM_X_OFFSET, y) > cnst.MAX_ARM_REACH_PHYSICAL)
      return false;
    return true;
  }

  double invCosRule(double a, double b, double c) {
    double C = Math.acos(
        (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2))
            / (2 * a * b));
    return C;
  }
}
