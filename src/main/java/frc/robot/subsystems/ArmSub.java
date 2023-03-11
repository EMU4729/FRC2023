package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.CurveFit;
import frc.robot.utils.logger.Logger;

public class ArmSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final PIDController armSeg1Controller = cnst.ARM_SEG1_PID.createPIDController();
  private final PIDController ArmSeg2Controller = cnst.ARM_SEG2_PID.createPIDController();

  private final MotorController armSeg1MasterMotor = cnst.ARM_SEG1_MASTER_MOTOR_ID.createMotorController();
  private final MotorController armSeg1SlaveMotor = cnst.ARM_SEG1_SLAVE_MOTOR_ID.createMotorController();

  private final MotorController armSeg2MasterMotor = cnst.ARM_SEG2_MASTER_MOTOR_ID.createMotorController();
  private final MotorController armSeg2SlaveMotor = cnst.ARM_SEG2_SLAVE_MOTOR_ID.createMotorController();

  private final MotorControllerGroup armSeg1Motors = new MotorControllerGroup(armSeg1MasterMotor, armSeg1SlaveMotor);
  private final MotorControllerGroup armSeg2Motors = new MotorControllerGroup(armSeg2MasterMotor, armSeg2SlaveMotor);

  private final Encoder armSeg1Encoder = cnst.ARM_SEG1_MASTER_MOTOR_ID.createEncoder();
  private final Encoder armSeg2Encoder = cnst.ARM_SEG2_MASTER_MOTOR_ID.createEncoder();

  private boolean invert = false;
  private boolean calibrated = false;

  private CurveFit seg1Curve = new CurveFit(-0.5, 0.5, 0.1, 0.5, 1);
  private CurveFit seg2Curve = new CurveFit(-0.5, 0.5, 0.1, 0.5, 1);

  private List<Pair<Double, Double>> targets = new ArrayList<Pair<Double, Double>>();
  private double armSeg1TargetAngle;
  private double armSeg2TargetAngle;
  private Pair<Double, Double> prevArmTargetPoints;

  public ArmSub() {
    targets.add(0, new Pair<Double, Double>(0.0, 0.0));
    armSeg1Controller.setTolerance(0);
    ArmSeg2Controller.setTolerance(0);
  }

  /**
   * Updates the arm tab in shuffleboard. Call this function regularly.
   * 
   * @param armSeg1Output The speed of the upper arm motors
   * @param armSeg2Output  The speed of the fore arm motors
   */
  private void updateShuffleboard(double armSeg1Output, double armSeg2Output) {
    ShuffleControl.armTab.setOutputs(armSeg1Output, armSeg2Output);
    ShuffleControl.armTab.setTargetCoords(getFinalTarget().getFirst(), getFinalTarget().getSecond());
    ShuffleControl.armTab.setTargetAngles(armSeg1TargetAngle, armSeg2TargetAngle);
    ShuffleControl.armTab.setEncoderAngles(armSeg1Encoder.getDistance(), armSeg2Encoder.getDistance());
    ShuffleControl.armTab.setControllerErrors(armSeg1Controller.getPositionError(),
        ArmSeg2Controller.getPositionError());
    ShuffleControl.calibrationTab.setArmCalibrated(calibrated);
  }

  /**
   * Forward kinematics.
   * 
   * @param armSeg1Angle The angle of the upper arm
   * @param armSeg2Angle  The angle of the fore arm
   * @return The calculated coordinates of the end of the arm.
   */
  private Pair<Double, Double> forK(double armSeg1Angle, double armSeg2Angle) {
    armSeg2Angle *= -1;
    double l1 = cnst.ARM_SEG1_LENGTH;
    double l2 = cnst.ARM_SEG2_LENGTH;

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
    return forK(armSeg1Encoder.getDistance(), armSeg2Encoder.getDistance());
  }

  /** @return The angle that the end of the arm makes with the robot horizontal */
  public double getEndAngle() {
    double armSeg2Angle = armSeg2Encoder.getDistance();
    return armSeg2Angle - 90;
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
  private Pair<Double, Double> invK(double x, double y) {
    double xSign = Math.signum(x);
    x = Math.abs(x);

    double L1 = cnst.ARM_SEG1_LENGTH;
    double L2 = cnst.ARM_SEG2_LENGTH;

    double r = Math.hypot(x, y);

    double theta = Math.atan2(y, x);
    double alpha = invCosRule(L1, r, L2);
    double beta = invCosRule(L2, L1, r);

    alpha = Math.toDegrees(alpha);
    beta = Math.toDegrees(beta);
    theta = Math.toDegrees(theta);

    double armSeg1Angle = xSign * (alpha + theta - 90);
    double armSeg2Angle = xSign * beta + armSeg1Angle;

    // Return previous results if coordinates are invalid
    if (Double.isNaN(armSeg2Angle) || Double.isNaN(armSeg1Angle)) {
      return new Pair<Double, Double>(armSeg1TargetAngle, armSeg2TargetAngle);
    }

    return new Pair<Double, Double>(armSeg1Angle, armSeg2Angle);
  }

  /**
   * Inverts the arm.
   * 
   * @apiNote Disabled until further notice.
   */
  public void invert() {
    // Disable inversion until confirmed to be good
    if (true) {
      return;
    }
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
   * @param armSeg1 The target angle for the upper arm
   * @param armSeg2  The target angle for the fore arm
   */
  private void setAngles(double armSeg1, double armSeg2) {
    if (!calibrated) {
      Logger.warn("ArmSub : Arm hasn't been calibrated yet!");
    }

    armSeg1TargetAngle = MathUtil.clamp(armSeg1, -85, 85);
    armSeg2TargetAngle = MathUtil.clamp(armSeg2, -180, 180);

    armSeg1Controller.setSetpoint(armSeg1TargetAngle);
    ArmSeg2Controller.setSetpoint(armSeg2TargetAngle);
  }

  /**
   * Sets the target coordinates for the arm.
   * 
   * @param x The target x
   * @param y The target y
   */
  public void setCoord(Pair<Double, Double> coord, double x, double y, boolean invertable) {
    if (!targetIsValid(new Pair<Double, Double>(x, y))) { // if target coord is not allowed stay still
      Logger.warn("ArmSub : setCoords : dest is not allowed");
      Pair<Double, Double> tmp = getCurTarget();
      x = tmp.getFirst();
      y = tmp.getSecond();
    }
    int inv = invert && invertable ? -1 : 1;
    targets.set(targets.indexOf(coord), new Pair<Double, Double>(x * inv, y));
    Pair<Double, Double> tmp = getCurTarget();
    Pair<Double, Double> res = invK(tmp.getFirst(), tmp.getSecond());
    setAngles(res.getFirst(), res.getSecond());
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

    if (!targetIsValid(new Pair<Double, Double>(x, y))) { // if target coord is not allowed stay still
      Logger.warn("ArmSub : setCoords : dest is not allowed");
      Pair<Double, Double> tmp = getCurTarget();
      x = tmp.getFirst();
      y = tmp.getSecond();
    }
    int inv = invert && invertable ? -1 : 1;
    targets.add(idx, new Pair<Double, Double>(x * inv, y));
    Pair<Double, Double> tmp = getCurTarget();
    Pair<Double, Double> res = invK(tmp.getFirst(), tmp.getSecond());
    setAngles(res.getFirst(), res.getSecond());
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
    return moveTo(1, 0.0);
  }

  /** @return A {@link Command} that moves the arm to the far field position */
  public Command farField() {
    return moveTo(1.5, 0.0);
  }

  /** @return a {@link Command} that moves the arm to the lower rung position */
  public Command lowerRung() {
    return moveTo(1, 1);
  }

  /** @return a {@link Command} that moves the arm to the upper rung position */
  public Command upperRung() {
    return moveTo(1.4, 1.4);
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

          return ArmSeg2Controller.atSetpoint() && armSeg1Controller.atSetpoint();
        }, this);
  }

  /**
   * Zeroes the upper and fore arm encoders.
   * Use this method when the upper arm pointing up and the fore arm is pointed
   * down.
   */
  public void calibrate() {
    armSeg1Encoder.reset();
    armSeg2Encoder.reset();
    calibrated = true;
    setAngles(cnst.ARM_REACH_EXCLUSION[0][1], 0);
    // addCoord(0, cnst.ARM_REACH_EXCLUSION[0][0], cnst.ARM_SWING_THROUGH_HEIGHT,
    // false);
    setDestCoord(0, 0, false);
    Logger.info("ArmSub : Calibrated!");
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

  /** Kills the robot if an illegal arm angle is reached. */
  private void killCheck() {
    double armSeg1Angle = armSeg1Encoder.getDistance();
    double armSeg2Angle = armSeg2Encoder.getDistance();

    if (Math.abs(armSeg1Angle) > 90 || Math.abs(armSeg2Angle) > 185) {
      throw new IllegalStateException(
          String.format("ArmSub::killCheck : Illegal angles reached, killing robot! (armSeg2: %f, armSeg1: %f)",
              armSeg2Angle, armSeg1Angle));
    }
  }

  boolean targetIsValid(Pair<Double, Double> to) {
    return targetIsValid(to, getCurTarget());
  }

  boolean targetIsValid(Pair<Double, Double> to, Pair<Double, Double> at) {
    return targetIsValid(to.getFirst(), to.getSecond(), at.getFirst(), at.getSecond());
  }

  /**
   * Checks if the given coordinates are valid for the arm.
   * 
   * @param toX    The coordinate x
   * @param toY    The coordinate y
   * @param strict true prevents the robot from entering illegal space,
   *               false prevents the robot reaching too far but not from hitting
   *               itself
   * @return True if valid, false if not
   */
  boolean targetIsValid(double toX, double toY, double atX, double atY) {
    // check legal reach limits (target is outside lim and not improving)
    if ((toX < cnst.MAX_ARM_REACH_LEGAL[0][0] && toX <= atX) ||
        (toX > cnst.MAX_ARM_REACH_LEGAL[0][1] && toX >= atX)) {
      ShuffleControl.armTab.setCheckOne(false);
      return false;
    }
    ShuffleControl.armTab.setCheckOne(true);
    if ((toY < cnst.MAX_ARM_REACH_LEGAL[1][0] && toY <= atY) ||
        (toY > cnst.MAX_ARM_REACH_LEGAL[1][1] && toY >= atY)) {
      ShuffleControl.armTab.setCheckTwo(false);
      return false;
    }
    ShuffleControl.armTab.setCheckTwo(true);

    // max arm reach is around the axle x is around robot center x is adjusted to be
    // around axle before checking
    if (Math.hypot(toX + cnst.ARM_SEG1_X_OFFSET, toY) > cnst.MAX_ARM_REACH_PHYSICAL &&
        Math.hypot(toX + cnst.ARM_SEG1_X_OFFSET, toY) >= Math.hypot(atX + cnst.ARM_SEG1_X_OFFSET, atY)) {
      ShuffleControl.armTab.setCheckThree(false);
      return false;
    }
    ShuffleControl.armTab.setCheckThree(true);

    // check arm swing-through bounds
    double upperPas = cnst.ARM_SWING_THROUGH_HEIGHT * 1.05;
    double lowerPas = cnst.ARM_SWING_THROUGH_HEIGHT * 0.95;
    if (toX > cnst.ARM_REACH_EXCLUSION[0][0] &&
        toX < cnst.ARM_REACH_EXCLUSION[0][1] &&
        Math.abs(toX) >= Math.abs(atX) && // further from 0 than cur (can cross 0)
        ((atY > upperPas && toY < lowerPas) || // dont cross into the lower exclusion
            (atY < lowerPas && atY > upperPas) || // dont cross into the upper exclusion
            (toY > upperPas && toY >= atY) ||
            (atY < lowerPas && toY <= atY))) {

      ShuffleControl.armTab.setCheckFour(false);
      return false;
    }
    ShuffleControl.armTab.setCheckFour(true);

    // check robot limits
    if (toX > cnst.ARM_REACH_ROBOT_EXCLUSION[0][0] &&
        toX < cnst.ARM_REACH_ROBOT_EXCLUSION[0][1] &&
        toY < cnst.ARM_REACH_ROBOT_EXCLUSION[1][1] &&
        Math.abs(toX) >= Math.abs(atX) && // further from 0 than cur (can cross 0)
        toY >= atY) {
      ShuffleControl.armTab.setCheckFive(false);
      return false;
    }
    ShuffleControl.armTab.setCheckFive(true);

    return true;
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

  /**
   * Interpolates the path to the destination point.
   * 
   * @return The next interpolated point to move to.
   */
  Pair<Double, Double> interpolateNext() {
    Pair<Double, Double> curPos = forK();
    double x1 = curPos.getFirst();
    double y1 = curPos.getSecond();

    Pair<Double, Double> destPos = getFinalTarget();
    double x2 = destPos.getFirst();
    double y2 = destPos.getSecond();

    double angle = Math.atan2(y2 - y1, x2 - x1);

    double changeX = cnst.ARM_INTERPOLATION_STEP * Math.cos(angle);
    double changeY = cnst.ARM_INTERPOLATION_STEP * Math.sin(angle);

    if (Math.hypot(changeX, changeY) < cnst.ARM_INTERPOLATION_STEP) {
      return destPos;
    }

    double x = changeX + x1;
    double y = changeY + y1;

    return new Pair<Double, Double>(x, y);
  }

  @Override
  public void periodic() {
    // PRAY TO GOD THAT THIS CODE WORKS.

    if (!calibrated) {
      // Don't do anything if no calibration has happened.
      updateShuffleboard(0, 0);
      return;
    }
    if(DriverStation.isDisabled()){//clear intergral when disabled
      armSeg1Controller.reset();
    }

    // killCheck();

    Pair<Double, Double> kinematicsCoords = forK(armSeg1Encoder.getDistance(), armSeg2Encoder.getDistance());
    ShuffleControl.armTab.setKinematicsCoords(-kinematicsCoords.getFirst(), kinematicsCoords.getSecond());

    double armSeg1Output = armSeg1Controller.calculate(armSeg1Encoder.getDistance() * -1);
    double armSeg2Output = ArmSeg2Controller.calculate(armSeg2Encoder.getDistance());

    armSeg1Output = MathUtil.clamp(armSeg1Output, -0.3, 0.3);
    armSeg2Output = MathUtil.clamp(armSeg2Output, -0.3, 0.3);

    if (!(armSeg1Controller.atSetpoint() && ArmSeg2Controller.atSetpoint())) {
      Pair<Double, Double> nextPoint = getCurTarget();//interpolateNext();
      Pair<Double, Double> res = invK(nextPoint.getFirst(), nextPoint.getSecond());
      setAngles(res.getFirst(), res.getSecond());
    }

    if (!armSeg1Controller.atSetpoint()) {
      armSeg1Motors.set(seg1Curve.fit(MathUtil.applyDeadband(armSeg1Output, 0.01)));
    } else {
      armSeg1Motors.stopMotor();

      if (ArmSeg2Controller.atSetpoint() && prevArmTargetPoints != getFinalTarget()) { // if at both setpoints
        if (targets.size() > 1)
          targets.remove(0);
        Pair<Double, Double> tmp = getCurTarget();
        Pair<Double, Double> res = invK(tmp.getFirst(), tmp.getSecond());
        setAngles(res.getFirst(), res.getSecond());
      }
    }

    if (!ArmSeg2Controller.atSetpoint()) {
      armSeg2Motors.set(seg2Curve.fit(MathUtil.applyDeadband(armSeg2Output, 0.01)));
    } else {
      armSeg2Motors.stopMotor();
    }
    updateShuffleboard(armSeg1Output, armSeg2Output);
  }
}
