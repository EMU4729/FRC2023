package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.simulation.SimConstants;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DriveSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();
  private final SimConstants simCnst = SimConstants.getInstance();
  // private final Variables vars = Variables.getInstance();

  private final MotorController leftMaster = cnst.DRIVE_MOTOR_ID_LM.createMotorController();
  private final MotorController leftSlave = cnst.DRIVE_MOTOR_ID_LS.createMotorController();
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave);

  private final MotorController rightMaster = cnst.DRIVE_MOTOR_ID_RM.createMotorController();
  private final MotorController rightSlave = cnst.DRIVE_MOTOR_ID_RS.createMotorController();
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave);

  public final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors); // pub for shuffleboard

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Simulation Variables

  /** @wip add corrected values */
  private final LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
      simCnst.KV_LINEAR,
      simCnst.KA_LINEAR,
      simCnst.KV_ANGULAR,
      simCnst.KA_ANGULAR);
  public final DifferentialDrivetrainSim drivetrainSimulator = new DifferentialDrivetrainSim(
      drivetrainSystem, DCMotor.getCIM(2), 10.71, cnst.ROBOT_WHEEL_WIDTH, cnst.ROBOT_WHEEL_RAD, null);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  public DriveSub() {
    addChild("Differential Drive", drive);
  }

  /**
   * Activates tank drive. Similar to MoveTank from ev3dev.
   * 
   * @param leftSpeed  The left speed.
   * @param rightSpeed The right speed.
   */
  public void tank(double leftSpeed, double rightSpeed) {
    leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
    rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Activates arcade drive. Similar to MoveSteering from ev3dev.
   * 
   * @param throttle The speed
   * @param steering The steering
   */
  public void arcade(double throttle, double steering) {
    throttle = MathUtil.clamp(throttle, -1, 1);
    steering = MathUtil.clamp(steering, -1, 1);
    drive.arcadeDrive(throttle, steering);

  }

  /** Stop all motors. */
  public void off() {
    drive.stopMotor();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Simulation Functions

  @Override
  public void simulationPeriodic() {
    // set sim motor volts to cur motor throt * bat volts
    drivetrainSimulator.setInputs(
        leftMotors.get() * RobotController.getInputVoltage(),
        rightMotors.get() * RobotController.getInputVoltage());
    drivetrainSimulator.update(0.02);
  }
}
