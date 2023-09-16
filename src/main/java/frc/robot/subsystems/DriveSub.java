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
import frc.robot.constants.Constants;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DriveSub extends SubsystemBase {
  private final MotorController leftMaster = Constants.drive.MOTOR_ID_LM.build();
  private final MotorController leftSlave = Constants.drive.MOTOR_ID_LS.build();
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave);

  private final MotorController rightMaster = Constants.drive.MOTOR_ID_RM.build();
  private final MotorController rightSlave = Constants.drive.MOTOR_ID_RS.build();
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave);

  public final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors); // pub for shuffleboard

  // Simulation Variables
  /** @wip add corrected values */
  private final LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
      Constants.sim.KV_LINEAR,
      Constants.sim.KA_LINEAR,
      Constants.sim.KV_ANGULAR,
      Constants.sim.KA_ANGULAR);
  public final DifferentialDrivetrainSim drivetrainSimulator = new DifferentialDrivetrainSim(
      drivetrainSystem, DCMotor.getCIM(2), 10.71, Constants.features.ROBOT_WHEEL_WIDTH,
      Constants.features.ROBOT_WHEEL_RAD, null);

  public DriveSub() {
    addChild("Differential Drive", drive);
  }

  /**
   * Tank drive.
   * 
   * @param leftSpeed  The left speed.
   * @param rightSpeed The right speed.
   */
  public void tank(double leftSpeed, double rightSpeed) {
    leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
    rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
    drive.tankDrive(leftSpeed, rightSpeed, false);
  }

  /**
   * Tank drives the robot using the specified voltages.
   * <strong>Highly unsafe.</strong> Values are uncapped, so use with caution.
   * 
   * @param leftVoltage  The left output
   * @param rightVoltage The right output
   */
  public void tankVoltage(double leftVoltage, double rightVoltage) {
    leftMotors.setVoltage(leftVoltage);
    rightMotors.setVoltage(rightVoltage);
    drive.feed();
  }

  /**
   * Arcade drive.
   * 
   * @param throttle The speed
   * @param steering The steering
   */
  public void arcade(double throttle, double steering) {
    throttle = MathUtil.clamp(throttle, -1, 1);
    steering = -MathUtil.clamp(steering, -1, 1);
    drive.arcadeDrive(throttle, -steering, true); // squared input fix later
  }

  /** Stops all motors. */
  public void off() {
    tank(0, 0);
  }

  @Override
  public void simulationPeriodic() {
    // set sim motor volts to cur motor throt * bat volts

    // the order is reversed because otherwise, simulation direction is the opposite of real life
    // this is probably a result of a deeper issue with the code that i don't want to fix right now
    drivetrainSimulator.setInputs(
        rightMotors.get() * RobotController.getInputVoltage(),
        leftMotors.get() * RobotController.getInputVoltage()
    );
    drivetrainSimulator.update(0.02);
  }
}
