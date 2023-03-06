package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.utils.logger.Logger;

/**
 * Moves the arm to far field position then retracts backwards as the pilot
 * moves the robot forwards.
 */
public class ArmPickUp extends CommandBase {
  Pose2d startPose = Subsystems.nav.getPose();

  public ArmPickUp() {
    addRequirements(Subsystems.arm);
    andThen(Subsystems.arm.lowField());
  }

  public double distanceFromStart() {
    Pose2d currentPose = Subsystems.nav.getPose();
    return Math.sqrt(
        Math.pow(startPose.getX() - currentPose.getX(), 2)
            + Math.pow(startPose.getY() - currentPose.getY(), 2));
  }

  @Override
  public void initialize() {
    startPose = Subsystems.nav.getPose();
  }

  @Override
  public void execute() {
    double distance = distanceFromStart();
    Subsystems.arm.setDestCoord(2 - distance, 1.5, true);
  }

  @Override
  public boolean isFinished() {
    if (RobotBase.isSimulation()) {
      Logger.info("ArmPickUp : In simulation, skipping...");
      return true;
    }

    double distance = distanceFromStart();
    return distance <= 0.5 || distance >= 2.5;
  }
}
