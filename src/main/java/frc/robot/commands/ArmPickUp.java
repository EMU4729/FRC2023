package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

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
    Subsystems.arm.setCoords(2 - distance, 1.5);
  }

  @Override
  public boolean isFinished() {
    double distance = distanceFromStart();
    return distance <= 0.5 || distance >= 2.5;
  }
}
