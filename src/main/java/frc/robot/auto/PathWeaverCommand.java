package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.constants.Constants;
import frc.robot.utils.logger.Logger;

/** Command that runs a PathWeaver path. */
public class PathWeaverCommand extends SequentialCommandGroup {
  /**
   * Constructs a new {@link PathWeaverCommand}.
   * 
   * @param pathweaverPath The path of the pathweaver file, relative to the
   *                       robot's deploy directory.
   */
  public PathWeaverCommand(String pathweaverPath) {
    // The following code was adapted from the code at
    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-following-trajectory.html

    Trajectory trajectory = new Trajectory();

    // Reads trajectory from the specified filepath
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathweaverPath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      Logger.info("PathWeaverCommand : Loaded path " + trajectoryPath);
    } catch (IOException e) {
      Logger.error("PathWeaverCommand : Unable to open trajectory: " + pathweaverPath + e);
    }

    // Don't even ask me what this does
    // All I know is that it works and no one should touch it
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        Subsystems.nav::getPose,
        new RamseteController(
            Constants.drive.RAMSETE_B,
            Constants.drive.RAMSETE_ZETA),
        new SimpleMotorFeedforward(
            Constants.drive.KS_VOLTS,
            Constants.drive.KV_VOLT_SECONDS_PER_METER,
            Constants.drive.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.drive.KINEMATICS,
        Subsystems.nav::getWheelSpeeds,
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        Subsystems.drive::tankVoltage,
        Subsystems.drive);

    addCommands(ramseteCommand, new InstantCommand(Subsystems.drive::off));
  }
}
