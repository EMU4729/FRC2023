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
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.utils.logger.Logger;

public class PathWeaverCommand extends SequentialCommandGroup {
  public final Constants cnst = Constants.getInstance();

  public PathWeaverCommand(String pathweaverPath) {
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathweaverPath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      Logger.error("PathWeaverCommand : Unable to open trajectory: " + pathweaverPath + e);
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        Subsystems.nav::getPose,
        new RamseteController(
            cnst.DRIVE_RAMSETE_B,
            cnst.DRIVE_RAMSETE_ZETA),
        new SimpleMotorFeedforward(
            cnst.DRIVE_KS_VOLTS,
            cnst.DRIVE_KV_VOLT_SECONDS_PER_METER,
            cnst.DRIVE_KA_VOLT_SECONDS_SQUARED_PER_METER),
        cnst.DRIVE_KINEMATICS,
        Subsystems.nav::getWheelSpeeds,
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        Subsystems.drive::tankVoltage,
        Subsystems.drive);

    addCommands(ramseteCommand, new InstantCommand(Subsystems.drive::off));
  }
}
