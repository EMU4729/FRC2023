package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class DefaultAuto extends SequentialCommandGroup {
  public final Constants cnst = Constants.getInstance();

  public DefaultAuto() {
    addCommands(new PathWeaverCommand(cnst.PATHWEAVER_PATH));
  }
}
