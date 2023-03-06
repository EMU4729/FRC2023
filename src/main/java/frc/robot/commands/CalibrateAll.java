package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;

/** Calibrates all subsystems at once. */
public class CalibrateAll extends InstantCommand {
  public CalibrateAll() {
    super(() -> {
      Subsystems.arm.calibrate();
      Subsystems.subArmPivot.calibrate();
      Subsystems.subArmRotate.calibrate();
    },
        Subsystems.arm,
        Subsystems.subArmPivot,
        Subsystems.subArmRotate);
  }
}
