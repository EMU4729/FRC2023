package frc.robot;

import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.GripperSub;
import frc.robot.subsystems.NavigationSub;
import frc.robot.subsystems.SubArmPivotSub;
import frc.robot.subsystems.SubArmRotateSub;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  public static final NavigationSub nav = new NavigationSub();
  public static final DriveSub drive = new DriveSub();
  public static final GripperSub gripper = new GripperSub();
  public static final SubArmPivotSub subArmPivot = new SubArmPivotSub();
  public static final SubArmRotateSub subArmRotate = new SubArmRotateSub();
  public static final ArmSub arm = new ArmSub();

  /** Calibrates all subsystems. */
  public static void calibrate() {
    subArmPivot.calibrate();
    subArmRotate.calibrate();
    arm.calibrate();
  }
}
