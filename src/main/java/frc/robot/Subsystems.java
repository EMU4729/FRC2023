package frc.robot;

import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.GripperGripSub;
import frc.robot.subsystems.GripperPosSub;
import frc.robot.subsystems.NavigationSub;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  public static final NavigationSub nav = new NavigationSub();
  public static final DriveSub drive = new DriveSub();
  public static final GripperGripSub gripperGrip = new GripperGripSub();
  public static final GripperPosSub gripperPos = new GripperPosSub();
  public static final ArmSub arm = new ArmSub();
}
