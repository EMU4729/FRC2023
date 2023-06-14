package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;

public class FeaturesConstants {

  protected FeaturesConstants() {
  }

  /** distance between wheel center side to side (m) */
  public final double ROBOT_WHEEL_WIDTH = 0.870;
  /** radius of the drive wheels (m) */
  public final double ROBOT_WHEEL_RAD = Units.inchesToMeters(3);
  /** length of the robot frame (m) */
  public final double ROBOT_LENGTH = Units.inchesToMeters(28.3);
  /** width of the robot frame (m) @wip check num */
  public final double ROBOT_WIDTH = Units.inchesToMeters(24);
  /** max reach outside frame perim [x(from frame),y(from floor)] (m) */
  public final double[] ROBOT_REACH_MAX = {
      (Units.inchesToMeters(48)),
      (Units.inchesToMeters(78))
  };

  /** Power distribution object */
  public final PowerDistribution PDB = new PowerDistribution();

  /** PCM Type */
  public final PneumaticsModuleType PCM = PneumaticsModuleType.CTREPCM;

  /** PCM CAN ID @wip */
  public final int PCM_ID = 5;
}
