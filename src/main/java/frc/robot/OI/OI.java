package frc.robot.OI;

import java.util.Optional;

/**
 * OI - Use this class to access and initialize all controller-related stuff.
 */
public class OI {
  private static Optional<OI> inst = Optional.empty();
  public static OI getInstance() {
    if (!inst.isPresent()) inst = Optional.of(new OI());
    return inst.get();
  }

  /////////

  public final XboxCtrl xBox1 = new XboxCtrl();

}
