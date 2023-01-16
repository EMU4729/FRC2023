package frc.robot.OI;

import java.util.Optional;

/**
 * OI - Use this class to access and initialize all controller-related stuff.
 */
public class OICtrl {
  private static Optional<OICtrl> inst = Optional.empty();
  public static OICtrl getInstance() {
    if (!inst.isPresent()) inst = Optional.of(new OICtrl());
    return inst.get();
  }

  /////////

  public final XboxCtrl xBox1 = new XboxCtrl();

}
