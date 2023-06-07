package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class CrateSub extends SubsystemBase {
  private final DoubleSolenoid solenoid = new DoubleSolenoid(
      Constants.features.PCM_ID,
      Constants.features.PCM,
      Constants.crate.PORT_1,
      Constants.crate.PORT_2);

  public CrateSub() {
    // solenoid needs an initial direction for toggle() to work
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /** Toggles the crate solenoid */
  public void toggle() {
    solenoid.toggle();
  }

  /** Extends the crate solenoid */
  public void extend() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  /** Retracts the crate solenoid */
  public void retract() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * @return a {@link Command} that extends the solenoid at start, and retracts it
   *         on end
   */
  public Command shoot() {
    return this.startEnd(this::extend, this::retract);
  }

  // Simulation
  private final DoubleSolenoidSim solenoidSim = new DoubleSolenoidSim(
      Constants.features.PCM_ID,
      Constants.features.PCM,
      Constants.crate.PORT_1,
      Constants.crate.PORT_2);

  @Override
  public void simulationPeriodic() {
    solenoidSim.set(solenoid.get());
  }
}
