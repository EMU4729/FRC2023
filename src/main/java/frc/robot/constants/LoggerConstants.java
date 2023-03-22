package frc.robot.constants;

public class LoggerConstants {
  protected LoggerConstants() {
  }

  /** limit for repeated attempts to create log file on USB storage */
  public final int REPEAT_LIMIT_LOGGER_CREATION = 10;
  /** limit for repeated attempts to read auto from internal storage */
  public final int REPEAT_LIMIT_AUTO_READ = 10;
  /** save attempts per second for the logger */
  public final int SAVE_RATE = 10;

}
