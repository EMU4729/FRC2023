package frc.robot.utils.logger;

import edu.wpi.first.wpilibj.RobotController;

public class LogLine {
  private final LogLevel level;
  private final String content;

  public LogLine(String content, LogLevel level) {
    this.level = level;
    this.content = content;
  }

  public String toString() {
    String levelString = "[" + level.toString() + "]";
    String timeString = String.valueOf((int) (RobotController.getFPGATime()));

    return String.format("%s %s %s", levelString, timeString, content);
  }

}